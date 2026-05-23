/*
 * ethernet.c
 *
 *  Created on: May 23, 2026
 *      Author: kyubeom
 *
 * UDP packet receiver for steering control.
 *
 * Packet format:
 *
 * - ASMS packet: 5 bytes
 *   ASMS는 상위제어 용어 기준의 수동 조작기 / 모드 전환 source이다.
 *   조이스틱 ADC raw 값을 목표 조향각으로 변환하여 MANUAL 모드에서 각도 추종한다.
 *
 *   byte[0]   : mode
 *   byte[1]   : unused
 *   byte[2]   : unused
 *   byte[3:4] : joystick ADC raw, little-endian uint16_t
 *
 * - PC packet: 9 bytes
 *   PC는 상위제어 용어 기준의 자율주행 상위제어 / AUTO steering command source이다.
 *
 *   byte[0:3] : steering raw, little-endian int32_t
 *   byte[4:7] : speed raw, little-endian uint32_t
 *   byte[8]   : misc, bit7 = emergency stop
 */

#include "ethernet.h"

#include "main.h"
#include "config.h"

#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"

#include <stdbool.h>

/* =========================================
 * Private variables
 * ========================================= */

static struct udp_pcb *ethernet_udp_pcb = NULL;

static volatile bool        ethernet_initialized = false;
static volatile bool        ethernet_new_data = false;
static volatile bool        ethernet_emergency_request = false;
static volatile SteerMode_t ethernet_current_mode = STEER_MODE_NONE;
static volatile uint32_t    ethernet_last_rx_tick = 0U;

static Ethernet_Packet_t ethernet_latest_packet = {
    .source = ETHERNET_SOURCE_NONE,
    .steering_deg = 0.0f,
    .asms_adc_raw = 0U,
    .pc_steer_raw = 0,
    .speed_raw = 0U,
    .misc = 0U
};

/* =========================================
 * Utility functions
 * ========================================= */

static float Ethernet_ClampFloat(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }

    if (value < min_value) {
        return min_value;
    }

    return value;
}

/*
 * Clamp by ASMS manual steering limit.
 */
static float Ethernet_ClampAsmsSteeringDeg(float value)
{
    return Ethernet_ClampFloat(value,
                               ETHERNET_ASMS_MIN_STEERING_DEG,
                               ETHERNET_ASMS_MAX_STEERING_DEG);
}

/*
 * Clamp by PC AUTO steering limit.
 */
static float Ethernet_ClampPcSteeringDeg(float value)
{
    return Ethernet_ClampFloat(value,
                               ETHERNET_PC_MIN_STEERING_DEG,
                               ETHERNET_PC_MAX_STEERING_DEG);
}

/*
 * Clamp by mechanical steering limit.
 *
 * source별 제한을 거친 값이라도 최후 방어용으로 기계적 한계를 다시 적용한다.
 * 실제 control.c에서는 CONTROL_TARGET_MAX/MIN_STEERING_DEG 기준으로 한 번 더 제한된다.
 */
static float Ethernet_ClampMechanicalSteeringDeg(float value)
{
    return Ethernet_ClampFloat(value,
                               STEERING_MECHANICAL_MIN_DEG,
                               STEERING_MECHANICAL_MAX_DEG);
}

static uint16_t Ethernet_ReadUint16LE(const uint8_t *buf)
{
    return (uint16_t)(((uint16_t)buf[0]) |
                     ((uint16_t)buf[1] << 8));
}

static int32_t Ethernet_ReadInt32LE(const uint8_t *buf)
{
    return (int32_t)(((uint32_t)buf[0]) |
                    ((uint32_t)buf[1] << 8) |
                    ((uint32_t)buf[2] << 16) |
                    ((uint32_t)buf[3] << 24));
}

static uint32_t Ethernet_ReadUint32LE(const uint8_t *buf)
{
    return ((uint32_t)buf[0]) |
           ((uint32_t)buf[1] << 8) |
           ((uint32_t)buf[2] << 16) |
           ((uint32_t)buf[3] << 24);
}

/* =========================================
 * Conversion functions
 * ========================================= */

/*
 * Convert ASMS joystick ADC raw to steering target [deg].
 *
 * 전제:
 * - ASMS ADC raw는 unsigned 값이다.
 * - 중립값은 ETHERNET_ASMS_ADC_CENTER_RAW이다.
 * - ADC raw가 center보다 크면 양의 조향각, 작으면 음의 조향각으로 변환한다.
 * - 방향이 반대이면 ETHERNET_ASMS_POLARITY를 -1로 설정한다.
 */
static float Ethernet_AsmsAdcToSteeringDeg(uint16_t adc_raw)
{
    int32_t offset;
    int32_t positive_span;
    int32_t negative_span;
    float normalized;
    float steering_deg;

    if (adc_raw > ETHERNET_ASMS_ADC_MAX_RAW) {
        adc_raw = ETHERNET_ASMS_ADC_MAX_RAW;
    }

    if (adc_raw < ETHERNET_ASMS_ADC_MIN_RAW) {
        adc_raw = ETHERNET_ASMS_ADC_MIN_RAW;
    }

    offset = (int32_t)adc_raw - (int32_t)ETHERNET_ASMS_ADC_CENTER_RAW;

    /*
     * Deadband.
     *
     * 조이스틱 중립 근처 ADC 노이즈를 0으로 처리한다.
     */
    if ((offset < (int32_t)ETHERNET_ASMS_ADC_DEADBAND_RAW) &&
        (offset > -(int32_t)ETHERNET_ASMS_ADC_DEADBAND_RAW)) {
        offset = 0;
    }

    positive_span =
        (int32_t)ETHERNET_ASMS_ADC_MAX_RAW -
        (int32_t)ETHERNET_ASMS_ADC_CENTER_RAW;

    negative_span =
        (int32_t)ETHERNET_ASMS_ADC_CENTER_RAW -
        (int32_t)ETHERNET_ASMS_ADC_MIN_RAW;

    if ((positive_span <= 0) || (negative_span <= 0)) {
        return 0.0f;
    }

    if (offset >= 0) {
        normalized = (float)offset / (float)positive_span;
    } else {
        normalized = (float)offset / (float)negative_span;
    }

    /*
     * normalized 범위: -1.0 ~ +1.0
     * steering_deg 범위: ETHERNET_ASMS_MIN/MAX_STEERING_DEG
     */
    steering_deg =
        (float)ETHERNET_ASMS_POLARITY *
        ETHERNET_ASMS_MAX_STEERING_DEG *
        normalized;

    return Ethernet_ClampAsmsSteeringDeg(steering_deg);
}

/* =========================================
 * Packet handling functions
 * ========================================= */

static void Ethernet_SavePacket(Ethernet_Source_t source,
                                float steering_deg,
                                uint16_t asms_adc_raw,
                                int32_t pc_steer_raw,
                                uint32_t speed_raw,
                                uint8_t misc)
{
    ethernet_latest_packet.source = source;
    ethernet_latest_packet.steering_deg =
        Ethernet_ClampMechanicalSteeringDeg(steering_deg);

    ethernet_latest_packet.asms_adc_raw = asms_adc_raw;
    ethernet_latest_packet.pc_steer_raw = pc_steer_raw;
    ethernet_latest_packet.speed_raw = speed_raw;
    ethernet_latest_packet.misc = misc;

    ethernet_last_rx_tick = HAL_GetTick();
    ethernet_new_data = true;
}

/*
 * ASMS packet 처리.
 *
 * ASMS packet은 수동 조이스틱 ADC 입력뿐 아니라
 * AUTO / MANUAL / ESTOP 모드 전환도 담당한다.
 */
static void Ethernet_ProcessAsmsPacket(const uint8_t *buf)
{
    uint8_t mode;
    uint16_t asms_adc_raw;

    mode = buf[0];
    asms_adc_raw = Ethernet_ReadUint16LE(&buf[3]);

    if ((mode == (uint8_t)STEER_MODE_AUTO) ||
        (mode == (uint8_t)STEER_MODE_MANUAL) ||
        (mode == (uint8_t)STEER_MODE_ESTOP)) {
        ethernet_current_mode = (SteerMode_t)mode;
    }

    ethernet_last_rx_tick = HAL_GetTick();

    if (ethernet_current_mode == STEER_MODE_MANUAL) {
        float steering_deg;

        steering_deg = Ethernet_AsmsAdcToSteeringDeg(asms_adc_raw);

        Ethernet_SavePacket(ETHERNET_SOURCE_ASMS,
                            steering_deg,
                            asms_adc_raw,
                            0,
                            0U,
                            0U);
    } else if (ethernet_current_mode == STEER_MODE_ESTOP) {
        ethernet_emergency_request = true;
    }
}

/*
 * PC packet 처리.
 *
 * PC packet은 AUTO 모드에서 자율주행 상위제어의
 * steering command로만 사용한다.
 */
static void Ethernet_ProcessPcPacket(const uint8_t *buf)
{
    int32_t pc_steer_raw;
    uint32_t pc_speed_raw;
    uint8_t pc_misc;
    float steering_deg;

    /*
     * PC steering command는 AUTO 모드에서만 인정한다.
     * ASMS packet을 통해 AUTO 모드로 전환되기 전까지는 PC packet을 무시한다.
     */
    if (ethernet_current_mode != STEER_MODE_AUTO) {
        return;
    }

    pc_steer_raw = Ethernet_ReadInt32LE(&buf[0]);
    pc_speed_raw = Ethernet_ReadUint32LE(&buf[4]);
    pc_misc = buf[8];

    /*
     * PC misc bit7 = emergency stop.
     */
    if ((pc_misc & 0x80U) != 0U) {
        ethernet_current_mode = STEER_MODE_ESTOP;
        ethernet_emergency_request = true;
        ethernet_last_rx_tick = HAL_GetTick();
        return;
    }

    /*
     * Convert PC steering raw to steering target [deg].
     *
     * ETHERNET_PC_STEER_SCALE은 상위제어 packet 단위에 맞춰야 한다.
     */
    steering_deg = (float)pc_steer_raw * ETHERNET_PC_STEER_SCALE;
    steering_deg = Ethernet_ClampPcSteeringDeg(steering_deg);

    Ethernet_SavePacket(ETHERNET_SOURCE_PC,
                        steering_deg,
                        0U,
                        pc_steer_raw,
                        pc_speed_raw,
                        pc_misc);
}

static void Ethernet_UdpRecvCallback(void *arg,
                                     struct udp_pcb *pcb,
                                     struct pbuf *p,
                                     const ip_addr_t *addr,
                                     u16_t port)
{
    uint16_t len;
    uint16_t copied;
    uint8_t buffer[16] = {0};
    uint8_t sender_last_octet = 0U;

    (void)arg;
    (void)pcb;
    (void)port;

    if (p == NULL) {
        return;
    }

    len = p->tot_len;

    if (len > sizeof(buffer)) {
        pbuf_free(p);
        return;
    }

    copied = pbuf_copy_partial(p, buffer, len, 0);
    pbuf_free(p);

    if (copied != len) {
        return;
    }

    if ((addr == NULL) || !IP_IS_V4(addr)) {
        return;
    }

    sender_last_octet = ip4_addr4(ip_2_ip4(addr));

#if ETHERNET_USE_IP_FILTER

    if ((len == ETHERNET_ASMS_PACKET_SIZE) &&
        (sender_last_octet == ETHERNET_ASMS_IP_LAST_OCTET)) {
        Ethernet_ProcessAsmsPacket(buffer);
        return;
    }

    if ((len == ETHERNET_PC_PACKET_SIZE) &&
        (sender_last_octet == ETHERNET_PC_IP_LAST_OCTET)) {
        Ethernet_ProcessPcPacket(buffer);
        return;
    }

#else

    /*
     * Bring-up mode.
     *
     * 초기 통신 확인 단계에서는 IP 마지막 옥텟을 보지 않고
     * packet size만으로 ASMS/PC packet을 구분한다.
     */
    if (len == ETHERNET_ASMS_PACKET_SIZE) {
        Ethernet_ProcessAsmsPacket(buffer);
        return;
    }

    if (len == ETHERNET_PC_PACKET_SIZE) {
        Ethernet_ProcessPcPacket(buffer);
        return;
    }

#endif
}

/* =========================================
 * Public functions
 * ========================================= */

void Ethernet_Init(void)
{
    ethernet_initialized = false;

    ethernet_new_data = false;
    ethernet_emergency_request = false;
    ethernet_current_mode = STEER_MODE_NONE;
    ethernet_last_rx_tick = 0U;

    ethernet_latest_packet.source = ETHERNET_SOURCE_NONE;
    ethernet_latest_packet.steering_deg = 0.0f;
    ethernet_latest_packet.asms_adc_raw = 0U;
    ethernet_latest_packet.pc_steer_raw = 0;
    ethernet_latest_packet.speed_raw = 0U;
    ethernet_latest_packet.misc = 0U;

    ethernet_udp_pcb = udp_new();

    if (ethernet_udp_pcb == NULL) {
        return;
    }

    if (udp_bind(ethernet_udp_pcb, IP_ADDR_ANY, ETHERNET_UDP_PORT) != ERR_OK) {
        udp_remove(ethernet_udp_pcb);
        ethernet_udp_pcb = NULL;
        return;
    }

    udp_recv(ethernet_udp_pcb, Ethernet_UdpRecvCallback, NULL);

    ethernet_initialized = true;
}

bool Ethernet_IsInitialized(void)
{
    return ethernet_initialized;
}

bool Ethernet_HasNewData(void)
{
    return ethernet_new_data;
}

Ethernet_Packet_t Ethernet_GetLatestData(void)
{
    Ethernet_Packet_t packet;

    packet = ethernet_latest_packet;
    ethernet_new_data = false;

    return packet;
}

SteerMode_t Ethernet_GetCurrentMode(void)
{
    return ethernet_current_mode;
}

bool Ethernet_ConsumeEmergencyRequest(void)
{
    bool request;

    request = ethernet_emergency_request;
    ethernet_emergency_request = false;

    return request;
}

uint32_t Ethernet_GetLastRxTick(void)
{
    return ethernet_last_rx_tick;
}
