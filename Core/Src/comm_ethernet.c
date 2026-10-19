/*
 * comm_ethernet.c
 *
 *  Created on: Aug 27, 2026
 *      Author: kyubeom
 */

#include "comm_ethernet.h"

#include "main.h"
#include "config.h"

#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"

#include <string.h>


/* =========================================
 * Private variables
 * ========================================= */

static struct udp_pcb *comm_eth_udp_pcb = NULL;

static bool comm_eth_initialized = false;


/*
 * latest-wins mailbox
 */
static CommEthernet_AsmsPacket_t comm_eth_asms_packet;
static CommEthernet_PcPacket_t   comm_eth_pc_packet;

static bool comm_eth_asms_pending = false;
static bool comm_eth_pc_pending   = false;


/*
 * Transport-level last RX timestamps.
 *
 * "형식과 source 검사를 통과해서 parser가 받은 시각"이다.
 * 실제 제어 timeout용 valid command timestamp와는 구분한다.
 */
static uint32_t comm_eth_last_asms_rx_tick = 0U;
static uint32_t comm_eth_last_pc_rx_tick   = 0U;


/*
 * E-Stop은 일반 command mailbox와 분리한다.
 *
 * manager가 Consume하기 전까지 유지된다.
 */
static uint8_t comm_eth_estop_pending_mask =
    COMM_ETHERNET_ESTOP_NONE;


/* =========================================
 * Little-endian readers
 * ========================================= */

static uint16_t CommEthernet_ReadUint16LE(
    const uint8_t *buf)
{
    return
        ((uint16_t)buf[0]) |
        ((uint16_t)buf[1] << 8);
}


static int16_t CommEthernet_ReadInt16LE(
    const uint8_t *buf)
{
    return (int16_t)CommEthernet_ReadUint16LE(buf);
}


static uint32_t CommEthernet_ReadUint32LE(
    const uint8_t *buf)
{
    return
        ((uint32_t)buf[0])       |
        ((uint32_t)buf[1] << 8)  |
        ((uint32_t)buf[2] << 16) |
        ((uint32_t)buf[3] << 24);
}


static int32_t CommEthernet_ReadInt32LE(
    const uint8_t *buf)
{
    return (int32_t)CommEthernet_ReadUint32LE(buf);
}

/* =========================================
 * Little-endian writers
 * ========================================= */

static void CommEthernet_WriteUint16LE(
    uint8_t *buf,
    uint16_t value)
{
    buf[0] = (uint8_t)(value & 0xFFU);
    buf[1] = (uint8_t)((value >> 8) & 0xFFU);
}


static void CommEthernet_WriteInt16LE(
    uint8_t *buf,
    int16_t value)
{
    CommEthernet_WriteUint16LE(
        buf,
        (uint16_t)value
    );
}


static void CommEthernet_WriteUint32LE(
    uint8_t *buf,
    uint32_t value)
{
    buf[0] = (uint8_t)(value & 0xFFU);
    buf[1] = (uint8_t)((value >> 8) & 0xFFU);
    buf[2] = (uint8_t)((value >> 16) & 0xFFU);
    buf[3] = (uint8_t)((value >> 24) & 0xFFU);
}

/* =========================================
 * Source validation
 * ========================================= */

static bool CommEthernet_IsAsmsSender(
    const ip_addr_t *addr)
{
#if ETHERNET_USE_IP_FILTER

    uint8_t sender_last_octet;

    if ((addr == NULL) || !IP_IS_V4(addr)) {
        return false;
    }

    sender_last_octet =
        ip4_addr4(ip_2_ip4(addr));

    return
        (sender_last_octet ==
         ETHERNET_ASMS_IP_LAST_OCTET);

#else

    (void)addr;

    return true;

#endif
}


static bool CommEthernet_IsPcSender(
    const ip_addr_t *addr)
{
#if ETHERNET_USE_IP_FILTER

    uint8_t sender_last_octet;

    if ((addr == NULL) || !IP_IS_V4(addr)) {
        return false;
    }

    sender_last_octet =
        ip4_addr4(ip_2_ip4(addr));

    return
        (sender_last_octet ==
         ETHERNET_PC_IP_LAST_OCTET);

#else

    (void)addr;

    return true;

#endif
}


/* =========================================
 * ASMS parser
 * ========================================= */

static void CommEthernet_ParseAsmsPacket(
    const uint8_t *buf)
{
    uint32_t now_ms;

    now_ms = HAL_GetTick();

    /*
     * Byte 0
     */
    comm_eth_asms_packet.mode_raw =
        buf[ETHERNET_ASMS_MODE_OFFSET];

    /*
     * Byte 1~2
     */
    comm_eth_asms_packet.speed_raw =
        CommEthernet_ReadUint16LE(
            &buf[ETHERNET_ASMS_SPEED_OFFSET]
        );

    /*
     * Byte 3~4
     */
    comm_eth_asms_packet.steer_raw =
        CommEthernet_ReadInt16LE(
            &buf[ETHERNET_ASMS_STEER_OFFSET]
        );

    comm_eth_asms_packet.rx_tick_ms = now_ms;

    comm_eth_last_asms_rx_tick = now_ms;

    /*
     * latest-wins
     */
    comm_eth_asms_pending = true;


    /*
     * ASMS E-Stop은 sticky 처리.
     *
     * ASMS protocol:
     * 1 = AUTO
     * 2 = MANUAL
     * 3 = ESTOP
     *
     * 상태전환 자체는 manager가 담당한다.
     */
    if (comm_eth_asms_packet.mode_raw == 3U) {
        comm_eth_estop_pending_mask |=
            COMM_ETHERNET_ESTOP_ASMS;
    }
}


/* =========================================
 * PC parser
 * ========================================= */

static void CommEthernet_ParsePcPacket(
    const uint8_t *buf)
{
    uint32_t now_ms;

    now_ms = HAL_GetTick();

    /*
     * Byte 0~3
     */
    comm_eth_pc_packet.steer_raw =
        CommEthernet_ReadInt32LE(
            &buf[ETHERNET_PC_STEER_OFFSET]
        );

    /*
     * Byte 4~7
     */
    comm_eth_pc_packet.speed_raw =
        CommEthernet_ReadUint32LE(
            &buf[ETHERNET_PC_SPEED_OFFSET]
        );

    /*
     * Byte 8
     */
    comm_eth_pc_packet.misc =
        buf[ETHERNET_PC_MISC_OFFSET];

    comm_eth_pc_packet.rx_tick_ms = now_ms;

    comm_eth_last_pc_rx_tick = now_ms;

    /*
     * latest-wins
     */
    comm_eth_pc_pending = true;


    /*
     * PC E-Stop은 sticky 처리.
     *
     * E-Stop 효과 적용은 manager 책임.
     */
    if ((comm_eth_pc_packet.misc &
         ETHERNET_PC_MISC_ESTOP_MASK) != 0U) {

        comm_eth_estop_pending_mask |=
            COMM_ETHERNET_ESTOP_PC;
    }
}


/* =========================================
 * UDP receive callback
 * ========================================= */

static void CommEthernet_UdpRecvCallback(
    void *arg,
    struct udp_pcb *pcb,
    struct pbuf *p,
    const ip_addr_t *addr,
    u16_t port)
{
    uint16_t len;
    uint16_t copied;

    /*
     * 현재 최대 packet은 PC 9 bytes.
     */
    uint8_t buffer[ETHERNET_PC_PACKET_SIZE] = {0};

    (void)arg;
    (void)pcb;
    (void)port;


    if (p == NULL) {
        return;
    }

    len = p->tot_len;


    /*
     * 지원하지 않는 packet length는 즉시 폐기.
     */
    if ((len != ETHERNET_ASMS_PACKET_SIZE) &&
        (len != ETHERNET_PC_PACKET_SIZE)) {

        pbuf_free(p);
        return;
    }


    /*
     * pbuf chain을 contiguous local buffer로 복사.
     */
    copied =
        pbuf_copy_partial(
            p,
            buffer,
            len,
            0
        );

    pbuf_free(p);


    if (copied != len) {
        return;
    }


    /*
     * IPv4 source만 허용.
     */
    if ((addr == NULL) || !IP_IS_V4(addr)) {
        return;
    }


    /* =====================================
     * ASMS
     * ===================================== */

    if (len == ETHERNET_ASMS_PACKET_SIZE) {

        if (!CommEthernet_IsAsmsSender(addr)) {
            return;
        }

        CommEthernet_ParseAsmsPacket(buffer);
        return;
    }


    /* =====================================
     * PC
     * ===================================== */

    if (len == ETHERNET_PC_PACKET_SIZE) {

        if (!CommEthernet_IsPcSender(addr)) {
            return;
        }

        CommEthernet_ParsePcPacket(buffer);
        return;
    }
}


/* =========================================
 * Public API
 * ========================================= */

void CommEthernet_Init(void)
{
    /*
     * 재초기화 방어
     */
    if (comm_eth_udp_pcb != NULL) {
        udp_remove(comm_eth_udp_pcb);
        comm_eth_udp_pcb = NULL;
    }


    comm_eth_initialized = false;

    comm_eth_asms_pending = false;
    comm_eth_pc_pending = false;

    comm_eth_estop_pending_mask =
        COMM_ETHERNET_ESTOP_NONE;

    comm_eth_last_asms_rx_tick = 0U;
    comm_eth_last_pc_rx_tick = 0U;


    memset(
        &comm_eth_asms_packet,
        0,
        sizeof(comm_eth_asms_packet)
    );

    memset(
        &comm_eth_pc_packet,
        0,
        sizeof(comm_eth_pc_packet)
    );


    /*
     * UDP PCB 생성
     */
    comm_eth_udp_pcb = udp_new();

    if (comm_eth_udp_pcb == NULL) {
        return;
    }


    /*
     * UDP :5000 bind
     */
    if (udp_bind(
            comm_eth_udp_pcb,
            IP_ADDR_ANY,
            ETHERNET_UDP_PORT) != ERR_OK) {

        udp_remove(comm_eth_udp_pcb);
        comm_eth_udp_pcb = NULL;

        return;
    }


    /*
     * RX callback 등록
     */
    udp_recv(
        comm_eth_udp_pcb,
        CommEthernet_UdpRecvCallback,
        NULL
    );


    comm_eth_initialized = true;
}


bool CommEthernet_IsInitialized(void)
{
    return comm_eth_initialized;
}


bool CommEthernet_ConsumeAsmsPacket(
    CommEthernet_AsmsPacket_t *packet)
{
    if (packet == NULL) {
        return false;
    }

    if (!comm_eth_asms_pending) {
        return false;
    }

    *packet = comm_eth_asms_packet;

    comm_eth_asms_pending = false;

    return true;
}


bool CommEthernet_ConsumePcPacket(
    CommEthernet_PcPacket_t *packet)
{
    if (packet == NULL) {
        return false;
    }

    if (!comm_eth_pc_pending) {
        return false;
    }

    *packet = comm_eth_pc_packet;

    comm_eth_pc_pending = false;

    return true;
}


uint8_t CommEthernet_ConsumeEstopMask(void)
{
    uint8_t mask;

    mask = comm_eth_estop_pending_mask;

    comm_eth_estop_pending_mask =
        COMM_ETHERNET_ESTOP_NONE;

    return mask;
}


uint32_t CommEthernet_GetLastAsmsRxTick(void)
{
    return comm_eth_last_asms_rx_tick;
}


uint32_t CommEthernet_GetLastPcRxTick(void)
{
    return comm_eth_last_pc_rx_tick;
}

/* =========================================
 * Steering Status TX
 * ========================================= */

bool CommEthernet_SendSteerStatus(
    const CommEthernet_SteerStatus_t *status)
{
    ip_addr_t dest_addr;
    struct pbuf *p;
    err_t err;

    uint8_t tx_data[ETHERNET_STATUS_PACKET_SIZE] = {0};


    if (status == NULL) {
        return false;
    }

    if (!comm_eth_initialized ||
        comm_eth_udp_pcb == NULL) {
        return false;
    }


    /* Network interface / link 확인 */
    if (netif_default == NULL ||
        !netif_is_up(netif_default) ||
        !netif_is_link_up(netif_default)) {
        return false;
    }


    /* 목적지 IP 변환 */
    if (!ipaddr_aton(
            ETHERNET_STATUS_DEST_IP,
            &dest_addr)) {
        return false;
    }


    /* Byte 0~1: actual steering raw */
    CommEthernet_WriteInt16LE(
        &tx_data[ETHERNET_STATUS_ACTUAL_OFFSET],
        status->actual_raw
    );

    /* Byte 2~5: measurement timestamp */
    CommEthernet_WriteUint32LE(
        &tx_data[ETHERNET_STATUS_TIMESTAMP_OFFSET],
        status->sample_tick_ms
    );


    /* 송신용 pbuf 할당 */
    p = pbuf_alloc(
        PBUF_TRANSPORT,
        sizeof(tx_data),
        PBUF_RAM
    );

    if (p == NULL) {
        return false;
    }


    err = pbuf_take(
        p,
        tx_data,
        sizeof(tx_data)
    );

    if (err == ERR_OK) {
        err = udp_sendto(
            comm_eth_udp_pcb,
            p,
            &dest_addr,
            ETHERNET_STATUS_DEST_PORT
        );
    }


    pbuf_free(p);

    return (err == ERR_OK);
}