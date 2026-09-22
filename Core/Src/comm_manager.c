/*
 * comm_manager.c
 *
 *  Created on: Aug 27, 2026
 *      Author: kyubeom
 */

#include "comm_manager.h"

#include "comm_ethernet.h"
#include "comm_can.h"

#include "control.h"
#include "encoder.h"
#include "motor.h"
#include "svon.h"
#include "config.h"
#include "led.h"

#include "main.h"


/* =========================================
 * Compile-time validation
 * ========================================= */

#if ((COMM_MODE != COMM_MODE_ETHERNET_ONLY) && \
     (COMM_MODE != COMM_MODE_CAN_ONLY) && \
     (COMM_MODE != COMM_MODE_BOTH))
#error "Invalid COMM_MODE"
#endif


/* =========================================
 * Private variables
 * ========================================= */

static CommManager_State_t comm_manager_state;

static uint32_t comm_manager_last_can_status_tx_tick = 0U;


/* =========================================
 * Utility
 * ========================================= */

static float CommManager_ClampFloat(
    float value,
    float min_value,
    float max_value)
{
    if (value > max_value) {
        return max_value;
    }

    if (value < min_value) {
        return min_value;
    }

    return value;
}


/* =========================================
 * ASMS conversion
 * ========================================= */

static float CommManager_AsmsRawToSteeringDeg(
    int16_t raw)
{
    int32_t value;
    int32_t offset;

    int32_t positive_span;
    int32_t negative_span;

    float normalized;
    float steering_deg;


    /*
     * 우선 int32로 확장해서 계산한다.
     */
    value = (int32_t)raw;


    /*
     * ADC 허용범위 clamp
     */
    if (value > ASMS_ADC_MAX_RAW) {
        value = ASMS_ADC_MAX_RAW;
    }

    if (value < ASMS_ADC_MIN_RAW) {
        value = ASMS_ADC_MIN_RAW;
    }


    /*
     * Center 기준 offset
     */
    offset =
        value -
        ASMS_ADC_CENTER_RAW;


    /*
     * Deadband
     */
    if ((offset <= ASMS_ADC_DEADBAND_RAW) &&
        (offset >= -ASMS_ADC_DEADBAND_RAW)) {

        offset = 0;
    }


    positive_span =
        ASMS_ADC_MAX_RAW -
        ASMS_ADC_CENTER_RAW;

    negative_span =
        ASMS_ADC_CENTER_RAW -
        ASMS_ADC_MIN_RAW;


    /*
     * config 오류 방어
     */
    if ((positive_span <= 0) ||
        (negative_span <= 0)) {

        return 0.0f;
    }


    /*
     * -1 ~ +1 normalization
     */
    if (offset >= 0) {

        normalized =
            (float)offset /
            (float)positive_span;

    } else {

        normalized =
            (float)offset /
            (float)negative_span;
    }


    /*
     * scale + polarity
     */
    steering_deg =
        (float)ASMS_STEER_POLARITY *
        ASMS_STEER_SCALE *
        normalized;


    /*
     * 수치 오차 방어
     */
    steering_deg =
        CommManager_ClampFloat(
            steering_deg,
            -ASMS_STEER_SCALE,
            ASMS_STEER_SCALE
        );


    return steering_deg;
}


/* =========================================
 * PC conversion
 * ========================================= */

static float CommManager_PcRawToSteeringDeg(
    int32_t raw)
{
    return
        (float)raw *
        PC_STEER_SCALE *
        (float)PC_STEER_POLARITY;
}


/* =========================================
 * CAN conversion
 * ========================================= */

static float CommManager_CanRawToSteeringDeg(
    int16_t raw)
{
    if (CAN_REQUEST_STEER_SCALE <= 0.0f) {
        return 0.0f;
    }

    /*
     * raw = deg × scale
     *
     * 따라서
     *
     * deg = raw / scale
     */
    return
        (float)raw /
        CAN_REQUEST_STEER_SCALE;
}


/* =========================================
 * CAN status conversion
 * ========================================= */

static int16_t CommManager_SteeringDegToCanRaw(
    float steering_deg)
{
    float raw;


    raw =
        steering_deg *
        CAN_STATUS_STEER_SCALE;


    /*
     * int16 범위 보호
     */
    if (raw > 32767.0f) {
        raw = 32767.0f;
    }

    if (raw < -32768.0f) {
        raw = -32768.0f;
    }


    /*
     * 간단한 rounding
     */
    if (raw >= 0.0f) {
        raw += 0.5f;
    } else {
        raw -= 0.5f;
    }


    return (int16_t)raw;
}


/* =========================================
 * Control application
 * ========================================= */

static void CommManager_ApplyTarget(
    float steering_deg,
    CommManager_Source_t source)
{
    /*
     * Encoder feedback이 없으면 폐루프 조향 금지.
     * Fail-closed.
     */
    if (Encoder_IsInitialized() == 0U) {

        Control_Disable();
        SVON_Disable();

        comm_manager_state.active_source =
            COMM_SOURCE_NONE;

        return;
    }

    /*
     * 실제 최종 steering limit은
     * Control_SetTargetSteeringDeg()에서 한 번 더 clamp된다.
     */
    Control_SetTargetSteeringDeg(steering_deg);


    /*
     * 정상적인 유효 조향명령이 들어오면 Servo ON.
     */
    if (SVON_IsEnabled() == 0U) {
        SVON_Enable();
    }


    /*
     * Control enable
     */
    if (Control_IsEnabled() == 0U) {
        Control_Enable();
    }


    comm_manager_state.active_source = source;
}


/* =========================================
 * Emergency handling
 * ========================================= */

static void CommManager_ApplyEmergency(
    uint8_t source_mask)
{
    /*
     * Control_Disable() 내부에서 Motor_Stop()까지 수행한다.
     */
    Control_Disable();

    SVON_Disable();


    comm_manager_state.mode =
        COMM_STEER_MODE_ESTOP;

    comm_manager_state.active_source =
        COMM_SOURCE_NONE;

    comm_manager_state.last_estop_source_mask =
        source_mask;

    LED_RED_ON();
}


/* =========================================
 * Valid command timestamp
 * ========================================= */

static void CommManager_MarkEthernetValid(
    uint32_t tick)
{
    comm_manager_state.last_valid_ethernet_tick =
        tick;

    comm_manager_state.ethernet_timeout = 0U;

    LED_GREEN_ON();
}


static void CommManager_MarkCanValid(
    uint32_t tick)
{
    comm_manager_state.last_valid_can_tick =
        tick;

    comm_manager_state.can_timeout = 0U;

    #if COMM_MODE == COMM_MODE_CAN_ONLY
        LED_GREEN_ON();
    #endif
}


/* =========================================
 * E-Stop input processing
 * ========================================= */

static uint8_t CommManager_ProcessEmergencyInputs(void)
{
    uint8_t source_mask = 0U;


#if ((COMM_MODE == COMM_MODE_ETHERNET_ONLY) || \
     (COMM_MODE == COMM_MODE_BOTH))

    {
        uint8_t ethernet_estop_mask;

        ethernet_estop_mask =
            CommEthernet_ConsumeEstopMask();


        if ((ethernet_estop_mask &
             COMM_ETHERNET_ESTOP_ASMS) != 0U) {

            source_mask |=
                COMM_ESTOP_SOURCE_ETHERNET_ASMS;
        }


        if ((ethernet_estop_mask &
             COMM_ETHERNET_ESTOP_PC) != 0U) {

            source_mask |=
                COMM_ESTOP_SOURCE_ETHERNET_PC;
        }
    }

#endif


#if ((COMM_MODE == COMM_MODE_CAN_ONLY) || \
     (COMM_MODE == COMM_MODE_BOTH))

    if (CommCan_ConsumeEstopRequest()) {

        source_mask |=
            COMM_ESTOP_SOURCE_CAN;
    }

#endif


    if (source_mask != 0U) {

        CommManager_ApplyEmergency(
            source_mask
        );

        return 1U;
    }


    return 0U;
}


/* =========================================
 * ASMS processing
 * ========================================= */

static void CommManager_ProcessAsms(void)
{
    CommEthernet_AsmsPacket_t packet;
    float steering_deg;


    if (!CommEthernet_ConsumeAsmsPacket(&packet)) {
        return;
    }


    switch (packet.mode_raw) {

    /* =====================================
     * AUTO
     * ===================================== */

    case COMM_STEER_MODE_AUTO:

        /*
         * ASMS AUTO packet은 mode만 변경한다.
         *
         * 실제 steering target은
         * PC packet이 들어와야 갱신된다.
         *
         * 따라서 여기서는
         * last_valid_ethernet_tick을 갱신하지 않는다.
         */
        comm_manager_state.mode =
            COMM_STEER_MODE_AUTO;

        LED_RED_OFF();

        break;


    /* =====================================
     * MANUAL
     * ===================================== */

    case COMM_STEER_MODE_MANUAL:

        comm_manager_state.mode =
            COMM_STEER_MODE_MANUAL;

        LED_RED_OFF();
        
        steering_deg =
            CommManager_AsmsRawToSteeringDeg(
                packet.steer_raw
            );


        CommManager_ApplyTarget(
            steering_deg,
            COMM_SOURCE_ETHERNET_ASMS
        );


        CommManager_MarkEthernetValid(
            packet.rx_tick_ms
        );

        break;


    /* =====================================
     * ESTOP
     * ===================================== */

    case COMM_STEER_MODE_ESTOP:

        CommManager_ApplyEmergency(
            COMM_ESTOP_SOURCE_ETHERNET_ASMS
        );

        break;


    /* =====================================
     * Invalid mode
     * ===================================== */

    default:

        /*
         * 알 수 없는 ASMS mode는 무시한다.
         *
         * timeout timestamp도 갱신하지 않는다.
         */
        break;
    }
}


/* =========================================
 * PC processing
 * ========================================= */

static void CommManager_ProcessPc(void)
{
    CommEthernet_PcPacket_t packet;
    float steering_deg;


    if (!CommEthernet_ConsumePcPacket(&packet)) {
        return;
    }


    /*
     * E-Stop packet 자체가 mailbox에 남아 있을 수도 있으므로
     * 여기서도 한 번 더 방어한다.
     */
    if ((packet.misc &
         ETHERNET_PC_MISC_ESTOP_MASK) != 0U) {

        CommManager_ApplyEmergency(
            COMM_ESTOP_SOURCE_ETHERNET_PC
        );

        return;
    }


#if PC_ALLOW_AUTO_ENTRY

    /*
     * ASMS 없이 PC 단독 테스트.
     *
     * NONE 상태에서만 AUTO 진입 가능.
     *
     * MANUAL / ESTOP은 PC가 덮어쓰지 않는다.
     */
    if (comm_manager_state.mode ==
        COMM_STEER_MODE_NONE) {

        comm_manager_state.mode =
            COMM_STEER_MODE_AUTO;
    }

#endif


    /*
     * AUTO 상태에서만 PC steering command 허용.
     */
    if (comm_manager_state.mode !=
        COMM_STEER_MODE_AUTO) {

        return;
    }


    steering_deg =
        CommManager_PcRawToSteeringDeg(
            packet.steer_raw
        );


    CommManager_ApplyTarget(
        steering_deg,
        COMM_SOURCE_ETHERNET_PC
    );


    CommManager_MarkEthernetValid(
        packet.rx_tick_ms
    );
}


/* =========================================
 * CAN Request processing
 * ========================================= */

static void CommManager_ProcessCan(void)
{
    CommCan_SteerRequest_t request;
    float steering_deg;


    if (!CommCan_ConsumeSteerRequest(
            &request)) {

        return;
    }


    /*
     * CAN ESTOP packet 자체가 mailbox에 남아 있는 경우
     * 재확인.
     */
    if ((request.flags &
         CAN_REQUEST_ESTOP_MASK) != 0U) {

        CommManager_ApplyEmergency(
            COMM_ESTOP_SOURCE_CAN
        );

        return;
    }


    /*
     * 정상 CAN packet이 들어왔다는 사실 자체는
     * BOTH mode에서도 기록한다.
     */
    CommManager_MarkCanValid(
        request.rx_tick_ms
    );


#if COMM_MODE == COMM_MODE_CAN_ONLY

    /*
     * CAN에는 별도 AUTO/MANUAL mode가 없으므로
     * 정상 Steering Request 자체를 AUTO command로 본다.
     *
     * CAN ESTOP 이후에도 정상 request가 들어오면
     * AUTO로 복귀한다.
     */
    comm_manager_state.mode =
        COMM_STEER_MODE_AUTO;

    LED_RED_OFF();

    steering_deg =
        CommManager_CanRawToSteeringDeg(
            request.steer_raw
        );


    CommManager_ApplyTarget(
        steering_deg,
        COMM_SOURCE_CAN
    );


#elif COMM_MODE == COMM_MODE_BOTH

    /*
     * BOTH에서는 CAN steering request를 수신만 한다.
     *
     * 실제 조향 제어권은 Ethernet에 있다.
     */
    (void)steering_deg;

#endif
}


/* =========================================
 * Ethernet timeout
 * ========================================= */

static void CommManager_CheckEthernetTimeout(
    uint32_t now_ms)
{
    uint32_t last_tick;


    last_tick =
        comm_manager_state.last_valid_ethernet_tick;


    /*
     * 아직 유효 명령을 한 번도 받은 적 없음.
     */
    if (last_tick == 0U) {
        return;
    }


    /*
     * 이미 timeout 처리됨.
     */
    if (comm_manager_state.ethernet_timeout != 0U) {
        return;
    }


    if ((uint32_t)(now_ms - last_tick) <=
        ETHERNET_TIMEOUT_MS) {

        return;
    }


    comm_manager_state.ethernet_timeout = 1U;

    LED_GREEN_OFF();


#if ETHERNET_TIMEOUT_POLICY == COMM_TIMEOUT_POLICY_HOLD

    /*
     * 마지막 target / SVON / Control 상태 유지.
     */


#elif ETHERNET_TIMEOUT_POLICY == COMM_TIMEOUT_POLICY_RELEASE

    Control_Disable();
    SVON_Disable();

    comm_manager_state.active_source =
        COMM_SOURCE_NONE;


#else

#error "Invalid ETHERNET_TIMEOUT_POLICY"

#endif
}


/* =========================================
 * CAN timeout
 * ========================================= */
static void CommManager_CheckCanTimeout(
    uint32_t now_ms)
{
    uint32_t last_tick;

    last_tick =
        comm_manager_state.last_valid_can_tick;


    /*
     * 아직 유효 CAN Request를 한 번도 받지 않음.
     */
    if (last_tick == 0U) {
        return;
    }


    /*
     * 아직 timeout 아님.
     */
    if ((uint32_t)(now_ms - last_tick)
        <= CAN_TIMEOUT_MS) {

        comm_manager_state.can_timeout = 0U;
        return;
    }


    /*
     * CAN stale 상태 기록.
     *
     * CAN_ONLY / BOTH 모두 여기까지 수행한다.
     */
    comm_manager_state.can_timeout = 1U;
    
    #if COMM_MODE == COMM_MODE_CAN_ONLY
        LED_GREEN_OFF();
    #endif

#if COMM_MODE == COMM_MODE_CAN_ONLY

    /*
     * CAN이 실제 조향 제어권을 가지는 경우에만
     * timeout policy를 제어기에 적용한다.
     */

#if CAN_TIMEOUT_POLICY == COMM_TIMEOUT_POLICY_HOLD

    /*
     * 마지막 target / Control / SVON 상태 유지
     */


#elif CAN_TIMEOUT_POLICY == COMM_TIMEOUT_POLICY_RELEASE

    Control_Disable();
    SVON_Disable();

    comm_manager_state.active_source =
        COMM_SOURCE_NONE;


#else
#error "Invalid CAN_TIMEOUT_POLICY"
#endif

#endif /* COMM_MODE == COMM_MODE_CAN_ONLY */
}


/* =========================================
 * CAN Status
 * ========================================= */

static void CommManager_SendCanStatus(void)
{
    CommCan_SteerStatus_t status;

    Control_State_t control_state;

    float actual_deg;
    float target_deg;


    actual_deg =
        Encoder_GetSteeringDeg();

    control_state =
        Control_GetState();

    target_deg =
        control_state.target_steering_deg;


    status.actual_raw =
        CommManager_SteeringDegToCanRaw(
            actual_deg
        );


    status.target_raw =
        CommManager_SteeringDegToCanRaw(
            target_deg
        );


    status.flags = 0U;


    if (Control_IsReached() != 0U) {

        status.flags |=
            CAN_STATUS_FLAG_REACHED;
    }


    if (Control_IsEnabled() != 0U) {

        status.flags |=
            CAN_STATUS_FLAG_CONTROL_ENABLED;
    }


    if (SVON_IsEnabled() != 0U) {

        status.flags |=
            CAN_STATUS_FLAG_SVON_ENABLED;
    }


    if (Motor_IsOutputActive() != 0U) {

        status.flags |=
            CAN_STATUS_FLAG_MOTOR_OUTPUT_ACTIVE;
    }


    if (Encoder_IsInitialized() != 0U) {

        status.flags |=
            CAN_STATUS_FLAG_ENCODER_INITIALIZED;
    }


    /*
     * Status는 best-effort.
     *
     * TX mailbox가 꽉 차면 이번 주기는 skip한다.
     */
    if (CommCan_SendSteerStatus(&status))
        LED_BLUE_ON();
    else
        LED_BLUE_OFF();
}


/* =========================================
 * CAN Status scheduler
 * ========================================= */

static void CommManager_ServiceCanStatus(
    uint32_t now_ms)
{
#if ((COMM_MODE == COMM_MODE_CAN_ONLY) || \
     (COMM_MODE == COMM_MODE_BOTH))

    if ((uint32_t)(
            now_ms -
            comm_manager_last_can_status_tx_tick)
        >= CAN_STATUS_PERIOD_MS) {

        /*
         * 지연 후 여러 frame을 burst 송신하지 않고
         * 현재 시점부터 다시 주기를 잡는다.
         */
        comm_manager_last_can_status_tx_tick =
            now_ms;


        CommManager_SendCanStatus();
    }

#else

    (void)now_ms;

#endif
}


/* =========================================
 * Public
 * ========================================= */

void CommManager_Init(void)
{
    comm_manager_state.mode =
        COMM_STEER_MODE_NONE;

    comm_manager_state.active_source =
        COMM_SOURCE_NONE;

    comm_manager_state.last_valid_ethernet_tick =
        0U;

    comm_manager_state.last_valid_can_tick =
        0U;

    comm_manager_state.ethernet_timeout =
        0U;

    comm_manager_state.can_timeout =
        0U;

    comm_manager_state.last_estop_source_mask =
        0U;


    comm_manager_last_can_status_tx_tick =
        HAL_GetTick();


#if ((COMM_MODE == COMM_MODE_ETHERNET_ONLY) || \
     (COMM_MODE == COMM_MODE_BOTH))

    CommEthernet_Init();

#endif


#if ((COMM_MODE == COMM_MODE_CAN_ONLY) || \
     (COMM_MODE == COMM_MODE_BOTH))

    CommCan_Init();

#endif
}


/* =========================================
 * Main manager update
 * ========================================= */

void CommManager_Update(void)
{
    uint32_t now_ms;


    /*
     * 1. Emergency를 가장 먼저 처리.
     *
     * 같은 loop에서 정상 조향명령과 E-Stop이 동시에 존재하면
     * E-Stop이 우선한다.
     */
    if (CommManager_ProcessEmergencyInputs() == 0U) {


        /* =================================
         * 2. Ethernet inputs
         * ================================= */

#if ((COMM_MODE == COMM_MODE_ETHERNET_ONLY) || \
     (COMM_MODE == COMM_MODE_BOTH))

        /*
         * ASMS를 PC보다 먼저 처리한다.
         *
         * 같은 cycle에서
         *
         * ASMS AUTO → PC command
         *
         * 순으로 들어왔다면
         * PC command가 바로 허용된다.
         */
        CommManager_ProcessAsms();

        CommManager_ProcessPc();

#endif


        /* =================================
         * 3. CAN input
         * ================================= */

#if ((COMM_MODE == COMM_MODE_CAN_ONLY) || \
     (COMM_MODE == COMM_MODE_BOTH))

        CommManager_ProcessCan();

#endif
    }


    /*
     * 중요:
     *
     * packet 처리가 끝난 후 HAL_GetTick()을 snapshot한다.
     *
     * 기존에 발생했던
     *
     * now_ms를 먼저 읽음
     * → MX_LWIP_Process 중 RX callback
     * → last_rx_tick이 now_ms보다 미래
     *
     * 문제가 다시 발생하지 않도록 한다.
     */
    now_ms = HAL_GetTick();


    /* =====================================
     * 4. Timeout
     * ===================================== */

#if ((COMM_MODE == COMM_MODE_ETHERNET_ONLY) || \
     (COMM_MODE == COMM_MODE_BOTH))

    /*
     * BOTH의 제어권은 Ethernet이므로
     * Ethernet timeout이 제어에 영향을 준다.
     */
    CommManager_CheckEthernetTimeout(
        now_ms
    );

#endif


#if ((COMM_MODE == COMM_MODE_CAN_ONLY) || \
     (COMM_MODE == COMM_MODE_BOTH))

    /*
     * CAN timeout 상태는 CAN_ONLY / BOTH 모두 감지한다.
     *
     * 단,
     * - CAN_ONLY : timeout policy(HOLD/RELEASE)를 실제 제어에 적용
     * - BOTH     : can_timeout 상태만 기록하고,
     *              Ethernet 제어에는 영향을 주지 않음
     *
     * 실제 Control/SVON RELEASE 여부는
     * CommManager_CheckCanTimeout() 내부에서 결정한다.
     */
    CommManager_CheckCanTimeout(
        now_ms
    );

#endif


    /* =====================================
     * 5. CAN Status
     * ===================================== */

    CommManager_ServiceCanStatus(
        now_ms
    );
}


CommManager_SteerMode_t
CommManager_GetMode(void)
{
    return comm_manager_state.mode;
}


CommManager_Source_t
CommManager_GetActiveSource(void)
{
    return comm_manager_state.active_source;
}


CommManager_State_t
CommManager_GetState(void)
{
    return comm_manager_state;
}
