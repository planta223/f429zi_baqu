/*
 * comm_can.c
 *
 *  Created on: Aug 27, 2026
 *      Author: kyubeom
 */

#include "comm_can.h"

#include "main.h"
#include "can.h"
#include "config.h"

#include <string.h>


/* =========================================
 * Private variables
 * ========================================= */

static bool comm_can_initialized = false;


/*
 * Steering Request latest-wins mailbox.
 */
static volatile CommCan_SteerRequest_t comm_can_request;

static volatile bool comm_can_request_pending = false;


/*
 * ESTOP은 일반 request mailbox와 분리한다.
 *
 * 정상 request가 뒤이어 와도 manager가 consume하기 전에는
 * ESTOP event가 사라지지 않는다.
 */
static volatile bool comm_can_estop_pending = false;


/*
 * Transport-level timestamp.
 */
static volatile uint32_t comm_can_last_request_rx_tick = 0U;


/* =========================================
 * Little-endian utilities
 * ========================================= */

static uint16_t CommCan_ReadUint16LE(
    const uint8_t *buf)
{
    return
        ((uint16_t)buf[0]) |
        ((uint16_t)buf[1] << 8);
}


static int16_t CommCan_ReadInt16LE(
    const uint8_t *buf)
{
    return (int16_t)CommCan_ReadUint16LE(buf);
}


static void CommCan_WriteUint16LE(
    uint8_t *buf,
    uint16_t value)
{
    buf[0] = (uint8_t)(value & 0xFFU);
    buf[1] = (uint8_t)((value >> 8) & 0xFFU);
}


static void CommCan_WriteInt16LE(
    uint8_t *buf,
    int16_t value)
{
    CommCan_WriteUint16LE(
        buf,
        (uint16_t)value
    );
}


/* =========================================
 * RX parser
 * ========================================= */

static void CommCan_ParseSteerRequest(
    const uint8_t *data)
{
    uint32_t now_ms;
    int16_t steer_raw;
    uint8_t flags;

    now_ms = HAL_GetTick();

    steer_raw =
        CommCan_ReadInt16LE(
            &data[CAN_REQUEST_STEER_OFFSET]
        );

    flags =
        data[CAN_REQUEST_FLAGS_OFFSET];


    comm_can_last_request_rx_tick = now_ms;


    /*
     * E-Stop frame은 일반 steering mailbox에 넣지 않는다.
     * sticky emergency event로만 저장한다.
     */
    if ((flags & CAN_REQUEST_ESTOP_MASK) != 0U) {

        comm_can_estop_pending = true;

        return;
    }


    /*
     * 일반 steering request
     */
    comm_can_request.steer_raw = steer_raw;
    comm_can_request.flags = flags;
    comm_can_request.rx_tick_ms = now_ms;

    comm_can_request_pending = true;
}


/* =========================================
 * RX frame validation
 * ========================================= */

static bool CommCan_IsValidSteerRequest(
    const CAN_RxHeaderTypeDef *header)
{
    if (header == NULL) {
        return false;
    }


    /*
     * Standard 11-bit CAN ID만 사용.
     */
    if (header->IDE != CAN_ID_STD) {
        return false;
    }


    /*
     * Remote frame은 허용하지 않는다.
     */
    if (header->RTR != CAN_RTR_DATA) {
        return false;
    }


    /*
     * Steering Request ID
     */
    if (header->StdId != CAN_ID_STEER_REQUEST) {
        return false;
    }


    /*
     * DLC 검사
     */
    if (header->DLC != CAN_STEER_REQUEST_DLC) {
        return false;
    }


    return true;
}


/* =========================================
 * CAN filter
 * ========================================= */

static bool CommCan_ConfigFilter(void)
{
    CAN_FilterTypeDef filter = {0};


    /*
     * CAN1 Filter Bank 0 사용.
     *
     * 32-bit mask mode.
     *
     * Standard CAN ID는 bxCAN filter register에서
     * bit 5부터 배치된다.
     */
    filter.FilterBank = 0U;

    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;


    /*
     * ID = 0x100
     */
    filter.FilterIdHigh =
        (uint16_t)(CAN_ID_STEER_REQUEST << 5);

    filter.FilterIdLow = 0U;


    /*
     * 11-bit Standard ID 전체 비교.
     */
    filter.FilterMaskIdHigh =
        (uint16_t)(0x7FFU << 5);

    filter.FilterMaskIdLow = 0U;


    /*
     * 수신 FIFO0 사용.
     */
    filter.FilterFIFOAssignment =
        CAN_RX_FIFO0;


    filter.FilterActivation =
        ENABLE;


    /*
     * CAN1/CAN2 filter bank 분리 기준.
     *
     * F429 bxCAN에서는 CAN1과 CAN2가 filter bank를 공유한다.
     * CAN1만 사용할 경우 일반적으로 14로 두어도 문제없다.
     */
    filter.SlaveStartFilterBank = 14U;


    if (HAL_CAN_ConfigFilter(
            &COMM_CAN_HANDLE,
            &filter) != HAL_OK) {

        return false;
    }


    return true;
}


/* =========================================
 * Public initialization
 * ========================================= */

void CommCan_Init(void)
{
    comm_can_initialized = false;

    comm_can_request_pending = false;
    comm_can_estop_pending = false;

    comm_can_last_request_rx_tick = 0U;


    /*
     * ISR/main 공유 mailbox 초기화
     */
    comm_can_request.steer_raw = 0;
    comm_can_request.flags = 0U;
    comm_can_request.rx_tick_ms = 0U;


    /*
     * 0x100 Steering Request filter 설정.
     */
    if (!CommCan_ConfigFilter()) {
        return;
    }


    /*
     * CAN peripheral 시작.
     */
    if (HAL_CAN_Start(
            &COMM_CAN_HANDLE) != HAL_OK) {

        return;
    }


    /*
     * FIFO0 message pending interrupt 활성화.
     */
    if (HAL_CAN_ActivateNotification(
            &COMM_CAN_HANDLE,
            CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {

        HAL_CAN_Stop(&COMM_CAN_HANDLE);
        return;
    }


    comm_can_initialized = true;
}


bool CommCan_IsInitialized(void)
{
    return comm_can_initialized;
}


/* =========================================
 * RX interrupt callback
 * ========================================= */

void HAL_CAN_RxFifo0MsgPendingCallback(
    CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];


    /*
     * 다른 CAN peripheral callback이면 무시.
     */
    if (hcan != &COMM_CAN_HANDLE) {
        return;
    }


    /*
     * FIFO에 여러 frame이 쌓여 있을 수 있으므로
     * 현재 존재하는 frame을 모두 처리한다.
     *
     * Steering command는 latest-wins이므로
     * 최종적으로 가장 마지막 frame이 mailbox에 남는다.
     */
    while (HAL_CAN_GetRxFifoFillLevel(
               hcan,
               CAN_RX_FIFO0) > 0U) {

        memset(&rx_header, 0, sizeof(rx_header));
        memset(rx_data, 0, sizeof(rx_data));


        if (HAL_CAN_GetRxMessage(
                hcan,
                CAN_RX_FIFO0,
                &rx_header,
                rx_data) != HAL_OK) {

            return;
        }


        /*
         * ID / IDE / RTR / DLC validation.
         */
        if (!CommCan_IsValidSteerRequest(
                &rx_header)) {

            continue;
        }


        CommCan_ParseSteerRequest(rx_data);
    }
}


/* =========================================
 * Request consume
 * ========================================= */

bool CommCan_ConsumeSteerRequest(
    CommCan_SteerRequest_t *request)
{
    bool has_request = false;

    if (request == NULL) {
        return false;
    }

    /*
     * CAN RX ISR이 mailbox를 수정하지 못하도록
     * 아주 짧은 구간 동안 RX0 interrupt를 막는다.
     */
    HAL_NVIC_DisableIRQ(COMM_CAN_RX_IRQn);

    if (comm_can_request_pending) {

        request->steer_raw =
            comm_can_request.steer_raw;

        request->flags =
            comm_can_request.flags;

        request->rx_tick_ms =
            comm_can_request.rx_tick_ms;

        comm_can_request_pending = false;

        has_request = true;
    }

    HAL_NVIC_EnableIRQ(COMM_CAN_RX_IRQn);

    return has_request;
}


/* =========================================
 * ESTOP consume
 * ========================================= */

bool CommCan_ConsumeEstopRequest(void)
{
    bool pending;

    HAL_NVIC_DisableIRQ(COMM_CAN_RX_IRQn);

    pending = comm_can_estop_pending;

    comm_can_estop_pending = false;

    HAL_NVIC_EnableIRQ(COMM_CAN_RX_IRQn);

    return pending;
}


/* =========================================
 * RX timestamp
 * ========================================= */

uint32_t CommCan_GetLastRequestRxTick(void)
{
    return comm_can_last_request_rx_tick;
}


/* =========================================
 * Steering Status TX
 * ========================================= */

bool CommCan_SendSteerStatus(
    const CommCan_SteerStatus_t *status)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[CAN_STEER_STATUS_DLC];
    uint32_t tx_mailbox;


    if (status == NULL) {
        return false;
    }


    if (!comm_can_initialized) {
        return false;
    }


    /*
     * CAN TX mailbox가 모두 사용 중이면
     * 이번 Status transmission은 skip.
     *
     * Status는 주기적으로 다시 보내므로
     * blocking하지 않는 것이 낫다.
     */
    if (HAL_CAN_GetTxMailboxesFreeLevel(
            &COMM_CAN_HANDLE) == 0U) {

        return false;
    }


    memset(
        &tx_header,
        0,
        sizeof(tx_header)
    );

    memset(
        tx_data,
        0,
        sizeof(tx_data)
    );


    /* =====================================
     * TX Header
     * ===================================== */

    tx_header.StdId =
        CAN_ID_STEER_STATUS;

    tx_header.ExtId = 0U;

    tx_header.IDE =
        CAN_ID_STD;

    tx_header.RTR =
        CAN_RTR_DATA;

    tx_header.DLC =
        CAN_STEER_STATUS_DLC;

    tx_header.TransmitGlobalTime =
        DISABLE;


    /* =====================================
     * Byte 0~1: actual steering raw
     * ===================================== */

    CommCan_WriteInt16LE(
        &tx_data[CAN_STATUS_ACTUAL_OFFSET],
        status->actual_raw
    );


    /* =====================================
     * Byte 2~3: target steering raw
     * ===================================== */

    CommCan_WriteInt16LE(
        &tx_data[CAN_STATUS_TARGET_OFFSET],
        status->target_raw
    );


    /* =====================================
     * Byte 4: flags
     * ===================================== */

    tx_data[CAN_STATUS_FLAGS_OFFSET] =
        status->flags;


    /* =====================================
     * Send
     * ===================================== */

    if (HAL_CAN_AddTxMessage(
            &COMM_CAN_HANDLE,
            &tx_header,
            tx_data,
            &tx_mailbox) != HAL_OK) {

        return false;
    }


    return true;
}
