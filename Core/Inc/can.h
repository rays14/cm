#ifndef _CAN_H_
#define _CAN_H_ 1

#include <stdint.h>
#include <stdio.h>
#include <stm32l4xx_hal.h>
#include "stm32l4xx_hal_can.h"

typedef struct can_t {
    CAN_HandleTypeDef  *hcan1;
    CAN_TxHeaderTypeDef txHeader;
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t             txData[8];
    uint8_t             rxData[8];
    uint32_t            txMailbox;
} can_t;

extern uint32_t myMsgId;

void canInit(can_t *this, CAN_HandleTypeDef *hcan1);
void canTx(can_t *this, uint32_t msgId, uint8_t *data, int len);
void canRx(can_t *this, uint32_t *msgId, uint8_t *data, int len);

#endif // _CAN_H_

