#include "can.h"

uint32_t myMsgId = 0x0A1;

void canInit(can_t *this, CAN_HandleTypeDef *hcan1) {
    assert(this);
    assert(hcan1);
    CAN_FilterTypeDef canfilterconfig;

    this->hcan1 = hcan1;
    this->txHeader.IDE = CAN_ID_STD;
    this->txHeader.StdId = 0x000;
    this->txHeader.RTR = CAN_RTR_DATA;
    this->txHeader.DLC = 0;

    // Don't use filters. Figure it out in the software.
    canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig.FilterBank = 12;  // Which filter bank to use from the assigned ones
    canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;

    // Just set the masks. Software will decide on the ID.
    //canfilterconfig.FilterIdHigh = 0xFFF << 5; //0x446 << 5;
    //canfilterconfig.FilterIdLow = 0;
    canfilterconfig.FilterMaskIdHigh = 0xFFF << 5;
    canfilterconfig.FilterMaskIdLow = 0x0000;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilterconfig.SlaveStartFilterBank = 12;  // How many filters to assign to the CAN1 (master can)

    HAL_CAN_ConfigFilter(this->hcan1, &canfilterconfig);
}

void canTx(can_t *this, uint32_t msgId, uint8_t *data, int len) {
    assert(data);
    assert((len > 0) && (len <= 8));
    this->txHeader.IDE = CAN_ID_STD;
    this->txHeader.StdId = msgId & 0x4ff;
    this->txHeader.RTR = CAN_RTR_DATA;
    this->txHeader.DLC = len;
    for (int i = 0; i < len; i++) {
        this->txData[i] = data[i];
    } 
    if (HAL_CAN_AddTxMessage(this->hcan1, 
        &this->txHeader, this->txData, &this->txMailbox) != HAL_OK) {
       //Error_Handler ();
    }
}

void canRx(can_t *this, uint32_t *msgId, uint8_t *data, int len) {
    assert(this);
    assert(data);
    assert((len > 0) && (len <= 0));

    //CAN_RxHeaderTypeDef RxHeader;
    //uint8_t RxData[8];

    if (HAL_CAN_GetRxMessage(this->hcan1, CAN_RX_FIFO0, &this->rxHeader, this->rxData) != HAL_OK) {
        //Error_Handler();
    }
    *msgId = this->rxHeader.StdId;
    for (int i = 0; i < len; i++) {
        data[i] =  this->rxData[i];
    } 

    //if ((RxHeader.StdId == 0x103)) {
	//    datacheck = 1;
    //}
}
