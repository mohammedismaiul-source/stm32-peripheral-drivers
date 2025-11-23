/*
 *
 *      Author: moham
 */
#ifndef STM32F446RE_I2C_DRIVER_H_
#define STM32F446RE_I2C_DRIVER_H_

#include "STM32F446RE.h"

/*
 * Basic configuration structure for I2C peripheral
 */
typedef struct
{
    uint32_t I2C_ClockSpeed;      /* SCL speed in Hz (100k, 200k, 400k)       */
    uint8_t  I2C_OwnAddress;      /* Own device address (7-bit)               */
    uint8_t  I2C_AckEnable;       /* Enable/disable ACK                       */
    uint8_t  I2C_FastModeDuty;    /* Duty cycle in fast mode                  */

} I2C_ConfigBasic_t;

/*
 * Handle structure for I2C peripheral
 */
typedef struct
{
    I2C_RegDef_t      *pI2Cx;       /* I2C peripheral base address              */
    I2C_ConfigBasic_t  I2C_Config;  /* User configuration                       */

    uint8_t  *pTxBuffer;           /* Pointer to Tx buffer                     */
    uint8_t  *pRxBuffer;           /* Pointer to Rx buffer                     */
    uint32_t  TxLen;               /* Tx length                                */
    uint32_t  RxLen;               /* Rx length                                */

    uint8_t   TxRxState;           /* Communication state                      */
    uint8_t   DevAddr;             /* Slave / device address                   */
    uint32_t  RxSize;              /* Reception size                           */
    uint8_t   RepeatedStart;       /* Repeated start flag                      */

} I2C_HandleBasic_t;


/*
 * I2C application states
 */
#define I2C_STATE_READY          0
#define I2C_STATE_BUSY_RX        1
#define I2C_STATE_BUSY_TX        2

/*
 * @I2C_ClockSpeed
 */
#define I2C_SPEED_STANDARD       100000U   /* 100 kHz  */
#define I2C_SPEED_FAST_200K      200000U   /* 200 kHz  */
#define I2C_SPEED_FAST_400K      400000U   /* 400 kHz  */

/*
 * @I2C_AckEnable
 */
#define I2C_ACK_ON               1
#define I2C_ACK_OFF              0

/*
 * @I2C_FastModeDuty
 */
#define I2C_DUTY_2               0
#define I2C_DUTY_16_9            1

/*
 * I2C status flag definitions
 */
#define I2C_STATUS_TXE           (1U << I2C_SR1_TXE)
#define I2C_STATUS_RXNE          (1U << I2C_SR1_RXNE)
#define I2C_STATUS_SB            (1U << I2C_SR1_SB)
#define I2C_STATUS_OVR           (1U << I2C_SR1_OVR)
#define I2C_STATUS_AF            (1U << I2C_SR1_AF)
#define I2C_STATUS_ARLO          (1U << I2C_SR1_ARLO)
#define I2C_STATUS_BERR          (1U << I2C_SR1_BERR)
#define I2C_STATUS_STOPF         (1U << I2C_SR1_STOPF)
#define I2C_STATUS_ADD10         (1U << I2C_SR1_ADD10)
#define I2C_STATUS_BTF           (1U << I2C_SR1_BTF)
#define I2C_STATUS_ADDR          (1U << I2C_SR1_ADDR)
#define I2C_STATUS_TIMEOUT       (1U << I2C_SR1_TIMEOUT)

#define I2C_REPEAT_START_DISABLE RESET
#define I2C_REPEAT_START_ENABLE  SET


/*
 * I2C application event codes
 */
#define I2C_EVENT_TX_COMPLETE        0
#define I2C_EVENT_RX_COMPLETE        1
#define I2C_EVENT_STOP_DETECTED      2
#define I2C_EVENT_ERROR_BERR         3
#define I2C_EVENT_ERROR_ARLO         4
#define I2C_EVENT_ERROR_AF           5
#define I2C_EVENT_ERROR_OVR          6
#define I2C_EVENT_ERROR_TIMEOUT      7
#define I2C_EVENT_DATA_REQUEST       8   /* Data request from master */
#define I2C_EVENT_DATA_RECEIVE       9   /* Data received from master */

/*
 * Peripheral Clock setup
 */
void I2C_ClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/*
 * Init and De-init
 */
void I2C_InitPeripheral(I2C_HandleBasic_t *pI2CHandle);
void I2C_Reset(I2C_RegDef_t *pI2Cx);


/*
 * Data Send and Receive (blocking)
 */
void I2C_MasterSendBlocking(I2C_HandleBasic_t *pI2CHandle,
                            uint8_t *pTxbuffer,
                            uint32_t Len,
                            uint8_t SlaveAddr,
                            uint8_t RepeatedStart);

void I2C_MasterReceiveBlocking(I2C_HandleBasic_t *pI2CHandle,
                               uint8_t *pRxBuffer,
                               uint8_t Len,
                               uint8_t SlaveAddr,
                               uint8_t RepeatedStart);

/*
 * Data Send and Receive (interrupt mode)
 */
uint8_t I2C_MasterSendIT(I2C_HandleBasic_t *pI2CHandle,
                         uint8_t *pTxbuffer,
                         uint32_t Len,
                         uint8_t SlaveAddr,
                         uint8_t RepeatedStart);

uint8_t I2C_MasterReceiveIT(I2C_HandleBasic_t *pI2CHandle,
                            uint8_t *pRxBuffer,
                            uint8_t Len,
                            uint8_t SlaveAddr,
                            uint8_t RepeatedStart);

/*
 * Close Tx/Rx in interrupt mode
 */
void I2C_CloseRx(I2C_HandleBasic_t *pI2CHandle);
void I2C_CloseTx(I2C_HandleBasic_t *pI2CHandle);

/*
 * Simple slave APIs
 */
void    I2C_SlaveWrite(I2C_RegDef_t *pI2C, uint8_t data);
uint8_t I2C_SlaveRead (I2C_RegDef_t *pI2C);

/*
 * IRQ configuration and ISR handling
 */
void I2C_ConfigIRQ(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_SetIRQPriority(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_HandleEventIRQ(I2C_HandleBasic_t *pI2CHandle);
void I2C_HandleErrorIRQ(I2C_HandleBasic_t *pI2CHandle);

/*
 * Other peripheral control APIs
 */
void    I2C_EnablePeripheral(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
uint8_t I2C_ReadFlag(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void    I2C_ConfigAck(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
void    I2C_GenerateStop(I2C_RegDef_t *pI2Cx);

void    I2C_SlaveConfigCallbacks(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/*
 * Application callback (to be implemented in user code)
 */
void I2C_ApplicationCallback(I2C_HandleBasic_t *pI2CHandle, uint8_t AppEvent);

#endif /* STM32F446RE_I2C_DRIVER_H_ */
