/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"

#define FW_VERSION  0x0201
#define I2C_TX_LENGTH   0x8
#define I2C_RX_LENGTH   0x1

/* Commands */
#define PHYHANDHELD_FW_VER		0x0001
#define PHYHANDHELD_SWRST		0x0002
#define PHYHANDHELD_BUT_BITMAP		0x0003
#define PHYHANDHELD_ADC     		0x0004

/* Button Mapping */
#define PHYHANDHELD_R_BUT			0x4000
#define PHYHANDHELD_R_BUT1			0x2000
#define PHYHANDHELD_J1_SW			0x1000
#define PHYHANDHELD_J2_SW			0x0800
#define PHYHANDHELD_BUT				0x0400
#define PHYHANDHELD_BUT1			0x0200
#define PHYHANDHELD_BUT2			0x0100
#define PHYHANDHELD_BUT3			0x0080
#define PHYHANDHELD_BUT4			0x0040
#define PHYHANDHELD_BUT5			0x0020
#define PHYHANDHELD_BUT6			0x0010
#define PHYHANDHELD_BUT7			0x0008
#define PHYHANDHELD_BUT8			0x0004
#define PHYHANDHELD_BUT9			0x0002
#define PHYHANDHELD_BUT10			0x0001

/* Data received from Controller during a Write transfer */
uint8_t gRxPacket;

/* Data sent to Controller during a Read transfer */
uint8_t gTxPacket[I2C_TX_LENGTH];

/* Boolean to know when a stop command was issued */
bool gStopReceived = false;

/* ADC variables:
 * gADCResult is the adc result taken from the Memory Register
 */

volatile uint16_t gADCResult0;
volatile uint16_t gADCResult1;
volatile uint16_t gADCResult2;
volatile uint16_t gADCResult3;
uint16_t gLastADCResult0;
uint16_t gLastADCResult1;
uint16_t gLastADCResult2;
uint16_t gLastADCResult3;
bool gCheckADC = false;

/* Initialize the current and last GPIO states to be equal */
uint16_t gGpioState = 0x0000;
uint16_t gLastGpioState = 0x0000;

/* Process Command will take the command from the I2C Rx and parse which command is to be issued */
void processCommand(uint8_t cmd);

void FW4Version(uint8_t cmd);

/* ADC Process will start the ADC and separate the ADC result into 8 bits to place into the Tx Packet buffer. */
void ADC4Process(void);

void But4Process(void);

void DL_GPIO_init(uint32_t pincmIndex);

// GPIO Init
void DL_GPIO_init(uint32_t pincmIndex)
{
    IOMUX->SECCFG.PINCM[pincmIndex] =
        (IOMUX_PINCM_INENA_ENABLE | IOMUX_PINCM_PC_CONNECTED | ((uint32_t) 0x00000001));
}

int main(void)
{
    SYSCFG_DL_init();

    // Buttons GPIO init
    DL_GPIO_init(GPIO_BUTTONS_J1_SW_IOMUX);
    DL_GPIO_init(GPIO_BUTTONS_J2_SW_IOMUX);
    DL_GPIO_init(GPIO_BUTTONS_R_BUT_IOMUX);
    DL_GPIO_init(GPIO_BUTTONS_R_BUT1_IOMUX);
    DL_GPIO_init(GPIO_BUTTONS_BUT1_IOMUX);
    DL_GPIO_init(GPIO_BUTTONS_BUT2_IOMUX);
    DL_GPIO_init(GPIO_BUTTONS_BUT3_IOMUX);
    DL_GPIO_init(GPIO_BUTTONS_BUT4_IOMUX);
    DL_GPIO_init(GPIO_BUTTONS_BUT6_IOMUX);
    DL_GPIO_init(GPIO_BUTTONS_BUT7_IOMUX);
    DL_GPIO_init(GPIO_BUTTONS_BUT9_IOMUX);
    DL_GPIO_init(GPIO_BUTTONS_BUT10_IOMUX);

    DL_GPIO_disableOutput(GPIO_BUTTONS_PORT, GPIO_BUTTONS_J1_SW_PIN);
    DL_GPIO_disableOutput(GPIO_BUTTONS_PORT, GPIO_BUTTONS_J2_SW_PIN);
    DL_GPIO_disableOutput(GPIO_BUTTONS_PORT, GPIO_BUTTONS_R_BUT_PIN);    
    DL_GPIO_disableOutput(GPIO_BUTTONS_PORT, GPIO_BUTTONS_R_BUT1_PIN);
    DL_GPIO_disableOutput(GPIO_BUTTONS_PORT, GPIO_BUTTONS_BUT1_PIN);
    DL_GPIO_disableOutput(GPIO_BUTTONS_PORT, GPIO_BUTTONS_BUT2_PIN);
    DL_GPIO_disableOutput(GPIO_BUTTONS_PORT, GPIO_BUTTONS_BUT3_PIN);
    DL_GPIO_disableOutput(GPIO_BUTTONS_PORT, GPIO_BUTTONS_BUT4_PIN);
    DL_GPIO_disableOutput(GPIO_BUTTONS_PORT, GPIO_BUTTONS_BUT6_PIN);
    DL_GPIO_disableOutput(GPIO_BUTTONS_PORT, GPIO_BUTTONS_BUT7_PIN);
    DL_GPIO_disableOutput(GPIO_BUTTONS_PORT, GPIO_BUTTONS_BUT9_PIN);
    DL_GPIO_disableOutput(GPIO_BUTTONS_PORT, GPIO_BUTTONS_BUT10_PIN);
    
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(GPIO_BUTTONS_INT_IRQN);

    NVIC_EnableIRQ(I2C_INST_INT_IRQN);

    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    DL_TimerG_startCounter(TIMER_0_INST);

    DL_UART_Main_transmitData(UART_0_INST, 'Y');

    while (1)
    {
        /* Sleep until the stop command is given from the I2C Controller */
        while(gStopReceived == false)
            __WFE();
        gStopReceived = false;

        /* Process given commands */
        processCommand(gRxPacket);
    }
}

/* Process the command given */
void processCommand(uint8_t cmd){
    DL_UART_Main_transmitData(UART_0_INST, '0' + cmd);

    switch(cmd){
        case PHYHANDHELD_FW_VER:
            //FWVersion(cmd);
            FW4Version(cmd);
            break;
        case PHYHANDHELD_SWRST:
            break;
        case PHYHANDHELD_BUT_BITMAP:
            //ButProcess();
            But4Process();
            break;
        case PHYHANDHELD_ADC:
            ADC4Process();
            break;
        default:
            break;
    }
}

void FW4Version(uint8_t cmd){
    gTxPacket[0] = (uint8_t)((FW_VERSION >> 8) & 0xFF);
    gTxPacket[1] = (uint8_t)(FW_VERSION & 0xFF);
    gTxPacket[2] = 0;
    gTxPacket[3] = 0;
    gTxPacket[4] = 0;
    gTxPacket[5] = 0;
    gTxPacket[6] = 0;
    gTxPacket[7] = 0;

    /* Fill TX FIFO with firmware version */
    DL_I2C_fillTargetTXFIFO(I2C_INST, gTxPacket, I2C_TX_LENGTH);
}

/* Start the ADC process of sampling and storing into the TxPacket Buffer */
void ADC4Process(void){
    uint16_t diff0;
    uint16_t diff1;
    uint16_t diff2;
    uint16_t diff3;

    /* Start the ADC Sampling */

    DL_ADC12_startConversion(ADC12_0_INST);

    /* Sleep until the ADC sampling and conversion is finished, then reset the boolean */
    while(gCheckADC == false){
        __WFE();
    }

    gADCResult0 =
                DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);
    gADCResult1 =
                DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_1);
    gADCResult2 =
                DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_2);
    gADCResult3 =
                DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_3);
    
    gCheckADC = false;

    diff0 = abs(gADCResult0 - gLastADCResult0);
    diff1 = abs(gADCResult1 - gLastADCResult1);
    diff2 = abs(gADCResult2 - gLastADCResult2);
    diff3 = abs(gADCResult3 - gLastADCResult3);
    if((diff0 > 0x50) || (diff1 > 0x50) || (diff2 > 0x50) || (diff3 > 0x50))
    {
        //DL_GPIO_togglePins(GPIO_LEDS_PORT, GPIO_LEDS_LED_GREEN_PIN);
        DL_GPIO_togglePins(GPIO_INT_PORT, GPIO_INT_OUT_PIN);

        gLastADCResult0 = gADCResult0;
        gLastADCResult1 = gADCResult1;
        gLastADCResult2 = gADCResult2;
        gLastADCResult3 = gADCResult3;
    }

    gTxPacket[0] = (uint8_t)((gADCResult0 >> 8) & 0xFF);
    gTxPacket[1] = (uint8_t)(gADCResult0 & 0xFF);
    gTxPacket[2] = (uint8_t)((gADCResult1 >> 8) & 0xFF);
    gTxPacket[3] = (uint8_t)(gADCResult1 & 0xFF);
    gTxPacket[4] = (uint8_t)((gADCResult2 >> 8) & 0xFF);
    gTxPacket[5] = (uint8_t)(gADCResult2 & 0xFF);
    gTxPacket[6] = (uint8_t)((gADCResult3 >> 8) & 0xFF);
    gTxPacket[7] = (uint8_t)(gADCResult3 & 0xFF);

    //DL_ADC12_stopConversion(ADC12_0_INST);

    /* Fill TX FIFO with firmware version */
    DL_I2C_fillTargetTXFIFO(I2C_INST, gTxPacket, I2C_TX_LENGTH);
    
    DL_ADC12_enableConversions(ADC12_0_INST);
}

// GPIO Status read
uint8_t DL_GPIO_readPinStatus(uint32_t pins)
{
    if(DL_GPIO_readPins(GPIO_BUTTONS_PORT,pins))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void But4Process(void) {
    gTxPacket[0] = (uint8_t)((gGpioState >> 8) & 0xFF);
    gTxPacket[1] = (uint8_t)(gGpioState & 0xFF);
    gTxPacket[2] = 0;
    gTxPacket[3] = 0;
    gTxPacket[4] = 0;
    gTxPacket[5] = 0;
    gTxPacket[6] = 0;
    gTxPacket[7] = 0;

    DL_I2C_fillTargetTXFIFO(I2C_INST, gTxPacket, I2C_TX_LENGTH);
    
    return;
}

void ADC12_0_INST_IRQHandler(void) {
  switch (DL_ADC12_getPendingInterrupt(ADC12_0_INST)) {
  case DL_ADC12_IIDX_MEM3_RESULT_LOADED:
    gCheckADC = true;
    break;
  default:
    break;
  }
}

void I2C_INST_IRQHandler(void)
{
    static bool commandReceived = false;

    switch (DL_I2C_getPendingInterrupt(I2C_INST)) {
        case DL_I2C_IIDX_TARGET_START:
            //DL_I2C_fillTargetTXFIFO(I2C_INST, gTxPacket, I2C_TX_LENGTH);
            commandReceived   = false;
            break;
        case DL_I2C_IIDX_TARGET_RXFIFO_TRIGGER:
            commandReceived = true;

            /* Store received data in buffer */
            while (DL_I2C_isTargetRXFIFOEmpty(I2C_INST) != true) {
                gRxPacket = DL_I2C_receiveTargetData(I2C_INST);
            }
            break;
        case DL_I2C_IIDX_TARGET_TX_DONE:
            break;
        case DL_I2C_IIDX_TARGET_STOP:
            /* If commands were received, set the command count to RxCount and reset fifo */
            if (commandReceived == true) {
                commandReceived = false;
                DL_I2C_flushTargetTXFIFO(I2C_INST);
            }
            /* Set Stop Received to true to allow the main loop to proceed */
            gStopReceived = true;
            break;
        default:
            break;
    }
}

void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_MAIN_IIDX_EOT_DONE:
            break;
        case DL_UART_MAIN_IIDX_DMA_DONE_TX:
            break;
        default:
            break;
    }
}

void GROUP1_IRQHandler(void)
{
    gGpioState = 0xffff;
    
    /* Store the current pin state */
    gGpioState = (DL_GPIO_readPinStatus(GPIO_BUTTONS_BUT10_PIN) << 0) |
                 (DL_GPIO_readPinStatus(GPIO_BUTTONS_BUT9_PIN) << 1) |
                 (1 << 2) |
                 (DL_GPIO_readPinStatus(GPIO_BUTTONS_BUT7_PIN) << 3) |
                 (DL_GPIO_readPinStatus(GPIO_BUTTONS_BUT6_PIN) << 4) |
                 (1 << 5) |
                 (DL_GPIO_readPinStatus(GPIO_BUTTONS_BUT4_PIN) << 6) |
                 (DL_GPIO_readPinStatus(GPIO_BUTTONS_BUT3_PIN) << 7) |
                 (DL_GPIO_readPinStatus(GPIO_BUTTONS_BUT2_PIN) << 8) |
                 (DL_GPIO_readPinStatus(GPIO_BUTTONS_BUT1_PIN) << 9) |
                 (1 << 10) |
                 (DL_GPIO_readPinStatus(GPIO_BUTTONS_J2_SW_PIN) << 11) |
                 (DL_GPIO_readPinStatus(GPIO_BUTTONS_J1_SW_PIN) << 12) |
                 (DL_GPIO_readPinStatus(GPIO_BUTTONS_R_BUT1_PIN) << 13) |
                 (DL_GPIO_readPinStatus(GPIO_BUTTONS_R_BUT_PIN) << 14) |
                 (1 << 15);

    /* Loop through all pins */
    for (int i = 0; i < 16; i++) {
        /* Check if the current pin state changed */
        if (((gGpioState >> i) & 0x1) != ((gLastGpioState >> i) & 0x1)) {
            /* Set LED to indicate start of transfer */
            DL_GPIO_togglePins(GPIO_INT_PORT, GPIO_INT_OUT_PIN);

        }
    }
    gLastGpioState = gGpioState;
}

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            //DL_GPIO_togglePins(GPIO_LEDS_PORT, GPIO_LEDS_LED_GREEN_PIN);
            gRxPacket = PHYHANDHELD_ADC;
            gStopReceived = true;
            break;
        default:
            break;
    }
}
