/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
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
 * --/COPYRIGHT--*/
/******************************************************************************
 * MSP432 UART - with 24MHz DCO BRCLK
 *
 *
 *  MCLK = HSMCLK = SMCLK = DCO of 24MHz
 *
 *               MSP432P401            NRF52-DK
 *             -----------------      ----------
 *            |                 |    |          |
 *       RST -|     P3.3/UCA0TXD|----|P0.08     |
 *            |                 |    |          |
 *           -|                 |    |          |
 *            |     P3.2/UCA0RXD|----|P0.06     |
 *            |                 |    |          |
 *            |                 |    |          |
 *            |                 |    |          |
 *
 *******************************************************************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include "LcdDriver/HAL_MSP_EXP432P401R_Crystalfontz128x128_ST7735.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* UART Configuration Parameter. These are the configuration parameters to
 * make the eUSCI A UART module to operate with a 115200 baud rate. These
 * values were calculated using the online calculator that TI provides
 * at:
 * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 */
const eUSCI_UART_ConfigV1 uartConfig =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        13,                                      // BRDIV = 13
        0,                                       // UCxBRF = 0
        37,                                      // UCxBRS = 37
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_MSB_FIRST,                  // MSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling
        EUSCI_A_UART_8_BIT_LEN                  // 8 bit data length
};

/* Due to different endianess in the two boards we need to change the bit order */
uint8_t bit_invertion(uint8_t val){
    val = (val & 0xF0) >> 4 | (val & 0x0F) << 4;
    val = (val & 0xCC) >> 2 | (val & 0x33) << 2;
    val = (val & 0xAA) >> 1 | (val & 0x55) << 1;
    return val;
}

/* State Initialization */
typedef enum {
    IDLE,        // When Display the Name
    TEMPERATURE, // When Display the Temperature
    HUMIDITY,    // When Display the Humidity
    AIR          // When Display the Air Condition
}State_t;

State_t current_state = IDLE;

/* Initialization the variable for UART */
uint8_t TXData;          // Data to transmit
uint8_t RXData[2] = {0}; // Received Data
uint8_t rec_index = 0;   // Index to control order of received data

/* Graphic library context */
Graphics_Context g_sContext;

/* ADC results buffer */
static uint16_t resultsBuffer;

/* Joystick Initialization */
void _adcInit(){
    /* Configures Pin 6.0 and 4.4 as ADC input */
        GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION);
        GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);

        /* Initializing ADC (ADCOSC/64/8) */
        ADC14_enableModule();
        ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8, 0);

        /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM1 (A15, A9)  with repeat)
             * with internal 2.5v reference */
        ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, true);
        ADC14_configureConversionMemory(ADC_MEM0,
                ADC_VREFPOS_AVCC_VREFNEG_VSS,
                ADC_INPUT_A15, ADC_NONDIFFERENTIAL_INPUTS);

        ADC14_configureConversionMemory(ADC_MEM1,
                ADC_VREFPOS_AVCC_VREFNEG_VSS,
                ADC_INPUT_A9, ADC_NONDIFFERENTIAL_INPUTS);

        /* Enabling the interrupt when a conversion on channel 1 (end of sequence)
         *  is complete and enabling conversions */
        ADC14_enableInterrupt(ADC_INT1);

        /* Enabling Interrupts */
        Interrupt_enableInterrupt(INT_ADC14);
        Interrupt_enableMaster();

        /* Setting up the sample timer to automatically step through the sequence
         * convert.
         */
        ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

        /* Triggering the start of the sample */
        ADC14_enableConversion();
        ADC14_toggleConversionTrigger();
}

/* Graphics Initialization */
void _graphicsInit()
{
    /* Initializes display */
    Crystalfontz128x128_Init();

    /* Set default screen orientation */
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);

    /* Initializes graphics context */
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128,
                         &g_sCrystalfontz128x128_funcs);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    Graphics_clearDisplay(&g_sContext);

}

/* Hardware Initialization */
void _hwInit()
{
    /* Halting WDT and disabling master interrupts */
    WDT_A_holdTimer();
    Interrupt_disableMaster();

    /* Selecting P3.2 and P3.3 in UART mode and P1.0 as output (LED) */
        GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
                 GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
        GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    /* Set the core voltage level to VCORE1 */
    PCM_setCoreVoltageLevel(PCM_VCORE1);

    /* Set 2 flash wait states for Flash bank 0 and 1*/
    FlashCtl_setWaitState(FLASH_BANK0, 2);
    FlashCtl_setWaitState(FLASH_BANK1, 2);

    /* Initializes Clock System */
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_24);
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    /* Configuring UART Module */
    UART_initModule(EUSCI_A2_BASE, &uartConfig);

    /* Enable UART module */
    UART_enableModule(EUSCI_A2_BASE);

    /* Enabling interrupts */
    UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    Interrupt_enableInterrupt(INT_EUSCIA2);
    Interrupt_enableSleepOnIsrExit();

    _graphicsInit();
    _adcInit();
}

int main(void)
{
    // Hardware Initialization
    _hwInit();

    while(1)
    {
        int j = 0;         // value to create dynamic string
        char string[20];   // String to Display
        /* Checking state in order to decide what to display
         * IDLE: "SmartSense" in the middle
         * TEMPERATURE: "Temperature:" in top-middle, "val.val C" in the middle
         * HUMIDITY: "Humidity:" in top-middle, "val %" in the middle
         * AIR: "Air Condition:" in top-middle, "val ppm" in the middle
         */
        if (current_state == TEMPERATURE){
            sprintf(string, "Temperature:");
            Graphics_drawStringCentered(&g_sContext, (int8_t *) string,
                                AUTO_STRING_LENGTH, 64, 30, OPAQUE_TEXT);

            j = sprintf(string, "%u.", RXData[0]);
            j += sprintf(string+j, "%u", RXData[1]);
            Graphics_drawStringCentered(&g_sContext, (int8_t *) string, 5, 55, 70, OPAQUE_TEXT);

            sprintf(string, "C");
            Graphics_drawStringCentered(&g_sContext, (int8_t *) string, 5, 80, 70, OPAQUE_TEXT);
        } else if (current_state == HUMIDITY){
            sprintf(string, "Humidity:");
            Graphics_drawStringCentered(&g_sContext, (int8_t *) string,
                    AUTO_STRING_LENGTH, 64, 30, OPAQUE_TEXT);

            sprintf(string, "%u", RXData[0]);
            Graphics_drawStringCentered(&g_sContext, (int8_t *) string, 5, 55, 70, OPAQUE_TEXT);

            sprintf(string, "%%");
            Graphics_drawStringCentered(&g_sContext, (int8_t *) string, 5, 80, 70, OPAQUE_TEXT);
        } else if (current_state == AIR){
            sprintf(string, "Air Condition:");
            Graphics_drawStringCentered(&g_sContext, (int8_t *) string,
                    AUTO_STRING_LENGTH, 64, 30, OPAQUE_TEXT);

            sprintf(string, "%u", RXData[0] + (RXData[1]*256));
            Graphics_drawStringCentered(&g_sContext, (int8_t *) string, 5, 55, 70, OPAQUE_TEXT);

            sprintf(string, "ppm");
            Graphics_drawStringCentered(&g_sContext, (int8_t *) string, 5, 80, 70, OPAQUE_TEXT);
        } else if (current_state == IDLE){
            sprintf(string, "SmartSense");
            Graphics_drawStringCentered(&g_sContext, (int8_t *) string,
                                            AUTO_STRING_LENGTH, 60, 60, OPAQUE_TEXT);

        }

        /* Initialize Interrupt */
        Interrupt_enableSleepOnIsrExit();
        PCM_gotoLPM0InterruptSafe();
    }
}



/* Interrupt UART: When a Data arrive from UART this function is called,
 * And save the value in the RXData buffer.
 * Every new information come in two Data Packet */
void EUSCIA2_IRQHandler(void)
{
    uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A2_BASE);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        RXData[rec_index++] = bit_invertion(UART_receiveData(EUSCI_A2_BASE));
        if (rec_index == 2) {
            rec_index = 0;
        }
        Interrupt_disableSleepOnIsrExit();
    }

}

//Variable in order to change only one time every joystick move
uint8_t changed = 0;

/* Interrupt Joystick: Detect when Joystick move or is pressed.
 * Used to change state of the machine:
 * Temperature -> Humidity -> Air Condition -> Temperature
 * IDLE -> *press* -> Temperature
 * Every stage -> *press* -> IDLE */
void ADC14_IRQHandler(void)
{
    uint64_t status;

    /* Check Joystick interrupt flag status */
    status = ADC14_getEnabledInterruptStatus();
    ADC14_clearInterruptFlag(status);

    /* ADC_MEM1 conversion completed */
    if(status & ADC_INT1)
    {
        /* Store ADC14 conversion results */
        resultsBuffer = ADC14_getResult(ADC_MEM0);

        /* Joystick.X < 300 = moved to left
         * Joystick.X > 10000 = moved to right */
        if(resultsBuffer<300){
            if (changed == 0){
                Graphics_clearDisplay(&g_sContext); // Clear screen before changing state
                if(current_state==TEMPERATURE){
                    TXData = 3;
                    uint8_t temp = bit_invertion(TXData);
                    UART_transmitData(EUSCI_A2_BASE, temp);
                    current_state=AIR;
                } else if(current_state==AIR){
                    TXData = 2;
                    uint8_t temp = bit_invertion(TXData);
                    UART_transmitData(EUSCI_A2_BASE, temp);
                    current_state=HUMIDITY;
                } else if(current_state==HUMIDITY){
                    TXData = 1;
                    uint8_t temp = bit_invertion(TXData);
                    UART_transmitData(EUSCI_A2_BASE, temp);
                    current_state=TEMPERATURE;
                }
                changed++;
            }
        } else if(resultsBuffer>10000){
             if (changed == 0){
                 Graphics_clearDisplay(&g_sContext); // Clear screen before changing state
                 if(current_state==TEMPERATURE){
                     TXData = 2;
                     uint8_t temp = bit_invertion(TXData);
                     UART_transmitData(EUSCI_A2_BASE, temp);
                     current_state=HUMIDITY;
                  } else if(current_state==HUMIDITY){
                     TXData = 3;
                     uint8_t temp = bit_invertion(TXData);
                     UART_transmitData(EUSCI_A2_BASE, temp);
                     current_state=AIR;
                  } else if(current_state==AIR){
                     TXData = 1;
                     uint8_t temp = bit_invertion(TXData);
                     UART_transmitData(EUSCI_A2_BASE, temp);
                     current_state=TEMPERATURE;
                  }
                 changed++;
             }
        } else {
            if (changed == 1) {changed = 0;}
        }

        /* Check if Joystick is pressed */
        if (!(P4IN & GPIO_PIN1)){
            Graphics_clearDisplay(&g_sContext); // Clear screen before changing state
            if(current_state==IDLE){
                current_state=TEMPERATURE;
            } else {
                TXData = 1;
                uint8_t temp = bit_invertion(TXData);
                UART_transmitData(EUSCI_A2_BASE, temp);
                current_state=IDLE;
            }
        }
    }
}

