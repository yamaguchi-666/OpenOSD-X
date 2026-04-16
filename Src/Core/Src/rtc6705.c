#include <stdbool.h>
#include <stdio.h>
#include "main.h"
#include "char_canvas.h"
#include "rtc6705.h"
//#include "targets.h"
//#include "common.h"
//#include "gpio.h"

extern SPI_HandleTypeDef hspi2;


//static uint32_t powerUpAfterSettleTime = 0;


void rtc6705writeRegister(uint8_t addr, uint32_t data)
{
    uint8_t tx[5];

    tx[0] = addr | 0x10;
    tx[1] = data & 0x1f;
    tx[2] = ((data)>>5) & 0x1f;
    tx[3] = ((data)>>10) & 0x1f;
    tx[4] = ((data)>>15) & 0x1f;

    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, tx, 5, 100);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
}

void rtc6705readRegister(uint8_t addr, uint32_t *data)
{
    addr &= 0xf;

    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &addr, 1, 100);
    uint8_t rx[4];
    HAL_SPI_Receive(&hspi2, rx, 4, 100);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

    *data = rx[0] | 
            ((rx[1] & 0x1f)<<5) |
            ((rx[2] & 0x1f)<<10) |
            ((rx[3] & 0x1f)<<15);
}

uint32_t initRtc6705(void)
{
    rtc6705writeRegister(StateRegister, 0);
    return 0;
}

uint32_t initRtc6705check(void)
{
    uint32_t rx;
    rtc6705readRegister(StateRegister, &rx);
    if ((rx & 0x7) == 0x2){
        return 1;
    }
    return 0;
}

void rtc6705PowerAmpOn(void)
{
    rtc6705writeRegister(PredriverandPAControlRegister, POWER_AMP_ON);
}

void rtc6705PowerAmpOff(void)
{
    rtc6705writeRegister(PredriverandPAControlRegister, 0);
}

void rtc6705WriteFrequency(uint16_t newFreq)
{
    uint32_t freq = newFreq * 1000U;
    freq /= 40;
    uint32_t SYN_RF_N_REG = freq / 64;
    uint32_t SYN_RF_A_REG = freq % 64;
    uint32_t EX_CAP = 70 - (newFreq - 5000) * 56 / 1000;

    rtc6705writeRegister(SynthesizerRegisterA, SYNTH_REG_A_DEFAULT);      // reset
    rtc6705writeRegister(SynthesizerRegisterB, SYN_RF_A_REG | (SYN_RF_N_REG << 7));
    rtc6705writeRegister(RFVCODFCControlRegister, (VCO_CONTROL<<6) | EX_CAP);  // VCO fine tune
    
}

