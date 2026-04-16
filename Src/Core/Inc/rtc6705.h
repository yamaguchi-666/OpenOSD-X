#ifndef __RTC6705_H
#define __RTC6705_H

#include <stdint.h>

#define SynthesizerRegisterA 0x00
#define SynthesizerRegisterB 0x01
#define SynthesizerRegisterC 0x02
#define RFVCODFCControlRegister 0x03
#define VCOControlRegister 0x04
#define VCOControlRegisterCont 0x05
#define AudioModulatorControlRegister 0x06
#define PredriverandPAControlRegister 0x07
#define StateRegister 0x0F

#define RTC6705_READ_BIT 0x00
#define RTC6705_WRITE_BIT 0x01

#define SYNTH_REG_A_DEFAULT 0x0190
#define POWER_AMP_ON 0x9f7e0        // 0b1001111101111110000

#define	VCO_CONTROL 0x3FF

#define MIN_FREQ 5000
#define MAX_FREQ 5999

#define PLL_SETTLE_TIME 500

uint32_t initRtc6705(void);
uint32_t initRtc6705check(void);
void rtc6705PowerAmpOn(void);
void rtc6705PowerAmpOff(void);
void rtc6705WriteFrequency(uint16_t newFreq);
#if 0
void rtc6705spiPinSetup(void);
void rtc6705writeRegister(uint32_t data);
uint32_t rtc6705readRegister(uint8_t reg);
void rtc6705ResetState(void);
void rtc6705ResetSynthRegA(void);
void rtc6705PowerAmpOn(void);
void rtc6705PowerAmpOff(void);
uint8_t rtc6705CheckFrequency(void);
void rtc6705PowerUpAfterPLLSettleTime();
#endif

#endif
