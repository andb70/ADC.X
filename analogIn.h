/*
 * released under GNU AFFERO GENERAL PUBLIC LICENSE Version 3, 19 November 2007
 * 
 *      Author:     Andrea Biasutti
 *      Date:       July 5th 2019
 *      Hardware:   PIC32MX440256H
 * 
 * source available here https://github.com/andb70/ADC.X
 * 
 */
#ifndef _ANALOG_IN
#define _ANALOG_IN

#include <p32xxxx.h> // Include PIC32 specifics header file
#include <plib.h> // Include the PIC32 Peripheral Library
#include "config.h"

// ADC
/* A/D Interrupt at the completion of conversion for each ADC_NUM-th sample */
#if (ADC_NUM ==1)
#define ADC_SAMPLES_PER_INT   ADC_SAMPLES_PER_INT_1
#else
#define ADC_SAMPLES_PER_INT     ((ADC_NUM-1) << _AD1CON2_SMPI_POSITION) 
#endif

#define CONFIG1 (ADC_MODULE_ON | ADC_FORMAT_INTG32 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON)
#define CONFIG2 (ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | \
                ADC_SAMPLES_PER_INT | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF) 
#define CONFIG3 (ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_15)
#define CONFIGPORT (ENABLE_AN1_ANA | ENABLE_AN2_ANA | ENABLE_AN3_ANA | ENABLE_AN4_ANA)
#define CONFIGSCAN (SKIP_SCAN_AN0 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 | SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | \
                    SKIP_SCAN_AN10 | SKIP_SCAN_AN11 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | \
                    SKIP_SCAN_AN15)

// scaling of a analog value
//#define linear(v, mI, MI, mO, MO)   ( (v-mI) / (MI-mI)*(MO-mO) +mO)
unsigned int adc_value[ADC_NUM];

int linear(int x, int in_min, int in_max, int out_min, int out_max);
void initializeADC();
void checkAna();

#endif