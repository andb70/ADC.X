#include <p32xxxx.h> // Include PIC32 specifics header file
#include <plib.h> // Include the PIC32 Peripheral Library
#include <stdio.h>
#include <stdlib.h>
#include "analogIn.h"

char adc_ready = 0;// flag set by adc interrupt
char adc_index = 0;
unsigned int i=0;

void checkAna(){
    if (adc_ready)
    {
        adc_ready = 0;
        for (adc_index = 0; adc_index < ADC_NUM; adc_index++)
        {
            adc_value[adc_index] = ReadADC10(adc_index);
        }

    }
}

int linear(int x, int in_min, int in_max, int out_min, int out_max)
{
    if (x < in_min)
        return out_min;
    if (x > in_max)
        return out_max;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void initializeADC() {

    CloseADC10(); // Generally, you should disable the ADC before setup.
    // Use ground as negative reference for channel A instead of pin AN1 (RB1)
    SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF);
    OpenADC10(CONFIG1, CONFIG2, CONFIG3, CONFIGPORT, CONFIGSCAN);
    // Setup for the ADC10.
    ConfigIntADC10(ADC_INT_ON | ADC_INT_PRI_3);
    EnableADC10(); // Enables the ADC10.
}

//
void __ISR(_ADC_VECTOR, ipl3) ADC10Handler(void)
{
	adc_ready = 1;
	mAD1ClearIntFlag();
}
