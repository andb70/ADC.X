#pragma config FPLLIDIV = DIV_2 // PLL Input Divider (1x Divider)
#pragma config FPLLMUL = MUL_20 // PLL Multiplier (24x Multiplier)
#pragma config UPLLIDIV = DIV_2 // USB PLL Input Divider (12x Divider)
#pragma config UPLLEN = OFF // USB PLL Enable (Disabled and Bypassed)
#pragma config FPLLODIV = DIV_1 // System PLL Output Clock Divider (PLL Divide by 256)
// DEVCFG1
#pragma config FNOSC = PRIPLL // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = ON // Secondary Oscillator Enable (Enabled)
#pragma config IESO = ON // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = HS // Primary Oscillator Configuration (HS osc mode)
#pragma config OSCIOFNC = ON // CLKO Output Signal Active on the OSCO Pin (Enabled)
#pragma config FPBDIV = DIV_8 // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)
#pragma config FCKSM = CSDCMD // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576 // Watchdog Timer Postscaler (1:1048576)
#pragma config FWDTEN = OFF // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
// DEVCFG0
#pragma config DEBUG = OFF // Background Debugger Enable (Debugger is disabled)
#pragma config ICESEL = ICS_PGx2 // ICE/ICD Comm Channel Select (ICE EMUC2/EMUD2 pins shared with PGC2/PGD2)
#pragma config PWP = OFF // Program Flash Write Protect (Disable)
#pragma config BWP = OFF // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF // Code Protect (Protection Disabled)

#define SYS_FREQ 			(80000000L)


// nella slide le righe vanno a capo, nel codice ogni #define va su un?unica riga
#include <p32xxxx.h> // Include PIC32 specifics header file
#include <plib.h> // Include the PIC32 Peripheral Library
#include <stdio.h>
#include <stdlib.h>
#include "config.h"
#include "../display7s.X/display7s.h"
#include "../timers1.X/timers.h"
#include "../sn74hc595.X/shift595.h"
#include "analogIn.h"


// LEDS
// on board GREEN
#define monitorGLed(state)  LATGbits.LATG6 = state
#define monitorGLedToggle() LATGbits.LATG6 = !LATGbits.LATG6

// on board YELLOW
#define monitorYLed(state)  LATDbits.LATD1 = state
#define monitorYLedToggle() LATDbits.LATD1 = !LATDbits.LATD1
void initLeds();

// BUTTONS
void checkButtons(char timerId);
// btn1 = RD2, btn2 = RD3
#define POS_BTN_1 0x2
#define POS_BTN_2 0x3
#define BTN_MASK (1<<POS_BTN_1 | 1<<POS_BTN_2)

union _buttons {
    struct {
        unsigned BTN_1_UP: 1;
        unsigned BTN_1_DOWN: 1;
        unsigned BTN_2_UP: 1;
        unsigned BTN_2_DOWN: 1;
    };
    struct {
        unsigned all: 4; // all buttons
        unsigned : 3; // pad
        unsigned debounce: 1; // true for few ms after a BTN_DOWN or BTN_UP 
    };
} buttons;
unsigned int oldPortD;

// GLOBALS
int count = 0;
void initGlobals();

int main() {
    
    initLeds();
    initTimers();
    // Initialize port pins to be used with display
    initShift595();
    initializeADC(); // Initialize the ADC10
    initGlobals();
    
    char adc_index = 0;
    unsigned int a;
    unsigned int i=0;
    float r[2];
    
#define MIN_ADC0    0x0+0x30
#define MAX_ADC0    0x3FF-0x30
#define MIN_ADC1    0x0
#define MAX_ADC1    0x200
#define MIN_OUT     0x0
#define MAX_OUT     0xF
    
    while (1) 
    {
        checkTimers();
        checkButtons(0);
        checkAna();
        
        if (buttons.BTN_1_DOWN)
        {
            i=0; 
            monitorGLedToggle();
        }
        if (buttons.BTN_2_DOWN)
        {
            i=1; 
            monitorGLedToggle();
        }
        
        r[0] = linear(adc_value[0], MIN_ADC0, MAX_ADC0, MIN_OUT, MAX_OUT );
        r[1] = linear(adc_value[1], MIN_ADC1, MAX_ADC1, MIN_OUT, MAX_OUT );
        
        if (timer_usFinished(1))
        {
            shiftdata595(display7s[(char)r[i]]);
            timer_usSet(1, 10); // 50 kHz
        }
        if (heartbeat())
        {
            monitorYLedToggle();
        }
    }
    return 1;
}


void checkButtons(char timerId){
    unsigned int portD;
    unsigned int trig;
    union _buttons tmp;
    
    // if we are surveying a debounce after a BTN_[UP or DOWN], simply wait
    // until the period has elapsed and exit here
    if (timer_usRunning(timerId))
        return;
    
    // get the status of the port and check it against the old status
    portD = PORTD & BTN_MASK;
    
    // falling edge: when port is 0 and oldport is 1
    trig = ~portD & oldPortD;    
    tmp.BTN_1_DOWN = (trig >> POS_BTN_1);
    tmp.BTN_2_DOWN = (trig >> POS_BTN_2);
    
    // raising edge: when port is 1 and oldport is 0
    trig = portD & ~oldPortD;
    tmp.BTN_1_UP = (trig >> POS_BTN_1);
    tmp.BTN_2_UP = (trig >> POS_BTN_2);
    
    // if debounce period has just elapsed then assign the result trigger    
    if (buttons.debounce)
    {
        buttons.debounce = 0;
    }
    // or, if any button is being pressed/released then start the debounce period
    else if (tmp.all){
        buttons.debounce = 1;
        timer_msSet(timerId,20);
        return;
    }
    buttons = tmp;
    oldPortD = portD;
}


void initLeds() {
    mJTAGPortEnable(0); // Disable JTAG
    PORTSetPinsDigitalOut(IOPORT_B, BIT_13); // RED led @ D8
    PORTSetPinsDigitalOut(IOPORT_D, BIT_1); //  YELLOW led on-board
    PORTSetPinsDigitalOut(IOPORT_G, BIT_6); //  GREEN  led on-board    
}

void initGlobals() {
    // default state: all off
    LATB = 0;
    LATD = 0;
    LATG = 0;
    
    oldPortD = PORTD & BTN_MASK;
}