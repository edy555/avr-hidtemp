/* Name: main.c
 * Project: hid-data, example how to use HID for data transfer
 * Author: Christian Starkjohann
 * Creation Date: 2008-04-11
 * Tabsize: 4
 * Copyright: (c) 2008 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 * This Revision: $Id: main.c 777 2010-01-15 18:34:48Z cs $
 */

/*
This example should run on most AVRs with only little changes. No special
hardware resources except INT0 are used. You may have to change usbconfig.h for
different I/O pins for USB. Please note that USB D+ must be the INT0 pin, or
at least be connected to INT0 as well.
*/

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */
#include <avr/eeprom.h>
#include <avr/sleep.h>

#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv.h"
#include "oddebug.h"        /* This is also an example for using debug macros */

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

#if 1
PROGMEM char usbHidReportDescriptor[25] = {    /* USB report descriptor */
    0x06, 0x00, 0xff,              // USAGE_PAGE (Vendor Defined Page 1)
    0x09, 0x01,                    // USAGE (Vendor Usage 1)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x1f,              //   LOGICAL_MAXIMUM (8191)
    0x35, 0xe6,                    //   PHYSICAL_MINIMUM (-26)
    0x45, 0x65,                    //   PHYSICAL_MAXIMUM (101)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x09, 0x00,                    //   USAGE (Undefined)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0xc0                           // END_COLLECTION
#if 0
    0x05, 0x0c,                    // USAGE_PAGE (Consumer Device)
    0x09, 0x01,                    // USAGE (Consumer Control)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x1f,              //   LOGICAL_MAXIMUM (8191)
    0x35, 0xe6,                    //   PHYSICAL_MINIMUM (-26)
    0x45, 0x65,                    //   PHYSICAL_MAXIMUM (101)
    //0x67, 0x01, 0x00, 0x01, 0x00,  //   UNIT (K)
    //0x67, 0x01, 0xf0, 0xd1, 0x21,  //   UNIT (V)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (2)
    0x0a, 0x05, 0x01,              //   USAGE (Room Tempereture)
    //    0x82, 0x02, 0x01,              //   INPUT (Data,Var,Abs,Buf)
    0xb2, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
    0xc0                           // END_COLLECTION
#endif
};
#else
PROGMEM char usbHidReportDescriptor[22] = {    /* USB report descriptor */
    0x06, 0x00, 0xff,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    // USAGE (Vendor Usage 1)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x80,                    //   REPORT_COUNT (128)
    0x09, 0x00,                    //   USAGE (Undefined)
    0xb2, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
    0xc0                           // END_COLLECTION
};
#endif

/* Since we define only one feature report, we don't use report-IDs (which
 * would be the first byte of the report). The entire report consists of 128
 * opaque data bytes.
 */

/* The following variables store the status of the current data transfer */
static uchar    currentAddress;
static uchar    bytesRemaining;

typedef unsigned short ushort;

static uchar    adcPending;
static uchar    adcCount;
static ushort   adcHistory[8];
static ushort   adcValue;


/* ------------------------------------------------------------------------- */

unsigned int getAdcValue()
{
    unsigned int value = 0;
    int i;
    for (i = 0; i < 8; i++)
      value += adcHistory[i];
    return value;
}


/* usbFunctionRead() is called when the host requests a chunk of data from
 * the device. For more information see the documentation in usbdrv/usbdrv.h.
 */
uchar   usbFunctionRead(uchar *data, uchar len)
{
    if(len > bytesRemaining)
        len = bytesRemaining;
    //eeprom_read_block(data, (uchar *)0 + currentAddress, len);

    *(ushort*)data = (ushort)getAdcValue();
    currentAddress += len;
    bytesRemaining -= len;
    return len;
}

#define EEPROM_SERIAL_NUMBER ((uchar *)0)
#define SERIAL_NUMBER_LEN 8
static int serial_number_buf[SERIAL_NUMBER_LEN];

usbMsgLen_t usbFunctionDescriptor(usbRequest_t *rq)
{
  usbMsgLen_t len = 0;

  if (rq->wValue.bytes[1] == USBDESCR_STRING
      && rq->wValue.bytes[0] == 3 /*SERIAL_NUMBER*/) {
#if 1
    int i;
    for (i = 0; i < SERIAL_NUMBER_LEN; i++) {
      uchar c = eeprom_read_byte(EEPROM_SERIAL_NUMBER + i);
      serial_number_buf[i] = c;
    }
    len = serial_number_buf[0] * 2 + 2;
    serial_number_buf[0] = USB_STRING_DESCRIPTOR_HEADER(serial_number_buf[0]);
#else
    serial_number_buf[0] = USB_STRING_DESCRIPTOR_HEADER(4);
    serial_number_buf[1] = '1';
    serial_number_buf[2] = '2';
    serial_number_buf[3] = '3';
    serial_number_buf[4] = '4';
    len = 4 * 2 + 2;
#endif
    usbMsgPtr = (uchar *)serial_number_buf;
  }
  return len;
}

/* usbFunctionWrite() is called when the host sends a chunk of data to the
 * device. For more information see the documentation in usbdrv/usbdrv.h.
 */
uchar   usbFunctionWrite(uchar *data, uchar len)
{
    if(bytesRemaining == 0)
        return 1;               /* end of transfer */
    if(len > bytesRemaining)
        len = bytesRemaining;
    eeprom_write_block(data, EEPROM_SERIAL_NUMBER + currentAddress, len);
    currentAddress += len;
    bytesRemaining -= len;
    return bytesRemaining == 0; /* return 1 if this was the last chunk */
}

/* ------------------------------------------------------------------------- */

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* HID class request */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* since we have only one report type, we can ignore the report-ID */
            bytesRemaining = 2;
            currentAddress = 0;
            return USB_NO_MSG;  /* use usbFunctionRead() to obtain data */
        }else if(rq->bRequest == USBRQ_HID_SET_REPORT){
            /* since we have only one report type, we can ignore the report-ID */
            bytesRemaining = SERIAL_NUMBER_LEN;
            currentAddress = 0;
            return USB_NO_MSG;  /* use usbFunctionWrite() to receive data from host */
        }
    }else{
        /* ignore vendor type requests, we don't use any */
    }
    return 0;
}


static void adcPoll(void)
{
    if(adcPending && !(ADCSRA & (1 << ADSC))){
		adcPending = 0;
        adcHistory[adcCount++] = (ushort)ADC;

        if (adcCount >= 8)
          adcCount = 0;
    }
}

static void timerPoll(void)
{
static uchar timerCnt;

    if(TIFR & (1 << TOV1)){	//This flag is triggered at 60 hz.
        TIFR = (1 << TOV1); /* clear overflow */
		if(++timerCnt >= 31){		 /* ~ 0.5 second interval */
            timerCnt = 0;
			adcPending = 1;
			ADCSRA |= (1 << ADSC);  /* start next conversion */
		}
	}
}

static void timerInit(void)
{
    TCCR1 = 0x0b;           /* select clock: 16.5M/1k -> overflow rate = 16.5M/256k = 62.94 Hz */
}

/* ------------------------------------------------------------------------- */
static void adcInit(void)
{
  ADMUX = 0b10010010; /* vref = 2.56V, measure ADC2 (PB4) */
  ADCSRA = 0b10000111; /* enable ADC, not free running, interrupt disable, rate = 1/128 */

  adcCount = 0;
  adcPending = 0;
}

/* ------------------------------------------------------------------------- */
/* ------------------------ Oscillator Calibration ------------------------- */
/* ------------------------------------------------------------------------- */

/* Calibrate the RC oscillator to 8.25 MHz. The core clock of 16.5 MHz is
 * derived from the 66 MHz peripheral clock by dividing. Our timing reference
 * is the Start Of Frame signal (a single SE0 bit) available immediately after
 * a USB RESET. We first do a binary search for the OSCCAL value and then
 * optimize this value with a neighboorhod search.
 * This algorithm may also be used to calibrate the RC oscillator directly to
 * 12 MHz (no PLL involved, can therefore be used on almost ALL AVRs), but this
 * is wide outside the spec for the OSCCAL value and the required precision for
 * the 12 MHz clock! Use the RC oscillator calibrated to 12 MHz for
 * experimental purposes only!
 */
static void calibrateOscillator(void)
{
uchar       step = 128;
uchar       trialValue = 0, optimumValue;
int         x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);

    /* do a binary search: */
    do{
        OSCCAL = trialValue + step;
        x = usbMeasureFrameLength();    /* proportional to current real frequency */
        if(x < targetValue)             /* frequency still too low */
            trialValue += step;
        step >>= 1;
    }while(step > 0);
    /* We have a precision of +/- 1 for optimum OSCCAL here */
    /* now do a neighborhood search for optimum value */
    optimumValue = trialValue;
    optimumDev = x; /* this is certainly far away from optimum */
    for(OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++){
        x = usbMeasureFrameLength() - targetValue;
        if(x < 0)
            x = -x;
        if(x < optimumDev){
            optimumDev = x;
            optimumValue = OSCCAL;
        }
    }
    OSCCAL = optimumValue;
}
/*
Note: This calibration algorithm may try OSCCAL values of up to 192 even if
the optimum value is far below 192. It may therefore exceed the allowed clock
frequency of the CPU in low voltage designs!
You may replace this search algorithm with any other algorithm you like if
you have additional constraints such as a maximum CPU clock.
For version 5.x RC oscillators (those with a split range of 2x128 steps, e.g.
ATTiny25, ATTiny45, ATTiny85), it may be useful to search for the optimum in
both regions.
*/

#define EEPROM_OSCCAL	((uchar *)511)

void    usbEventResetReady(void)
{
    calibrateOscillator();
    eeprom_write_byte(EEPROM_OSCCAL, OSCCAL);   /* store the calibrated value in EEPROM */
}

/* ------------------------------------------------------------------------- */

#define WHITE_LED 3
#define YELLOW_LED 1

typedef struct{
  ushort adcvalue;
}report_t;

static report_t reportBuffer;

int main(void)
{
    uchar   i;

    /* calibration value from last time */
    uchar calibrationValue = eeprom_read_byte(EEPROM_OSCCAL); 
    if(calibrationValue != 0xff){
        OSCCAL = calibrationValue;
    }

    wdt_enable(WDTO_1S);
    /* Even if you don't use the watchdog, turn it off here. On newer devices,
     * the status of the watchdog (on/off, period) is PRESERVED OVER RESET!
     */
    /* RESET status: all port bits are inputs without pull-up.
     * That's the way we need D+ and D-. Therefore we don't need any
     * additional hardware initialization.
     */
    odDebugInit();
    DBG1(0x00, 0, 0);       /* debug output: main starts */

    sbi(DDRB, WHITE_LED);
    sbi(DDRB, YELLOW_LED);
    cbi(DDRB, 4);
    cbi(PORTB, 4);
    sbi(MCUCR, PUD);

	timerInit();	//Create a timer that will trigger a flag at a ~60hz rate 
    adcInit();		//Setup the ADC conversions
    usbInit();
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    i = 0;
    while(--i){             /* fake USB disconnect for > 250 ms */
        wdt_reset();
        _delay_ms(1);
    }
    usbDeviceConnect();
    //set_sleep_mode(SLEEP_MODE_PWR_SAVE);
    //sleep_mode();
    //sleep_enable();
    sei();
    DBG1(0x01, 0, 0);       /* debug output: main loop starts */
    for(;;){                /* main event loop */
        DBG1(0x02, 0, 0);   /* debug output: main loop iterates */
        //sleep_cpu();
        sbi(PORTB, YELLOW_LED);
        wdt_reset();
        usbPoll();
        cbi(PORTB, YELLOW_LED);

        if(usbInterruptIsReady()){
            /* called after every poll of the interrupt endpoint */
            reportBuffer.adcvalue = getAdcValue();
            DBG1(0x03, 0, 0);   /* debug output: interrupt report prepared */
            usbSetInterrupt((void *)&reportBuffer, sizeof(reportBuffer));
        }

        timerPoll();
        adcPoll();
        _delay_ms(5);
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
