/*
             LUFA Library
     Copyright (C) Dean Camera, 2012.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2012  Dean Camera (dean [at] fourwalledcubicle [dot] com)
  Copyright 2012  Simon Foster (simon.foster [at] inbox [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file 
 *
 *  Main source file for the SerialToLCD program. This file contains the main tasks of
 *  the project and is responsible for the initial application hardware configuration.
 */

#include "etch.h"

/** Circular buffer to hold data from the host before it is sent to the LCD */
static RingBuffer_t Etch_Buffer;
static RingBuffer_t Response_Buffer;

/** Underlying data buffer for \ref FromHost_Buffer, where the stored bytes are located. */
static uint8_t      Etch_Buffer_Data[256];
static uint8_t      Response_Buffer_Data[128];

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber   = 0,
				.DataINEndpoint           =
					{
						.Address          = CDC_TX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.DataOUTEndpoint =
					{
						.Address          = CDC_RX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.NotificationEndpoint =
					{
						.Address          = CDC_NOTIFICATION_EPADDR,
						.Size             = CDC_NOTIFICATION_EPSIZE,
						.Banks            = 1,
					},
			},
	};


/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();

    RingBuffer_InitBuffer(&Etch_Buffer, Etch_Buffer_Data, sizeof(Etch_Buffer_Data));
    RingBuffer_InitBuffer(&Response_Buffer, Response_Buffer_Data, sizeof(Response_Buffer_Data));

	sei();

	for (;;)
	{
		/* Only try to read in bytes from the CDC interface if the transmit buffer is not full */
		if (!(RingBuffer_IsFull(&Etch_Buffer)))
		{
			int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

			/* Read bytes from the USB OUT endpoint into the USART transmit buffer */
			if (!(ReceivedByte < 0))
			  RingBuffer_Insert(&Etch_Buffer, ReceivedByte);
		}

        /* Timer should overflow every 1024us */
        if ( TIFR0 & _BV(TOV0))
        {
            uint16_t Buffer_Count;
            
            /* Clear timer expiry flag */
            TIFR0 |= _BV(TOV0);

            Buffer_Count = RingBuffer_GetCount(&Etch_Buffer);
            
            /* Note that we only send one byte of data at a time
               to the motor interface to ensure the correct stepping rate */
            if ( Buffer_Count ) 
            {
			    uint8_t Motor_Data = RingBuffer_Remove(&Etch_Buffer);
                  
                /* Motor interface */

                /* PD0..3 for Motor 1 */
                PORTD &= ( PORTD & 0xF0 );
                PORTD |= ( Motor_Data & 0x0F );

                /* PB4..7 for Motor 2 */
                PORTB &= ( PORTB & 0x0F );
                PORTB |= ( Motor_Data & 0xF0 );

                /* Queue the current etch buffer count to be sent back to the host
                   Host should monitor this to ensure no buffer overflow */
                RingBuffer_Insert( &Response_Buffer, (uint8_t)( Buffer_Count ));
            }
            
            /* Check if there's any data to be sent back to the host */
            Buffer_Count = RingBuffer_GetCount(&Response_Buffer);

            while (Buffer_Count)
            {
                /* Try to send the next byte of data to the host,
                   abort if there is an error without dequeuing */
                if (CDC_Device_SendByte(&VirtualSerial_CDC_Interface,
                         RingBuffer_Peek(&Response_Buffer)) != ENDPOINT_READYWAIT_NoError)
                {
                    break;
                }
    
                /* Dequeue the already sent byte from the buffer now we
                   have confirmed that no transmission error occurred */
                RingBuffer_Remove(&Response_Buffer);
                --Buffer_Count;
            }
        }
        
		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		USB_USBTask();
	}
}

/** Configures the board hardware and chip peripherals for the application's functionality. */
void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	/* Hardware Initialization */
	USB_Init();

	/* Set ports to output */
	DDRD |= 0x0F;
    DDRB |= 0xF0;
	
	/* Set the flush timer frequency to CLK(IO)/64
       -> Frequency is 250k
       -> overflow at 976.5625Hz
       -> every 1024 us */
	TCCR0B = (1 << CS01)|(1 << CS00);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}
