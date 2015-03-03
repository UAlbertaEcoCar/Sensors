/*
* Eco-Car - CAN Bus Master Node
* Based on CAN Bus Node A Demonstration
*/

#define DBG_LED0 LATBbits.LATB0
#define DBG_LED1 LATBbits.LATB1
#define DBG_LED2 LATBbits.LATB4
#define DBG_LED3 LATBbits.LATB5
#define ERR_LED  LATCbits.LATC5

#include <p18cxxx.h>
#include "..\Common\J1939.h"
#include "..\Common\ecocar.h"
#include "..\Common\AnalogHelper.h"
#include <delays.h>
#pragma config OSC = IRCIO67    // Oscillator Selection Bit
#pragma config BOREN = OFF      // Brown-out Reset disabled in hardware and software
#pragma config WDT = OFF        // Watchdog Timer disabled (control is placed on the SWDTEN bit)
#pragma config PBADEN = OFF
#pragma config LVP = OFF

J1939_MESSAGE Msg;

void main( void ) {
    unsigned char i = 0;
    // Pins C6 and C7 are used for UART TX and RX respectively
    InitEcoCar();
    ReadAnalog(0);
    J1939_Initialization( TRUE );

    LATB = 0;
    TRISB = 0;
    TRISC = 0;

    ShowBootupAnimation();

    while (J1939_Flags.WaitingForAddressClaimContention)
        J1939_Poll(5);

    // CANbus is now initialized and we can now loop while we check
    // our message receive buffer for new CANbus messages (where all received messages are put).
    while (1) {
        //Receive Messages
        J1939_Poll(10);
        while (RXQueueCount > 0) {
            J1939_DequeueMessage( &Msg );
            LATCbits.LATC5 = 1;

            if ( J1939_Flags.ReceivedMessagesDropped )
                J1939_Flags.ReceivedMessagesDropped = 0;
        }
    }
}
