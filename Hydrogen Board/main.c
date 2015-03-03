/*
* Eco-Car - H2 Sensor Board
 * CANbus slave node
*/

#include <p18cxxx.h>
#include "..\Common\J1939.h"
#include "..\Common\ecocar.h"
#include "..\Common\AnalogHelper.h"
#include <delays.h>

#pragma config OSC = IRCIO67    // Oscillator Selection Bit
#pragma config BOREN = OFF      // Brown-out Reset disabled in hardware and software
#pragma config WDT = OFF        // Watchdog Timer disabled (control is placed on the SWDTEN bit)
#pragma config PBADEN = OFF     // Port B analog off
#pragma config LVP = OFF        // Low voltage programming off

J1939_MESSAGE Msg;

const int _LOCAL_H2_OKAY_THRES_ = 2500; // Upper voltage threshold for okay
const int _REMOTE_H2_OKAY_THRES_ = 2500; // Upper voltage threshold for okay

void main( void )
{
    /** Not using InitEcoCar **/

    // Initialize registers:
    ADCON1 = 0x0F;                  // Disable analog inputs on all A ports.
                                    // AnalogHelper.c reenables them as it goes
    TRISA = 0b00000111;             // E-STOP is RA2
    TRISB = 0b00001000;             // Set B2 (CANTX) to output, B3 (CANRX) to input
    TRISC = 0b00000000;             // Set C to all output

    LATA = 0;                       // Reset A latches to low
    LATB = 0;                       // Reset B latches to low
    LATC = 0;                       // Reset C latches to low
    LATCbits.LATC7 = 1;             // Startup with H2_OK HIGH = Good to go
    
    // Run internal oscillator at 8 MHz
    OSCCONbits.IRCF2 = 1;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 1;

    // Multiply frequency using PLL to 4* = 32 MHz
    OSCTUNEbits.PLLEN = 1;

    // Wait a bit for PLL to stabilize.
    Delay10KTCYx(10);

    ShowBootupAnimation();

    J1939_Initialization( TRUE );


    while (J1939_Flags.WaitingForAddressClaimContention)
        J1939_Poll(5);
    
    /*
     * DBG0 is RB5
     * DBG1 is RB4
     * DBG2 is RB1
     * DBG3 is RB0
     * ERR is RC5
     */
    
    LATBbits.LATB0 = 0;

    // Main Loop
    while(1)
    {
        static int H2_R_OKAY;
        static int H2_L_OKAY;
        static int H2_GLOBAL_OKAY;
        int H2_OVERRIDE = PORTCbits.RC6;
        int H2_LOCAL_CONC = ReadAnalog(0);
        int H2_REMOTE_CONC = ReadAnalog(1);
        int E_STOP = PORTAbits.RA2;

        // If H2_OVERRIDE, turn on an indicator pattern
        if(H2_OVERRIDE)
            SetDebugStatus(1);

        // Compare the remote sensor reading with the defined threshold
        if(H2_REMOTE_CONC < _REMOTE_H2_OKAY_THRES_)
            H2_R_OKAY = 1;
        else
            H2_R_OKAY = 0;
        
        // Compare the local sensor reading with the defined threshold
        if(H2_LOCAL_CONC < _LOCAL_H2_OKAY_THRES_)
            H2_L_OKAY = 1;
        else
            H2_L_OKAY = 0;

        // If both are okay, and e-stop is not pressed, we keep running
        if(H2_L_OKAY == 1 && H2_R_OKAY == 1 && E_STOP == 0)
            H2_GLOBAL_OKAY = 1;
        else
            H2_GLOBAL_OKAY = 0;

        if(!H2_GLOBAL_OKAY && !H2_OVERRIDE)
        {
            // H2 Conc is off the charts man!
            // Shut that thing down!
            // Should shut down literally everything, so we may lose power
            LATCbits.LATC7 = 0;         // H2_OK
            SetErrorState(1);       // ERR LED
            while(PORTCbits.RC7 != 0);  // Make sure the thing is on
            while(1);                   // Wait forever

        } else
        {
            LATCbits.LATC7 = 1; // Drive H2_OK high
        }
        
        //Receive Messages from CANbus
        J1939_Poll(10);
        while (RXQueueCount > 0) {
            J1939_DequeueMessage( &Msg );
            // Currently, only broadcast messages are repeated
//            if( Msg.PDUFormat == PDU_BROADCAST )
//                putSerialData(Msg.GroupExtension, Msg.Data[0], Msg.Data[1]);
            if ( J1939_Flags.ReceivedMessagesDropped )
                J1939_Flags.ReceivedMessagesDropped = 0;
        }
    }
}
