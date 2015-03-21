/* 
 * File:   main.c
 * Author: Matt Amyotte
 *
 * This code is desinged to control the voltage signal running into the motor
 * controller pedal input.  It increases the input voltage from the pedal, to
 * creat a proper input range for the controller.  Additionally, it handles
 * the cruise control function, which sets the voltage to be constant.  Cruise
 * control is enabled through a button press interrupt, and turned off by the
 * same button, or the application of the brakes.
 *
 * Created on March 21, 2013, 8:19 PM
 */

#include <p18cxxx.h>
#include <delays.h>
#include <pwm.h>
#include <adc.h>
#include <timers.h>
#include "..\Common\J1939.h"
#include "..\Common\ecocar.h"

#pragma config OSC = IRCIO67    // Oscillator Selection Bit
#pragma config BOREN = OFF      // Brown-out Reset disabled in hardware and software
#pragma config WDT = OFF        // Watchdog Timer disabled (control is placed on the SWDTEN bit)

#define DATA_LEN 8
#define DATA_ITEM_LENGTH 3

// Minimum time between CANbus messages
#define CANBUS_SEND_INTERVAL 20

// Time to poll CANbus for messages
#define CANBUS_POLL_TIME 10

// Time between overamperage step-offs in ms
#define BACKOFF_STEP_TIME 100

// Number of millivolts to backoff in each step
#define BACKOFF_MV 100

// The value to give the pedal when we want the motors off
#define PEDAL_OFF 0
#define PEDAL_MAX 1023

// The values to increment or decrement the cruise control by
#define CRUISE_INC_INTERVAL 20
#define CRUISE_DEC_INTERVAL 20

///// WARNING: We rely on this condition: FC_WARN_CURRENT < FC_EMRG_CURRENT
#define FC_WARN_CURRENT 50000
#define FC_EMRG_CURRENT 60000

#define DDR_BRAKE   TRISCbits.TRISC0
#define INPUT_BRAKE PORTCbits.RC0

#define DDR_CRUISE   TRISAbits.TRISA7
#define INPUT_CRUISE PORTAbits.RA7

#define DDR_ERR_LED    TRISCbits.TRISC3
#define OUTPUT_ERR_LED LATCbits.LATC3

unsigned long int Pedal  = PEDAL_OFF;
unsigned long int MPedal = PEDAL_OFF;
bool              Set    = false;

volatile bool can_backoff       = false;
volatile bool need_to_send_data = false;
volatile bool backing_off       = false;

typedef enum backoff {
    NOT_BACKING_OFF = 0,
    BACKING_OFF = 1,
    EMERG_BACKOFF = 2,
} backoff_t;

typedef enum cruise {
    CRUISE_OFF = 0,
    CRUISE_ON = 1
} cruise_t;

void set_cruise(cruise_t new_value);
void process_CAN_message(J1939_MESSAGE Msg);
void increase_cruise_speed();
void decrease_cruise_speed();

cruise_t Cruise = CRUISE_OFF;
backoff_t BackOffStatus = NOT_BACKING_OFF;
J1939_MESSAGE Msg;

char sensorLatestData[DATA_LEN][DATA_ITEM_LENGTH] = {
    {DATA_MOTOR_PERC, 0x00, 0x00}
};

char sensorLastSentData[DATA_LEN][DATA_ITEM_LENGTH] = {
    {DATA_MOTOR_PERC, 0x00, 0x00}
};

void main(void) {
    InitEcoCar();
    J1939_Initialization( TRUE );

    while (J1939_Flags.WaitingForAddressClaimContention)
        J1939_Poll( CANBUS_POLL_TIME );

//    ADCON0 = 0x01;                  // Initialize the ADC Registers
//    ADCON1 = 0x0E;
//    ADCON2 = 0x8E;

//    TRISA = 0b00000001;             // Set A0 to input, others to output
//    TRISB = 0b00001011;             // Set B2 (CANTX) to output, B3 (CANRX) to input
//    TRISC = 0b00000000;
//    LATCbits.LATC3 = 0;

    // Configure output ports
    DDR_ERR_LED = PORT_DIRECTION_OUTPUT;
    OUTPUT_ERR_LED = PORT_OUTPUT_LOW;

    // Configure input ports
    DDR_BRAKE   = PORT_DIRECTION_INPUT;
    DDR_CRUISE  = PORT_DIRECTION_INPUT;

    // Setup Timer0
    INTCONbits.TMR0IE = TIMER_ENABLED;      // Enable the timer
    INTCONbits.TMR0IF = INTERRUPT_FLAG_CLR; // Clear the flag
    T0CON = 0b11000111;
    
    OpenTimer2( T1_SOURCE_CCP );
    OpenPWM1( 0xFF );        //Turn on PWM capabilities

    // Enable interrupts and disable priorities
    RCONbits.IPEN  = DISABLED;
    INTCONbits.GIE = ENABLED;

    forever {
        // Poll the CANbus briefly
        J1939_Poll( CANBUS_POLL_TIME );
        
        // Handle messages
        while ( RXQueueCount > 0 ) {
            // Get the message from the queue
            J1939_DequeueMessage( &Msg );

            // Process the message contents
            process_CAN_message();

            // Clear the message dropped flag if set
            if ( J1939_Flags.ReceivedMessagesDropped ) {
                J1939_Flags.ReceivedMessagesDropped = 0;
            }
        }

       // The brake can turn off the cruise
        if ( PORT_INPUT_HIGH == INPUT_BRAKE ) {
            MPedal = PEDAL_OFF;
            Cruise = CRUISE_OFF;
        } else if ( NOT_BACKING_OFF == BackOffStatus ) {
            // The brake is not engaged and we aren't backing off. Read cruise status
            if ( PORT_INPUT_HIGH == INPUT_CRUISE && true == Set ) {
                Set = false;
                if ( Cruise == CRUISE_OFF ) {
                    set_cruise( CRUISE_ON );
                } else if ( Cruise == CRUISE_ON ) {
                    set_cruise( CRUISE_OFF );
                }
            } else if ( PORT_INPUT_LOW == INPUT_CRUISE ) {
                Set = true;
            }
        }

        // Only allow the driver to control speed if we aren't backing off
        if ( NOT_BACKING_OFF == BackOffStatus  && CRUISE_OFF == Cruise ) {
            MPedal = ReadAnalog(4) * 0.20;
        } else if ( BACKING_OFF == BackOffStatus && can_backoff ) {
            can_backoff = false;
            
            if ( MPedal >= BACKOFF_MV ) {
                MPedal -= BACKOFF_MV;
            } else {
                MPedal = PEDAL_OFF;
            }
        } else if ( EMERG_BACKOFF == BackOffStatus ) {
            MPedal = PEDAL_OFF;
        }

        SetDCPWM1( MPedal );
    }
}

inline void process_CAN_message(J1939_MESSAGE Msg) {
    // Five types of messages are accepted:
    // - Cruise Set
    // - Cruise Clear
    // - Cruise Decrement
    // - Cruise Increment
    // - Get Motor Percent
    if (Msg.PDUFormat == PDU_CONTROL) {
        switch (Msg.GroupExtension) {
            case CTRL_CRUISE_SET:
                set_cruise( CRUISE_ON );
                break;
            case CTRL_CRUISE_CLR:
                set_cruise( CRUISE_OFF );
                break;
            case CTRL_CRUISE_INC:
                increase_cruise_speed();
                break;
            case CTRL_CRUISE_DEC:
                decrease_cruise_speed();
                break;
        }
    } else if (Msg.PDUFormat == PDU_BROADCAST) {
        // Check the FC current status
        if ( DATA_FC_CURR == Msg.GroupExtension ) {
            unsigned long int fc_curr = (Msg.Data[1] << 8) | Msg.Data[0];
            if ( fc_curr >= FC_WARN_CURRENT ) {
                // Only engage cruise if we switch from not backing off to backing off
                if ( NOT_BACKING_OFF == BackOffStatus ) {
                    Cruise = CRUISE_ON;
                }

                if ( fc_curr >= FC_EMRG_CURRENT ) {
                    // We're above the emergency current threshold
                    BackOffStatus = EMERG_BACKOFF;
                } else {
                    // We're above the warning current threshold
                    BackOffStatus = BACKING_OFF;
                }
            } else {
                // We're at a safe current level
                BackOffStatus = NOT_BACKING_OFF;
            }
        }
    }
}

inline void increase_cruise_speed() {
    // Can only modify the cruise speed if not in a backoff state
    if (NOT_BACKING_OFF != BackOffStatus) {
        return;
    }

    // Increase the speed by the interval
    MPedal += CRUISE_INC_INTERVAL;
    if (MPedal > PEDAL_MAX) {
        MPedal = PEDAL_MAX;
    }
}

inline void decrease_cruise_speed() {
    // Can only modify the cruise speed if not in a backoff state
    if (NOT_BACKING_OFF != BackOffStatus) {
        return;
    }

    // Increase the speed by the interval
    if (MPedal >= CRUISE_DEC_INTERVAL) {
        MPedal -= CRUISE_DEC_INTERVAL;
    } else {
        MPedal = PEDAL_OFF;
    }
}

inline void set_cruise( cruise_t new_value ) {
    Cruise = new_value == CRUISE_ON;
}

#pragma interrupt isr
void isr(void) {
    static int last_canbus_send = 0;
    static unsigned long long int time = 0;
    static unsigned long long int last_backoff_time = 0;

    if (INTERRUPT_FLAG_SET == INTCONbits.TMR0IF) {
        ++time;
        ++last_canbus_send;
        ++last_backoff_time;

        // Every timer overflow mark that we should send data
        if ( last_canbus_send > CANBUS_SEND_INTERVAL ) {
            need_to_send_data = 1;
            last_canbus_send = time;
        }

        if ( last_backoff_time > BACKOFF_STEP_TIME ) {
            can_backoff = true;
            last_backoff_time = time;
        }

        INTCONbits.TMR0IF = INTERRUPT_FLAG_CLR;
    }
}

#pragma code high_vector = 0x08
void high_interrupt(void) {
    _asm GOTO isr _endasm
}
#pragma code

void Process_and_Send_Data(J1939_MESSAGE *MsgPtr, int i) {
    char data[8];
    data[1] = sensorLatestData[i][1]; // MSB
    data[0] = sensorLatestData[i][2]; // LSB
    Broadcast_Data(MsgPtr, sensorLatestData[i][0], data);
}
