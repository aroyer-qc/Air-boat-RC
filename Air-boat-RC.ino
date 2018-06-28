  /***************************************************************
 * Arduino BOAT RC specific function
 ***************************************************************
 * by Alain Royer
 *
 *  Air boat RC controller
 *  
 *  Feature:
 *      PPM RC input        
 *      All standard navigation light : red, green and white
 *      Arming switch 
 *      RED rotating beacon (for fun) with controllable speed
 *          This circuit has 8 LEDs, the LEDs are arranged in a circle (Opposite side ON at the same time)
 *          The LEDs are connected to PWM outputs and their brightness is continuously adjusted on a sine wave pattern
 *          This gives the impression of LED brightness rotating around the circle
 *          Rotation speed is controlled by a PPM channel
 *      Motor reversal with protection
 *          Relay for thrust reversal on 1 PPM channel
 *          Motor is allowed to stop before switching the motor direction
 *          100% speed forward
 *          20% speed reverse
 *  Green and Red LED
 *  Flashing White for stern
 *  Steady white for bow
 * 
 */

#include <PPMReader.h>

//-------------------------------------------------------------------------------------------------
// Define(s)
//-------------------------------------------------------------------------------------------------

#define NUMBER_OF_BEACON_LED   	        4                                                     // The number of LEDs in the circuit 
#define NUMBER_OF_LED_STEP_INTENSITY    10
#define NUMBER_OF_LED_TOTAL_STEP        (NUMBER_OF_BEACON_LED * NUMBER_OF_LED_STEP_INTENSITY)
 
#define PPM_INTERRUPT_PIN               2                                                     // Interrupt on digital pin 2
#define PPM_CHANNEL_COUNT               6                                                     // PPMReader will expect 6 channel
 
#define CHANNEL_HEADING                 1                                                     // Channel of the direction control
#define CHANNEL_THROTTLE                3                                                     // Channel of the throttle
#define CHANNEL_BEACON_SPEED	        5                                                     // Rotation speed of the beacon light
#define CHANNEL_ARMING                  6                                                     // Arming switch channel

#define SERVO_THROTTLE_CHANNEL_PIN      9
#define	MIN_CHANNEL_VALUE               1000
#define MAX_CHANNEL_VALUE               2000
#define	MIDDLE_CHANNEL_VALUE	        (((MAX_CHANNEL_VALUE - MIN_CHANNEL_VALUE) / 2) + MIN_CHANNEL_VALUE)
#define RANGE_CHANNEL_VALUE             (MAX_CHANNEL_VALUE - MIN_CHANNEL_VALUE)
#define THROTTLE_RATE_REVERSE	        20
#define THROTTLE_RATE_FORWARD           100
#define THROTTLE_JITTER                 50
#define THROTTLE_DELAY_REVERSAL         65000                                                 // number of iteration before allowing motor reversal
 
#define RELAY_REVERSAL_PIN              12 
#define NAV_LIGHT_FLASHER_PIN           13 
#define SERVO_RUDDER_CHANNEL_PIN        10
	
#define NAV_PULSE_TIMING_HIGH	        200000UL                                              // Pulse is 200 mS
#define NAV_PULSE_TIMING_LOW            (1333333UL - NAV_PULSE_TIMING_HIGH)                   // Pulse will occurred every 1.3 Sec
 
//-------------------------------------------------------------------------------------------------
// Variable(s) and object(s)
//-------------------------------------------------------------------------------------------------

// LED variables
int       LedPin[NUMBER_OF_BEACON_LED] = {3, 5, 6, 11};                                      // An array of the pin numbers these LEDs are connected to
int       LedIntensity[40]             = {215, 176, 139, 105, 74,  49,  28,  12,  3,   0,   3,   12,  28,  49,  74,  105, 139, 176, 215, 255,
                                          255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255};
uint32_t  LedLoopCounter               = 0;                                                  // Incremented on each iteration of the main loop 
int       LedStep                      = 0;

// Navigation variables
uint32_t  NavTimePulseStamp            = 0;
bool      NavState                     = true;					

// PPM class
PPMReader PPM(PPM_INTERRUPT_PIN, PPM_CHANNEL_COUNT);                                         // Initialize a PPMReader class

// Throttle variables
uint32_t  ThrottleLastValue;
uint16_t  ThrottleLoopCounter           = 0;                                                 // Decremented on each iteration of the main loop 
bool      IsItForwardMotor              = true;
int       ThrottlePercent               = 100;

//-------------------------------------------------------------------------------------------------
// Setup
//-------------------------------------------------------------------------------------------------

void setup()
{
    // Relay control 
    pinMode(RELAY_REVERSAL_PIN, OUTPUT);
    digitalWrite(RELAY_REVERSAL_PIN, LOW);                                       // Forward mode at boot
	
    // Navigation light
    pinMode(NAV_LIGHT_FLASHER_PIN, OUTPUT);
    digitalWrite(NAV_LIGHT_FLASHER_PIN, LOW);                                    // Nay light off at boot
	
    for(int i = 0; i < NUMBER_OF_BEACON_LED; i++)         			 // Set the LED pins to be outputs
    {
        pinMode(LedPin[i], OUTPUT);
    }

    setupPWM16(20000);                                                           // 20 mSec timer reload
    //Serial.begin(9600);

    // Those are already the default value but here for convenience.
    PPM.blankTime            = 2100;
    PPM.minChannelValue      = MIN_CHANNEL_VALUE;                                // The minimum possible channel value. Should be lesser than maxChannelValue.
    PPM.maxChannelValue      = MAX_CHANNEL_VALUE;                                // The maximum possible channel value. Should be greater than minChannelValue.
    PPM.channelValueMaxError = 100;                                              // The maximum error in channel value to either direction for still considering the channel value as valid.
                                                                                 // This leeway is required because your PPM outputter may have tiny errors and also,
                                                                                 // the Arduino board's refresh rate only allows a limited resolution for timing functions.
}

//-------------------------------------------------------------------------------------------------
// Main loop
//-------------------------------------------------------------------------------------------------

void loop()
{
    // ----------------------------------------------------------------------------------------------------------------------------
    // Handle Rudder

    int Rudder;
    Rudder = PPM.latestValidChannelValue(CHANNEL_HEADING, 0);
    Rudder = (Rudder == 0) ? MIDDLE_CHANNEL_VALUE : Rudder;  			      // Neutral position until valid value available
    analogWrite16(SERVO_RUDDER_CHANNEL_PIN, Rudder);
	
    // ----------------------------------------------------------------------------------------------------------------------------
    // Handle Navigation Light
	
    uint32_t NavActualPulseTime = micros() - NavTimePulseStamp;
	
    if((NavState == true) && (NavActualPulseTime > NAV_PULSE_TIMING_HIGH))
    {
        NavState = false;
        digitalWrite(NAV_LIGHT_FLASHER_PIN, HIGH);
        NavTimePulseStamp = micros();
    }
    else if((NavState == false) && ((NavActualPulseTime > NAV_PULSE_TIMING_LOW)))
    {
        NavState = true;
        digitalWrite(NAV_LIGHT_FLASHER_PIN, LOW);
        NavTimePulseStamp = micros();
    }

    // ----------------------------------------------------------------------------------------------------------------------------
    // Handle Forward/Reverse motor relay 

    uint32_t Arming = PPM.latestValidChannelValue(CHANNEL_ARMING, 0);
    
    if(((Arming >  MIDDLE_CHANNEL_VALUE) && (IsItForwardMotor == true)) ||
       ((Arming <= MIDDLE_CHANNEL_VALUE) && (IsItForwardMotor == false)))
    {
        // Override the throttle
        analogWrite16(SERVO_THROTTLE_CHANNEL_PIN, MIN_CHANNEL_VALUE);
        ThrottleLoopCounter = THROTTLE_DELAY_REVERSAL;
        IsItForwardMotor    = (IsItForwardMotor == true) ? false : true;
        //Serial.print("Reversing \r\n");
    }
		
    // ----------------------------------------------------------------------------------------------------------------------------
    // Handle Throttle position
	
    if(ThrottleLoopCounter != 0)
    {
        ThrottleLoopCounter--;
        
        if(ThrottleLoopCounter == 0)                                                  // If we reach zero then reverse the motor
        {
            if(IsItForwardMotor == true)
            {
                digitalWrite(RELAY_REVERSAL_PIN, LOW);
                ThrottlePercent = 100;
            }
            else
            {
                digitalWrite(RELAY_REVERSAL_PIN, HIGH);
                ThrottlePercent = 20;
            }
        }
    }
    else
    {
        uint32_t Throttle = PPM.latestValidChannelValue(CHANNEL_THROTTLE, 0);
        Throttle = (Throttle == 0) ? MIN_CHANNEL_VALUE : Throttle;                    // Neutral position until a valid value available (prevent arming of ESC)
        Throttle -= MIN_CHANNEL_VALUE;
        Throttle *= ThrottlePercent;                                                  // Apply percent value
        Throttle /= 100;                                                              // Divide by 100%
        Throttle += MIN_CHANNEL_VALUE;
        analogWrite16(SERVO_THROTTLE_CHANNEL_PIN, Throttle);
        //Serial.print(String(Throttle) + "\r\n");
    }

    // ----------------------------------------------------------------------------------------------------------------------------
    // Handle LED beacon

    uint32_t BeaconSpeed  = (RANGE_CHANNEL_VALUE - (PPM.latestValidChannelValue(CHANNEL_BEACON_SPEED, 0) - MIN_CHANNEL_VALUE));

    if(LedLoopCounter >= BeaconSpeed)                                                 // If the number of main loop iterations is high enough then change the LEDs
    {
        int OffsetLed = 0;

        for(int i = 0; i < NUMBER_OF_BEACON_LED; i++)
        {
            analogWrite(LedPin[i], LedIntensity[(LedStep + OffsetLed) % NUMBER_OF_LED_TOTAL_STEP]);
            OffsetLed += 10;
        }
        LedLoopCounter = 0;                                                           // Reset the main loop counter
        LedStep++;
    
        if(LedStep >= 40)
        {
            LedStep = 0;
        }
    }
    else
    {                                                                 	              // Otherwise increment the counter
        LedLoopCounter++;
    }

    // ----------------------------------------------------------------------------------------------------------------------------
    // Print latest valid values from all channels
    //for(int Channel = 1; Channel <= PPM_CHANNEL_COUNT; Channel++)
    //{
    //    uint32_t Value = PPM.latestValidChannelValue(Channel, 0);
    //    Serial.print(String(Value) + " ");
    //}
    //Serial.println();
}

//-------------------------------------------------------------------------------------------------
// Function(s)
//-------------------------------------------------------------------------------------------------

void setupPWM16(int icr)
{
    DDRB  |= _BV(PB1) | _BV(PB2);                       // set pins as outputs
    TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);    // non-inverting PWM, mode 14: fast PWM, TOP=ICR1
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);       // prescaler Clock / 8  (1 MHz)
    ICR1 = icr;                                         // TOP counter value
}


// 16-bit version of analogWrite(). Works only on pins 9 and 10.
void analogWrite16(uint8_t pin, uint16_t val)
{
    switch(pin)
    {
        case  9: OCR1A = val; break;
        case 10: OCR1B = val; break;
    }
}
