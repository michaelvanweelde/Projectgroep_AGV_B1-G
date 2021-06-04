/*
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "USART.h"
#include <util/delay.h>

//Used Macros
#define MotorL_INIT DDRH |= (1<<PH3 | 1<<PH4)               // initialize the L motor on pins(D6)-(D7)
#define MotorL_CW  PORTH  = (PORTH | (1<<PH3)) & ~(1<<PH4)  // Rotate the L motor clockwise
#define MotorL_CCW PORTH  = (PORTH | (1<<PH4)) & ~(1<<PH3)  // Rotate the L motor counter clockwise
#define MotorL_BRK PORTH &= ~(1<<PH3 | 1<<PH4)              // Stop the L motor

#define MotorR_INIT DDRH |= (1<<PH5 | 1<<PH6)               // initialize the R motor on pins(D8)-(D9)
#define MotorR_CW  PORTH  = (PORTH | (1<<PH5)) & ~(1<<PH6)  // Rotate the R motor clockwise
#define MotorR_CCW PORTH  = (PORTH | (1<<PH6)) & ~(1<<PH5)  // Rotate the R motor counter clockwise
#define MotorR_BRK PORTH &= ~(1<<PH5 | 1<<PH6)              // Stop the R motor

#define IR_FL   (PINL & (1<<PL6))  // Read Front Left    IR sensor on pin(D23)
#define IR_FR1  (PINL & (1<<PL6))  // Read Front Right 1 IR sensor on pin(D24)
#define IR_FR2  (PINL & (1<<PL6))  // Read Front Right 2 IR sensor on pin(D25)
#define IR_BR   (PINL & (1<<PL6))  // Read Back  Right   IR sensor on pin(D26)
#define IR_BL   (PINL & (1<<PL6))  // Read Back  Left    IR sensor on pin(D27)

#define Distance_FL IRDistanceRead(0) // Get value from Front Left  IR Distance sensor on pin(A0)
#define Distance_FR IRDistanceRead(1) // Get value from Front Right IR Distance sensor on pin(A2)
#define Distance_B  IRDistanceRead(2) // Get value from Back        IR Distance sensor on pin(A4)

#define TCS3200_INIT    DDRB  |=  (1<<PB0 | 1<<PB1) // Initialize the TCS3200 Color sensor LEDS on pins(D52)-(D53)
#define TCS3200_LED_ON  PORTB |=  (1<<PB0 | 1<<PB1) // Turn on  the LEDs on pins(D52)-(D53)
#define TCS3200_LED_OFF PORTB &= ~(1<<PB0 | 1<<PB1) // Turn off the LEDs on pins(D52)-(D53)
#define TCS3200G_L      (PINB & (1<<PB3))   // Read the Left  TCS3200 green pin on pin(D50)
#define TCS3200G_R      (PINB & (1<<PB4))   // Read the Right TCS3200 green pin on pin(D51)
#define Color_L ColorSensorRead(0)  // Get value from L color sensor
#define Color_R ColorSensorRead(1)  // Get value from R color sensor

#define LED_INIT            DDRC  |=  (1<<PC1 | 1<<PC2 | 1<<PC5 | 1<<PC6 | 1<<PC7)   // initialize all LEDS
#define Signal_LED_L_ON     PORTC |=  (1<<PC2)   // Turn on  the Left  Signal LED on pin(D35)
#define Signal_LED_L_OFF    PORTC &= ~(1<<PC2)   // Turn off the Left  Signal LED on pin(D35)
#define Signal_LED_R_ON     PORTC |=  (1<<PC1)   // Turn on  the Right Signal LED on pin(D36)
#define Signal_LED_R_OFF    PORTC &= ~(1<<PC1)   // Turn off the Right Signal LED on pin(D36)
#define Warning_RED         PORTC  = (PORTC | (1<<PC7)) & ~((1<<PC6)|(1<<PC5)) // Turn on only the red   RGB diode on pin(D30)
#define Warning_Green       PORTC  = (PORTC | (1<<PC6)) & ~((1<<PC5)|(1<<PC7)) // Turn on only the green RGB diode on pin(D31)
#define Warning_Blue        PORTC  = (PORTC | (1<<PC5)) & ~((1<<PC7)|(1<<PC6)) // Turn on only the blue  RGB diode on pin(D32)
#define Warning_Off         PORTC &= ~(1<<PC7 | 1<<PC6 | PC5)                  // Turn off the RGB diodes

#define Emergency_Stop  (PINL & (1<<PL7))  // Read Emergency stop button on pin(D42)
#define Key_switch      (PINL & (1<<PL5))  // Read key switch on pin(D44)
#define Button          (PINL & (1<<PL3))  // Read main control button on pin(D46)

//Settings and dial in variables
#define TCS_Tout 2000 // what we consider an overdue measurement in clock cycles
#define ADC_Tout 2000 // what we consider an overdue measurement in clock cycles

#define AOP00 100   //Hall value of 0   degrees
#define AOP18 100   //Hall value of 180 degrees
#define AOP90 100   //Hall value of 90  degrees
#define AOP27 100   //Hall value of 270 degrees

#define testmode 1  //enable or disable disagnostic mode


int north_angle = 0;
int MagnometerRead();
int IRDistanceRead(int sensor);
int ColorSensorRead(int sensor);


int main(void)
{

//-Initialize ADCs---------
    ADMUX |= (0 << REFS1) |(1 << REFS0)|(1<<ADLAR);     // Voltage reference via capacitor on arduino
    ADCSRA |= (1 << ADEN) |(0 << ADIE);                 // ADC enable and ADC conversion complete interrupt disabled
    ADCSRA |= (1 << ADPS2)|(1 << ADPS1)|(1 << ADPS0);   // Prescaler settings (Currently 128)
    DIDR0 = 0b11111111;                                 // Disable Digital input
    DIDR2 = 0b11111111;                                 // Disable Digital input
    ADCSRA |= (1 << ADSC);                              // Run a single conversion in order to prime the circuit
//-------------------------

//-Initialize pins---------
    TCS3200_INIT;
    LED_INIT;
//-------------------------


//-Initialize motors-------
    MotorL_INIT;
    MotorR_INIT;
//-------------------------


//-Initialize Serial-------
    initUSART();
//-------------------------


//-zero magnometer---------
    north_angle = MagnometerRead();
//-------------------------


//-Enable interrupts-------
    sei();
//-------------------------
//-End of setup-----------------------------------------]
    printString("RDY");
    transmitByte('\n');


    while(testmode == 1){
    transmitByte('\n');
    // IR sensor diagnostic printing
    if (IR_FL > 0) printString("FL  "); else printString("    ");
    if (IR_FR1 > 0)printString("FR1 "); else printString("    ");
    if (IR_FR2 > 0)printString("FR2 "); else printString("    ");
    if (IR_BL > 0) printString("BL  "); else printString("    ");
    if (IR_BR > 0) printString("BR  "); else printString("    ");

    // Magnometer Diagnostic printing
    printString(" MAG=");
    printByte(MagnometerRead());

    // Distance sensor Diagnostic printing
    printString(" IR0=");
    printByte(IRDistanceRead(0));
    printString(" IR1=");
    printByte(IRDistanceRead(1));
    printString(" IR2=");
    printByte(IRDistanceRead(2));

    //colorsensor diagnostic printing
    printString(" CL0=");
    //printByte(ColorSensorRead(0));
    printString(" CL1=");
    //printByte(ColorSensorRead(1));
    transmitByte('\n');
    }

    while(1)
    {






    }
    return 0;
}




int MagnometerRead()
{
    // Initialize variables------------------------------------
    int hal1;
    int hal2;
    int angle;
    volatile int timeout=0;
    //---------------------------------------------------------


    // Switch the ADC to the correct pin and read data---------
    ADMUX  = (ADMUX | (1<<MUX1)|(1<<MUX2)) & ~(1<<MUX0);                // PIN selection (A6)
    ADCSRA |= (1 << ADSC);                                              // Start the conversion
    while ( (ADCSRA & (1 << ADSC)) == 1 && timeout++<(ADC_Tout/2) );    // Wait until conversion is finished
    hal1 = (ADCH);                                                      // ADC data is left aligned and can be read from ADCH directly as an 8 bit value
                                                                        // Because ADC value is an 8 bit value overflow is impossible

    ADMUX  = (ADMUX | (1<<MUX0)|(1<<MUX1)|(1<<MUX2));                   // PIN selection (A7)
    ADCSRA |= (1 << ADSC);                                              // Start the conversion
    while ( (ADCSRA & (1 << ADSC)) == 1 && timeout++<(ADC_Tout/2) );    // Wait until conversion is finished
    hal2 = (ADCH);                                                      // ADC data is left aligned and can be read from ADCH directly as an 8 bit value
    //---------------------------------------------------------         // Because ADC value is an 8 bit value overflow is impossible


    //-compare the data-----------
    if( (hal1 > hal2) && (hal1 >= AOP00) )angle = 0;
    if( (hal1 > hal2) && (hal1 <  AOP18) )angle = 180;
    if( (hal1 < hal2) && (hal2 >= AOP90) )angle = 90;
    if( (hal1 < hal2) && (hal2 <  AOP27) )angle = 270;
    //----------------------------

    return angle;
}



int IRDistanceRead(int sensor)
{
    // Initialize variables------------------------------------
    int distance=0;
    volatile int timeout=0;
    int X;
    //---------------------------------------------------------

    ADMUX  = ((ADMUX) & 0b11100000);
    // Switch the ADC to the correct pin-----------------------
    switch (sensor)
    {
    case 0:                                                     // Set pin A0 as read pin
        ADMUX  = (ADMUX | (0b00000000));    // PIN selection
        break;

    case 1:                                                     // Set pin A2 as read pin
        ADMUX  = (ADMUX | (0b00000010));  // PIN selection
        break;

    case 2:                                                     // Set pin A4 as read pin
        ADMUX  = (ADMUX | (0b00000100));  // PIN selection
        break;
    }
    //---------------------------------------------------------

    // Take measurements with the ADC and collect the average--
    for(X=0; X<5; X++)
    {
        ADCSRA |= (1 << ADSC);                                              // Start the conversion
        while ( (ADCSRA & (1 << ADSC)) == 1 && timeout++<(ADC_Tout/2) );    // Wait until conversion is finished
        distance += (ADCH);                                                 // ADC data is left aligned and can be read from ADCH directly as an 8 bit value
    }                                                                       // Because ADC value is an 8 bit value overflow is impossible
    //---------------------------------------------------------


    // If a timeout occurs attempt to retry the measurement----
    if( timeout == ADC_Tout/2 )
    {
        printString("warning ADC out of boundaries retrying");
        transmitByte('\n');
        distance=0;
        for(X=0; X<5; X++)
        {
            ADCSRA |= (1 << ADSC);                                          // Start the conversion
            while ( (ADCSRA & (1 << ADSC)) == 1 && timeout++<(ADC_Tout) );  // Wait until conversion is finished
            distance += (ADCH);                                             // ADC data is left aligned and can be read from ADCH directly as an 8 bit value
        }
        if( timeout == ADC_Tout)                                            // if retry has failed return 0
        {
            printString("Retry failed");
            transmitByte('\n');
            return 0;
        }
    }
    //---------------------------------------------------------


    //Take the average value of the measurements rounding down-
    distance = distance / X;
    //---------------------------------------------------------

    return distance;
}




int ColorSensorRead(int sensor)
{
    // Initialize variables------------------------------------
    volatile int width=0;
    int X;
    //---------------------------------------------------------


    for(X=0; X<10; X++)
    {
        //Create a timeout variable for error detection--------
        volatile int timeout=0;
        //-----------------------------------------------------


        // Wait until pin turns high then measure pulse width--
        while( (TCS3200G_L) && (sensor == 0) && (timeout++<TCS_Tout) );         // Wait for pin to turn low to prevent possible error
        while(!(TCS3200G_L) && (sensor == 0) && (timeout++<TCS_Tout) );         // wait out the Low period
        while( (TCS3200G_L) && (sensor == 0) && (timeout++<TCS_Tout) )width++;  // Measure the pulse width

        while( (TCS3200G_R) && (sensor == 1) && (timeout++<TCS_Tout) );         // Wait for pin to turn low to prevent possible error
        while(!(TCS3200G_R) && (sensor == 1) && (timeout++<TCS_Tout) );         // wait out the Low period
        while( (TCS3200G_R) && (sensor == 1) && (timeout++<TCS_Tout) )width++;  // Measure the pulse width
        //-----------------------------------------------------


        if( timeout == TCS_Tout )   // Catch a timeout scenario and end the function early returning a 0 to the program
        {
            // Display error code with data and return 0 for error-
            printString("warning TCS ");
            printByte(sensor);
            printString(" out of boundaries after: ");
            printByte(X);
            printString(" runs");
            transmitByte('\n');
            return 0;
        }
        //-----------------------------------------------------
    }

    //Take the average value of the measurements rounding down--
    width = width / X;
    //---------------------------------------------------------


    return width; // Return value
}
// potato
