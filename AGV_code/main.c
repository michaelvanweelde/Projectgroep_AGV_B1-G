/*
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "USART.h"



//Defines with standard setups
/*-------------------------------- example
#define motorX_INIT DDRA |= (1<<PA0 | 1<<PA1)               // initialize the X dir motor on pins(D22)-(D23)
#define motorX_CW  PORTA  = (PORTA | (1<<PA0)) & ~(1<<PA1)  // Rotate the X dir motor clockwise
#define motorX_CCW PORTA  = (PORTA | (1<<PA1)) & ~(1<<PA0)  // Rotate the X dir motor counter clockwise
#define motorX_BRK PORTA &= ~(1<<PA0 | 1<<PA1)              // Stop the X dir motor
----------------------------------*/


int north_angle = 0;
int UltrasoonRead(int sensor);
int MagnometerRead();
int IRDistanceRead(int sensor);


int main(void)
{

//-Initialise ADCs---------
    ADMUX |= (0 << REFS1) |(1 << REFS0)|(1<<ADLAR);     //Voltage reference via capacitor on arduino
    ADCSRA |= (1 << ADEN) |(0 << ADIE);                 //ADC enable and ADC conversion complete interrupt disabled
    ADCSRA |= (1 << ADPS2)|(1 << ADPS1)|(1 << ADPS0);   //Prescaler settings (Currently 128)
    DIDR0 = 0b11111111;                                 //Disable Digital input
    DIDR2 = 0b11111111;                                 //Disable Digital input
    ADCSRA |= (1 << ADSC);                              //Run a single conversion in order to prime the circuit
//-------------------------

//-Initialise Sensors------
    /*
    Cny70's
    setup pins

    ultrasoon
    setup pins

    TCS32000's
    setup pins
    set prescaler
    */
//-------------------------


//-Initialise motors-------
    /*
    setup pins
    setup PWM signals
    */
//-------------------------


//-Initialise I2C interface
    /*
    Setup I2C interface with TWI builtin
    */
//-------------------------


//-Initialise Serial-------
    initUSART();
    printString("RDY");
//-------------------------


//-Setup timer for delays--
    /*
    Setup timer with a +- 1ms overflow time
    */
//-------------------------


//-zero magnometer---------
    north_angle = MagnometerRead();
//-------------------------

//-Enable sei--------------
    sei();
//-------------------------

//-End of setup-----------------------------------------]
    while(1)
    {
        printByte( IRDistanceRead(0) );
        transmitByte(' ');
        printByte( IRDistanceRead(1) );
        transmitByte(' ');
        printByte( IRDistanceRead(2) );
        transmitByte('\n');


    }
    return 0;
}



int UltrasoonRead(int sensor)
{

    //-initialize variables-------
    volatile float echo=0.0;
    int timeout=0;
    //----------------------------


    //-Pulse the Trig pin---------
    //----------------------------


    //-Wait until echo turns high-
    //----------------------------


    //-then measure pulse width---
    //----------------------------


    return echo;
}



int MagnometerRead()
{
    int Xvalue=0;
    int Yvalue=0;
    int Zvalue=0;

    //-Read X value and write to mem
    //----------------------------

    //-Read Y value and write to mem
    //----------------------------

    //-Read Z value and write to mem
    //----------------------------

    //-compare the data-----------
    /*
    with trigonometry make sure that
    the center point of all 3 sensors
    is not close to the vehicle
    this way magnets are ignored
    */
    //----------------------------

    return Xvalue;
}



int IRDistanceRead(int sensor)
{
    int Distance=0;

    switch (sensor)
    {
    case 0: // Set pin 0 as read pin
        ADMUX  = (ADMUX) & ~((1<<MUX0)|(1<<MUX1)|(1<<MUX2));    //PIN selection
        break;

    case 1:// Set pin 2 as read pin
        ADMUX  = (ADMUX | (1<<MUX1)) & ~((1<<MUX0)|(1<<MUX2));  //PIN selection
        break;

    case 2: // Set pin 4 as read pin
         ADMUX  = (ADMUX | (1<<MUX2)) & ~((1<<MUX0)|(1<<MUX1)); //PIN selection
        break;
    }



    ADCSRA |= (1 << ADSC);
    while ((ADCSRA & (1 << ADSC)) == 1);

// ADC data is left aligned and can be read from ADCH as an 8 bit value
    Distance = (ADCH);

    return Distance;
}
