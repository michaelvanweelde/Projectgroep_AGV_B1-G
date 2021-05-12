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
    ADMUX |= (0 << REFS1)|(1 << REFS0);                 //Voltage reference via capacitor on arduino
    ADCSRA |= (1 << ADEN) | (0 << ADIE);                //ADC enable and ADC conversion complete interrupt disabled
    ADCSRA |= (1 << ADPS2)|(1 << ADPS1)|(1 << ADPS0);   //Prescaler settings (Currently 128)
    DIDR0 = 0b11111111;                                 //Disable Digital input
    DIDR2 = 0b11111111;                                 //Disable Digital input
  //ADCSRA |= (1<<ADSC);                                //Enable Automatic conversion Via ADC_VECT interrupt
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



int MagnometerRead(){
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



int IRDistanceRead(int sensor){
    int Distance=0;
// switch case with available pins
/*
    ADMUX |= (0 << MUX0)|(0 << MUX1)|(0 << MUX2)|(0 << MUX3)|(0 << MUX4);   //PIN selection
    ADCSRB |= (0 << MUX5);                                                  //PIN selection
*/


ADCSRA |= (1 << ADSC);
while ((ADCSRA & (1 << ADSC)) == 1);

// ADC data is right aligned by default and must be read from ADLC first
Distance = ADCL;        //Read the first data register
Distance += (ADCH<<8);  //Read the second data register and shift it by 8

return Distance;
}
