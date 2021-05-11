/*
 */

#include <avr/io.h>
#include <avr/interrupt.h>

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

int main(void)
{

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
    /*
    setup serial data line
    send serial RDY over line
    */
//-------------------------


//-Setup timer for delays--
    /*
    Setup timer with a +- 1ms overflow time
    */
//-------------------------


//-zero magnometer---------
north_angle = MagnometerRead();
//-------------------------


    sei();

//-End of setup-----------------------------------------]
    while(1)
    {




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
int Xvalue;
int Yvalue;
int Zvalue;

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
