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

sei();



    while(1){




    }
    return 0;
}
