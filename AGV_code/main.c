/*
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "USART.h"
#include "I2C_Master_H_file.h"	/* Include I2C header file */

#define clock_sla       0x11
#define clock_seconds   0x0F
#define Mag_adress      0x1E
#define Mag_read        0x3C
#define Mag_write       0x3D
#define Mag_x           0x03


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


#define TCS_Tout 2000 //what we consider an overdue measurement in clock cycles


int north_angle = 0;
int MagnometerRead();
int IRDistanceRead(int sensor);
int ColorSensorRead(int sensor);


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

//-------------------------


//-Initialise Serial-------
    initUSART();
//-------------------------


//-Setup timer for delays--
    /*
    Setup timer with a +- 1ms overflow time
    */
//-------------------------


//-zero magnometer---------
    I2C_Init();
    I2C_Start(0x3C);	/* Start and write SLA+W */
    I2C_Write(0x00);	/* Write memory location address */
    I2C_Write(0x70);	/* Configure register A as 8-average, 15 Hz default, normal measurement */
    I2C_Write(0xA0);	/* Configure register B for gain */
    I2C_Write(0x00);	/* Configure continuous measurement mode in mode register */
    I2C_Stop();		    /* Stop I2C */
    north_angle = MagnometerRead();
//-------------------------

//-Enable sei--------------
    sei();
//-------------------------
//-End of setup-----------------------------------------]
    printString("RDY");
    printByte('\n');

    while(1)
    {

        //uint8_t x, y, z;
        //I2C_Start(0x3C);	/* Start and wait for acknowledgment */
        //I2C_Write(0x03);	/* Write memory location address */
        //I2C_Repeated_Start(0x3D);/* Generate repeat start condition with SLA+R */
        //x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
        //z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
        //y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Nack());
        //I2C_Stop();		/* Stop I2C */




    }
    return 0;
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



int ColorSensorRead(int sensor)
{
    // initialize variables------------------------------------
    volatile int width=0;
    int X;
    //---------------------------------------------------------


    for(X=1; X<11; X++)
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
            printByte('\n');
            return 0;
        }
        //-----------------------------------------------------
    }

    //Take the average value of the measurements rounding down--
    width = width / X;
    //---------------------------------------------------------


    return width; // Return value
}
