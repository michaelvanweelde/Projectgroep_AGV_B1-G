/*
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "USART.h"
#include <util/delay.h>
#include <math.h>

//Used Macros
#define MotorL_INIT DDRB |= (1<<PB0 | 1<<PB3)               // initialize the L motor on pins(D6)-(D7)
#define MotorL_CW  PORTB  = (PORTB | (1<<PB0)) & ~(1<<PB3)  // Rotate the L motor clockwise
#define MotorL_CCW PORTB  = (PORTB | (1<<PB3)) & ~(1<<PB0)  // Rotate the L motor counter clockwise
#define MotorL_BRK PORTB &= ~(1<<PB0 | 1<<PB3)              // Stop the L motor

#define MotorR_INIT DDRL |= (1<<PL4 | 1<<PL2 )               // initialize the R motor on pins(D8)-(D9)
#define MotorR_CW  PORTL  = (PORTL | (1<<PL4)) & ~(1<<PL2)  // Rotate the R motor clockwise
#define MotorR_CCW PORTL  = (PORTL | (1<<PL2)) & ~(1<<PL4)  // Rotate the R motor counter clockwise
#define MotorR_BRK PORTL &= ~(1<<PL2 | 1<<PL4)              // Stop the R motor

#define IR_FL   (PINC & (1<<PC5))  // Read Front Left    IR sensor on pin(D23)
#define IR_FR1  (PINC & (1<<PC3))  // Read Front Right 1 IR sensor on pin(D24)
#define IR_FR2  (PINC & (1<<PC1))  // Read Front Right 2 IR sensor on pin(D25)
#define IR_BR   (PIND & (1<<PD7))  // Read Back  Right   IR sensor on pin(D26)
#define IR_BL   (PING & (1<<PG1))  // Read Back  Left    IR sensor on pin(D27)

#define Distance_FL IRDistanceRead(0) // Get value from Front Left  IR Distance sensor on pin(A0)
#define Distance_FR IRDistanceRead(1) // Get value from Front Right IR Distance sensor on pin(A2)
#define Distance_B  IRDistanceRead(2) // Get value from Back        IR Distance sensor on pin(A4)

#define TCS3200G_L      (PINL & (1<<PL5))   // Read the Left  TCS3200 green pin on pin(D50)
#define TCS3200G_R      (PINL & (1<<PL3))   // Read the Right TCS3200 green pin on pin(D51)
#define Color_L ColorSensorRead(0)  // Get value from L color sensor
#define Color_R ColorSensorRead(1)  // Get value from R color sensor

#define LED_INIT1           DDRH  |=  (1<<PH5 | 1<<PH6)   // initialize all LEDS
#define LED_INIT2           DDRB  |=  (1<<PB5 | 1<<PB6 | 1<<PB7)   // initialize all LEDS
#define Signal_LED_L_ON     PORTH |=  (1<<PH5)   // Turn on  the Left  Signal LED on pin(D35)
#define Signal_LED_L_OFF    PORTH &= ~(1<<PH5)   // Turn off the Left  Signal LED on pin(D35)
#define Signal_LED_R_ON     PORTH |=  (1<<PH6)   // Turn on  the Right Signal LED on pin(D36)
#define Signal_LED_R_OFF    PORTH &= ~(1<<PH6)   // Turn off the Right Signal LED on pin(D36)
#define Warning_Red         PORTB  = (PORTB | (1<<PB5)) & ~((1<<PB6)|(1<<PB7)) // Turn on only the red   RGB diode on pin(D30)
#define Warning_Green       PORTB  = (PORTB | (1<<PB6)) & ~((1<<PB5)|(1<<PB7)) // Turn on only the green RGB diode on pin(D31)
#define Warning_Blue        PORTB  = (PORTB | (1<<PB7)) & ~((1<<PB5)|(1<<PB6)) // Turn on only the blue  RGB diode on pin(D32)
#define Warning_Off         PORTB &= ~(1<<PB7 | 1<<PB6 | 1<<PB5)                  // Turn off the RGB diodes

#define Emergency_Stop  (PINA & (1<<PA6))  // Read Emergency stop button on pin(D42)
#define Key_switch      (PINA & (1<<PA4))  // Read key switch on pin(D44)
#define Button          (PINA & (1<<PA2))  // Read main control button on pin(D46)

//Settings and dial in variables
#define TCS_Tout 2000 // what we consider an overdue measurement in clock cycles
#define ADC_Tout 2000 // what we consider an overdue measurement in clock cycles

#define WaitTime 400    //how long we will wait after the button is released in ms
#define worktime 400    //how long we wait at our work positions

#define AOP00 100   //Hall value of 0   degrees
#define AOP18 100   //Hall value of 180 degrees
#define AOP90 100   //Hall value of 90  degrees
#define AOP27 100   //Hall value of 270 degrees

#define End_of_Travel   100 //IR value at end of track
#define Start_of_travel 200 //IR value at start of track
#define End_of_Zone     230 //IR value at end or work area
#define Turn_1          140 //IR value of first work area
#define Follow_Deadzone 10  //IR deadzone for following
#define Follow_Distance 80  //IR distance to keep while following

#define TCS_Aval    40 // Average TCS value for green
#define TCS_Max_s   5  // Desired maximum deviation to consider a measurement valid

#define testmode 1  //1 = enable or disable disagnostic mode for sensors
                    //2 = Eable or disable test mode for LEDS and motors

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
    LED_INIT1;
    LED_INIT2;
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

//-initialise variables----
int state;
int Lwork_done;
int Rwork_done;
int cycles;
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

    // button diagnostic printing
    if (Button > 0)         printString("BN ");  else printString("   ");
    if (Key_switch > 0)     printString("KS ");  else printString("   ");
    if (Emergency_Stop > 0) printString("EM ");  else printString("   ");

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
    printByte(ColorSensorRead(0));
    printString(" CL1=");
    printByte(ColorSensorRead(1));
    transmitByte('\n');
    }


    while(testmode == 2){
        for(int x=0; x <= 1000; x++){
        if(x==0)  {Signal_LED_L_ON;  Signal_LED_R_ON;}
        if(x==500){Signal_LED_L_OFF; Signal_LED_R_OFF;}

        if(x==0)  Warning_Red;
        if(x==250)Warning_Green;
        if(x==500)Warning_Blue;
        if(x==750)Warning_Off;

        if(x==0)  {MotorL_CW; MotorR_CW;}
        if(x==333){MotorL_BRK; MotorR_BRK;}
        if(x==666){MotorL_CCW; MotorR_CCW;}
        _delay_ms(2);
        }
    }


    while(1)
    {
    if (state == 0)Warning_Off;

    if(Button >= 1 && state == 0 && Key_switch== 0 ){
    cycles=0;
    Lwork_done=0 ;
    Rwork_done=0 ;
    state=1;
    while(Button >= 1 && Emergency_Stop == 0);
    _delay_ms(WaitTime);                             //Prevent contact bounce and allow for release of contact
    }

    if(Button >= 1 && state == 0 && Key_switch > 1 ){
        cycles=0;
        Lwork_done=0 ;
        Rwork_done=0 ;
        state=100;

    }

    if(Emergency_Stop >= 1){
        state=0;
        Warning_Red;
        while(Emergency_Stop >= 1);
        Warning_Off;
    }

    if(state > 0 && state < 100){
        Warning_Green;
        // center on line------------------------------------------------
        if (state == 1){
            if(IR_FR1 == 0 && IR_FR2 == 0 && IR_BR == 0){ MotorL_CW;  MotorR_BRK; }   // line invisible,                    |turn to line
            if(IR_FR1  > 0 && IR_FR2  > 0 && IR_BR == 0){ MotorL_BRK; MotorR_CW;  }   // machine on line rear end swung     |turn to center
            if(IR_FR1  > 0 && IR_FR2 == 0 && IR_BR == 0){ MotorL_CCW; MotorR_CW;  }   // machine turned sharply over line   |rotate to center
            if(IR_FR1 == 0 && IR_FR2  > 0 && IR_BR == 0){ MotorL_CW;  MotorR_CW;  }   // machine rotated on corner          |rotate to center

            if(IR_FR1 == 0 && IR_FR2 == 0 && IR_BR  > 0){ MotorL_CW;  MotorR_CCW; }   // machine diagonal left on line      |rotate to line
            if(IR_FR1  > 0 && IR_FR2 == 0 && IR_BR  > 0){ MotorL_BRK;  MotorR_CW; }   // machine too far right              |rotate to center
            if(IR_FR1 == 0 && IR_FR2  > 0 && IR_BR  > 0){ MotorL_CCW; MotorR_BRK; }   // machine too far left               |Rotate to center

            if(IR_FR1  > 0 && IR_FR2  > 0 && IR_BR  > 0) state = 2;                   // machine aligned                    |commence farming
        }
        else if (state!= 1 && state < 5 ){
            if(IR_FR1  > 0 && IR_FR2 == 0){ MotorL_BRK; MotorR_CW; _delay_ms(10); }   // machine leaning right    |turn to center   |use delays to filter movement stuttering
            if(IR_FR1 == 0 && IR_FR2  > 0){ MotorL_CW; MotorR_BRK; _delay_ms(10); }   // machine leaning left     |rotate to line   |use delays to filter movement stuttering
        }
        //-----------------------------------------------------------------


        // reverse to back of line-----------------------------------------
        if (state == 2){
            if(Distance_FL <  Start_of_travel || Distance_FR <  Start_of_travel){ MotorL_CCW; MotorR_CCW; }
            if(Distance_FL >= Start_of_travel && Distance_FR >= Start_of_travel)state = 3;
        }
        //-----------------------------------------------------------------


        // drive down line and look for Pots-------------------------------
            if (state == 3)
            {
                MotorL_CW;
                MotorR_CW;

                // if the left color sensor detects a pot halt and send work signal.
                if(ColorSensorRead(0) > (TCS_Aval - TCS_Max_s) && ColorSensorRead(0) < (TCS_Aval + TCS_Max_s) && (Lwork_done > 0))
                {
                    MotorL_BRK;
                    MotorR_BRK;
                    Signal_LED_L_ON;
                    _delay_ms(WaitTime);
                    Signal_LED_L_OFF;
                    Lwork_done=1;
                }
                // if work has been completed and driving continues wait for the pot to be undetected again before checking again.
                if(ColorSensorRead(0) == 0 )Lwork_done=0;
                //-----------------------------------------------------------------


                // if the right color sensor detects a pot halt and send work signal.
                if(ColorSensorRead(1) > (TCS_Aval - TCS_Max_s) && ColorSensorRead(1) < (TCS_Aval + TCS_Max_s) && (Rwork_done > 0))
                {
                    MotorL_BRK;
                    MotorR_BRK;
                    Signal_LED_R_ON;
                    _delay_ms(WaitTime);
                    Signal_LED_R_OFF;
                    Rwork_done=1;
                }
                // if work has been completed and driving continues wait for the pot to be undetected before checking again.
                if(ColorSensorRead(1) == 0 )Rwork_done=0;
                //-----------------------------------------------------------------
                if(Distance_FL <  End_of_Travel && Distance_FR <  End_of_Travel)state=4;
            }



            // Reverse until out of the working zone
            if(state == 4){
                if(Distance_FL <  End_of_Zone || Distance_FR <  End_of_Zone){ MotorL_CCW; MotorR_CCW; }
                if(Distance_FL >= End_of_Zone && Distance_FR >= End_of_Zone)state = 5;
            }
            //-----------------------------------------------------------------


            // rotate 90 degrees CCw
            if(state == 5){
                if(IR_FR1  > 0 && IR_FR2  > 0 && IR_BR == 0){ MotorL_CCW;  MotorR_CCW; }    // machine on line rear end swung     |turn to center
                if(IR_FR1 == 0 && IR_FR2  > 0 && IR_BR == 0){ MotorL_BRK;  MotorR_CCW; }    // machine on line rear end swung     |turn to center
                if(IR_FR1  > 0 && IR_FR2 == 0 && IR_BR == 0){ MotorL_CCW;  MotorR_BRK; }    // line invisible,                    |turn to line
                if(IR_FR1 == 0 && IR_FR2 == 0 && IR_BR == 0){ MotorL_BRK;  MotorR_CW ; }    // line invisible,                    |turn to line
                if(IR_BR   > 0 )state = 6;                                                  //

            }
            //-----------------------------------------------------------------


            // drive to next zone
            if(state == 6)
            {
                MotorL_CCW;
                MotorR_CCW;
                if(IR_FR2  > 0){state = 1; cycles++;}   // machine on line rear end swung     |turn to center
                //if(Distance_FL <  End_of_Zone || Distance_FR <  End_of_Zone){ MotorL_CCW; MotorR_CCW; }
                //if(Distance_FL >= End_of_Zone && Distance_FR >= End_of_Zone)state = 7;
            }
            //-----------------------------------------------------------------
            if (Key_switch > 1)break;
        }



            // if out of work stop
            if(cycles == 2)state=0;
            //--------------------


            if(state => 100){
            Warning_Blue;
            if(IR_BL > 0 || IR_BR > 0 || IR_FL > 0 || IR_FR1 > 0 || IR_FR2 > 0 ){MotorL_BRK;  MotorR_BRK; Warning_Red }
            else if (Distance_FL > Distance_FR + Follow_Deadzone){MotorL_CW;  MotorR_CCW;}
            else if (Distance_FR > Distance_FL + Follow_Deadzone){MotorL_CCW;  MotorR_CW;}
            else if (Distance_FL > Follow_Distance && Distance_FR > Follow_Distance ){MotorL_CW; MotorR_CW;}
            else {MotorL_BRK; MotorR_BRK;}

            (Key_switch == 0 )state=0;
            }



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

    //Calculate the standard deviation based on a desired value of TCS_Aval and reject if more than TCS_max_s
    if(sqrt((width-TCS_Aval*X)^2 /(X-1))>TCS_Max_s && (testmode != 1)){
    return 0;
       }



    //Take the average value of the measurements rounding down--
    width = width / X;
    //---------------------------------------------------------



    return width; // Return value
}
// potato
