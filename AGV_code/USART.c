
/*
  Quick and dirty functions that make serial communications work.

  Note that receiveByte() blocks -- it sits and waits _forever_ for
   a byte to come in.  If you're doing anything that's more interesting,
   you'll want to implement this with interrupts.

   initUSART requires BAUDRATE to be defined in order to calculate
     the bit-rate multiplier.  9600 is a reasonable default.

  May not work with some of the older chips:
    Tiny2313, Mega8, Mega16, Mega32 have different pin macros
    If you're using these chips, see (e.g.) iom8.h for how it's done.
    These old chips don't specify UDR0 vs UDR1.
    Correspondingly, the macros will just be defined as UDR.
*/

#include <avr/io.h>
#include "USART.h"
#include <util/setbaud.h>

void initUSART(void) {                                /* requires BAUD */
  UBRR0H = UBRRH_VALUE;                        /* defined in setbaud.h */
  UBRR0L = UBRRL_VALUE;
#if USE_2X
  UCSR0A |= (1 << U2X0);
#else
  UCSR0A &= ~(1 << U2X0);
#endif
                                  /* Enable USART transmitter/receiver */
  UCSR0B = (1 << TXEN0) | (1 << RXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);   /* 8 data bits, 1 stop bit */
}


void transmitByte(uint8_t data) {
                                     /* Wait for empty transmit buffer */
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = data;                                            /* send data */
}

uint8_t receiveByte(void) {
  loop_until_bit_is_set(UCSR0A, RXC0);       /* Wait for incoming data */
  return UDR0;                                /* return register value */
}


                       /* Here are a bunch of useful printing commands */

void printString(const char myString[]) {
  uint8_t i = 0;
  while (myString[i]) {
    transmitByte(myString[i]);
    i++;
  }
}

void readString(char myString[], uint8_t maxLength) {
  char response;
  uint8_t i;
  i = 0;
  while (i < (maxLength - 1)) {                   /* prevent over-runs */
    response = receiveByte();
    transmitByte(response);                                    /* echo */
    if (response == '\r') {                     /* enter marks the end */
      break;
    }
    else {
      myString[i] = response;                       /* add in a letter */
      i++;
    }
  }
  myString[i] = 0;                          /* terminal NULL character */
}

void printByte(uint8_t byte) {
              /* Converts a byte to a string of decimal text, sends it */
  transmitByte('0' + (byte / 100));                        /* Hundreds */
  transmitByte('0' + ((byte / 10) % 10));                      /* Tens */
  transmitByte('0' + (byte % 10));                             /* Ones */
}

void printHexByte(uint8_t byte) {
                        /* Prints a byte as its hexadecimal equivalent */
  uint8_t nibble;
  nibble = (byte & 0b11110000) >> 4;
  transmitByte(nibbleToHexCharacter(nibble));
  nibble = byte & 0b00001111;
  transmitByte(nibbleToHexCharacter(nibble));
}

char nibbleToHexCharacter(uint8_t nibble) {
                                   /* Converts 4 bits into hexadecimal */
  if (nibble < 10) {
    return ('0' + nibble);
  }
  else {
    return ('A' + nibble - 10);
  }
}
