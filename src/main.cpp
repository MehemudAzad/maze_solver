#define F_CPU 1000000UL
#include <avr/io.h>

//
#ifndef USART_RS232_H_FILE_H_        /* Define library H file if not defined */
#define USART_RS232_H_FILE_H_

#define F_CPU 1000000UL            /* Define CPU clock Frequency e.g. here its 8MHz */
#include <avr/io.h>              /* Include AVR std. library file */
#define BAUD_PRESCALE (((F_CPU / (BAUDRATE * 16UL))) - 1)  /* Define prescale value */

void USART_Init(unsigned long);        /* USART initialize function */
char USART_RxChar();            /* Data receiving function */
void USART_TxChar(char);          /* Data transmitting function */
void USART_SendString(char*);        /* Send string of USART data function */


#endif                    /* USART_RS232_H_FILE_H_ */

void USART_Init(unsigned long BAUDRATE)        /* USART initialize function */
{
  UCSRB |= (1 << RXEN) | (1 << TXEN);        /* Enable USART transmitter and receiver */
  UCSRC |= (1 << URSEL)| (1 << UCSZ0) | (1 << UCSZ1);  /* Write USCRC for 8 bit data and 1 stop bit */
  UBRRL = BAUD_PRESCALE;              /* Load UBRRL with lower 8 bit of prescale value */
  UBRRH = (BAUD_PRESCALE >> 8);          /* Load UBRRH with upper 8 bit of prescale value */
}

char USART_RxChar()                  /* Data receiving function */
{
  while (!(UCSRA & (1 << RXC)));          /* Wait until new data receive */
  return(UDR);                  /* Get and return received data */
}

void USART_TxChar(char data)            /* Data transmitting function */
{
  UDR = data;                    /* Write data to be transmitting in UDR */
  while (!(UCSRA & (1<<UDRE)));          /* Wait until data transmit and buffer get empty */
}

void USART_SendString(char *str)          /* Send string of USART data function */
{
  int i=0;
  while (str[i]!=0)
  {
    USART_TxChar(str[i]);            /* Send each char of string till the NULL */
    i++;
  }
}
//


#define LED PORTB    /* connected LED on PORT pin */

void motor_init(){
  DDRA = 0xff;
  DDRB = 0xff;
}

// IN1 - PA0
// IN2 - PA1
// IN3 - PA2
// IN4 - PA3

void motor_forward(){
  //IN1 - HI, IN2 - LO, IN3 - HI, IN4 - LO
  PORTA = 0b00000101;
//   PORTB = 0xff; 
}

void motor_stop(){
  PORTA = 0b00000000;
  PORTB = 0x00;
}

void motor_reverse(){
  
  PORTA = 0b00001010;
  PORTB = 0xff;
}

void motor_right(){
  PORTA = 0b00001001;
  PORTB = 0xff;
}

void motor_left(){
  PORTA = 0b00000110;
  PORTB = 0xff;
}


int main(void)
{
  char Data_in;
  DDRB = 0xff;    /* make PORT as output port */
  USART_Init(9600);  /* initialize USART with 9600 baud rate */
  motor_init();
  LED = 0b00000001;
  motor_forward();  /* start motor in forward direction */
  while(1)
  {
    
    Data_in = USART_RxChar();  /* receive data from Bluetooth device*/
    if(Data_in == 'F')
    {
      LED = 0b00000011;  /* Turn ON LED */
    //   motor_forward();
      USART_SendString("LED_ON | Going forward");/* send status of LED i.e. LED ON */
      
    }
    else if(Data_in == 'B')
    {
      LED = 0b00000011;  /* Turn ON LED */
    //   motor_reverse();
      USART_SendString("LED_ON | Going Reverse");
      /* send status of LED i.e. LED OFF */
    }
    else if(Data_in == 'S')
    {
      LED = 0b00000001;  /* Turn OFF LED */
    //   motor_stop();
      USART_SendString("LED_OFF | STOP");
      /* send status of LED i.e. LED OFF */
    }
    else if(Data_in == 'L')
    {
      LED = 0b00000011;  
    //   motor_left();
      USART_SendString("LED_ON | Going LEFT");
      /* send status of LED i.e. LED OFF */
    }
    else if(Data_in == 'R')
    {
      LED = 0x00000011;  /* Turn ON LED */
    //   motor_right();
      USART_SendString("LED_ON | Going RIGHT");
      /* send status of LED i.e. LED OFF */
    }
    else{
        USART_SendString("SELECT AMONG F,B,R,L");
        // motor_forward();
        motor_right();
    }
  }

}



// #include <avr/io.h>

// int main(void) {
//     // Set all ports as output
//     DDRA = 0xFF;  // Data Direction Register A - All pins output
//     DDRB = 0xFF;  // Data Direction Register B - All pins output
//     DDRC = 0xFF;  // Data Direction Register C - All pins output
//     DDRD = 0xFF;  // Data Direction Register D - All pins output

//     // Set all ports high
//     PORTA = 0xFF; // Set all bits of PORTA high
//     PORTB = 0xFF; // Set all bits of PORTB high
//     PORTC = 0xFF; // Set all bits of PORTC high
//     PORTD = 0xFF; // Set all bits of PORTD high

//     while (1) {
//         // Infinite loop to keep the program running
//     }

//     return 0;
// }
