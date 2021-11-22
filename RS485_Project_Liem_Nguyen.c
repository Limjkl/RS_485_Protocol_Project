//RS_485_Protocol_Project
//Liem Nguyen
// liem.nguyen3@mavs.uta.edu

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PA6 drives the red LED that will blink when a package is transmitted
// Green LED:
//   PE5 drives the green LED that will blink when a package is received
// UART0 user Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1
// UART1 RS485 Interface:
//   U1TX (PB1) and U1RX (PB0) are connected to the RS-485 Transceiver
//   The  RS-485 Transceiver is connected to a common bus which multiple nodes can communicate with each others
//   Configured to 38,400 baud, 8S1
//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "clock.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"

// Bitband aliases
#define DE      (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 5*4))) //PB5, RS_485 IC enable pin
#define TX_LED  (*((volatile uint32_t *)(0x42000000 + (0x400003FC-0x40000000)*32 + 6*4)))//PA6 Red LED
#define RX_LED  (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 5*4)))//PE5  Green LED
// Port masks

#define DE_MASK 32
#define RX_LED_MASK 32
#define TX_LED_MASK 64

//Global variables define

#define NULL 0
#define MAX_CHARS 80
#define MAX_FIELDS 5

typedef struct _USER_DATA
{ char buffer[MAX_CHARS+1];
uint8_t fieldCount;
uint8_t fieldPosition[MAX_FIELDS];
char fieldType[MAX_FIELDS];
} USER_DATA;

typedef struct _TX485_MSG
{ uint8_t DST_ADDRESS;
char COMMAND;
uint8_t CHANNEL;
uint8_t SIZE;
char data[MAX_CHARS];
uint8_t SEQ_ID;
uint8_t CHECKSUM;
bool VALID;
} TX485_MSG;

//-----------------------------------------------------------------------------
// Prototype Functions Call
//-----------------------------------------------------------------------------

void getsUart0(USER_DATA* data);
void parseFields(USER_DATA* data);
char* getFieldString(USER_DATA* data, uint8_t fieldNumber);
int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber);
bool isCommand(USER_DATA* data, const char strCommand[],uint8_t minArguments);
void initHw();
bool strcmp(char* str,const char strCommand[]);
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
#define DEBUG
int main(void)
{
    // Initialize hardware
    USER_DATA data;
    initHw();
    initUart0();
    initUart1();
    bool valid;


  while (true)
      {
      valid = 0;
      getsUart0(&data);
     #ifdef DEBUG
      putsUart0(data.buffer);
      putcUart0('\r');
      putcUart0('\n');
    #endif
      parseFields(&data);
    #ifdef DEBUG
      uint8_t i;
      for (i = 0; i < data.fieldCount; i++)
      {
      putcUart0(data.fieldType[i]);
      putcUart0('\r');
      putcUart0('\n');
      putsUart0(&data.buffer[data.fieldPosition[i]]);
      putcUart0('\r');
      putcUart0('\n');
      }
    #endif
      // Command evaluation
      // Reset command function
      if ( isCommand( &data, "Reset", 0))
      {
          //write key to the register field in order to change the bits in this register
          // request a reset for the core and all on-chip peripherals except the debug interface
          NVIC_APINT_R  |= NVIC_APINT_VECTKEY|NVIC_APINT_SYSRESETREQ |NVIC_APINT_VECT_RESET;
          valid = true;
      }
      // CS command base on ON or OFF set/clear a global bit
      if ( isCommand(&data, "cs", 1))
            {
            char* str = getFieldString(&data, 1);
           if( strcmp(str, "ON"))
            {
                valid = true;// global bit is set here
            }
           else if (strcmp(str, "OFF"))
           {
               valid = false;// global bit is cleared here
           }
            }
      //random command ON/OFF set/clear a global bit
      if ( isCommand(&data, "random", 1))
      {
          char* str = getFieldString(&data, 1);
          if( strcmp(str, "ON"))
          {
              valid = true;// global bit is set here
          }
          else if (strcmp(str, "OFF"))
          {
              valid = false;// global bit is cleared here
          }
      }
      // set command
      if ( isCommand( &data, "set", 3))
           {
           int32_t add = getFieldInteger(&data, 1);
           int32_t channel = getFieldInteger(&data, 2);
           int32_t value = getFieldInteger(&data, 3);
           valid = true;
           // do something with this information with sendRS485
           }
      //get command
      if ( isCommand( &data, "get", 2))
      {
          int32_t add = getFieldInteger(&data, 1);
          int32_t channel = getFieldInteger(&data, 2);

          valid = true;
          // do something with this information with sendRS485
      }
      //poll command
      if ( isCommand( &data, "poll", 0))
      {
          //send a poll request?
          valid = true;
          // do something with this information with sendRS485
      }
      //sa command
      if ( isCommand( &data, "sa", 2))
      {
          int32_t add = getFieldInteger(&data, 1);
          int32_t newadd = getFieldInteger(&data, 2);

          valid = true;
          // do something with this information with sendRS485
      }
      //square command
      if ( isCommand( &data, "square", 5))
      {
          int32_t Value1 = getFieldInteger(&data, 1);
          int32_t Value2 = getFieldInteger(&data, 2);
          int32_t Time1 = getFieldInteger(&data, 3);
          int32_t Time2 = getFieldInteger(&data, 4);
          int32_t Cycles = getFieldInteger(&data, 5);
          valid = true;
          // do something with this information with sendRS485
      }
      //pulse command
      if ( isCommand( &data, "pulse", 2))
      {
          int32_t Value1 = getFieldInteger(&data, 1);
          int32_t Duration = getFieldInteger(&data, 2);
          valid = true;
          // do something with this information with sendRS485
      }
      // Look for error
      if (!valid)
      {
       putsUart0("Invalid command\r\n");
      }



      }
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks for port B,E,A
    SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R1|SYSCTL_RCGCGPIO_R4|SYSCTL_RCGCGPIO_R0;
    _delay_cycles(3);

    // Configure LED and DE pins
    GPIO_PORTB_DIR_R |= DE_MASK;  // bits 5 are outputs
    GPIO_PORTB_DR2R_R |= DE_MASK ; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTB_DEN_R |= DE_MASK ;  // enable DE pin

    GPIO_PORTA_DIR_R |= TX_LED_MASK;  // bits 6 are outputs
    GPIO_PORTA_DR2R_R |= TX_LED_MASK ; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R |= TX_LED_MASK ;  // enable Red LED pin

    GPIO_PORTE_DIR_R |= RX_LED_MASK;  // bits 5 are outputs
    GPIO_PORTE_DR2R_R |=RX_LED_MASK ; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTE_DEN_R |= RX_LED_MASK ;  // enable Green LED pin
}

bool strcmp(char* str,const char strCommand[])
{
    int i=0;
    while(strCommand[i]!=NULL)
    {
        if(strCommand[i]-str[i]!=0)
        {
            return 0;
        }
        i++;
    }
    return 1;
}

bool isCommand(USER_DATA* data, const char strCommand[],uint8_t minArguments)
{
    if(minArguments<=(data->fieldCount - 1))
    {
        int i=data->fieldPosition[0];// first field
        int j=0;
        while(data->buffer[i]!=NULL)
        {
           if(strCommand[j]-data->buffer[i]!=0)
               {
               return 0;
               }
           i++;
           j++;
        }
       return 1;
    }
    else return 0;
}

int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    int number=0;
    int offset=0;
    if (fieldNumber <= data->fieldCount && data->fieldType[fieldNumber] == 'n' )
        {
        offset = data->fieldPosition[fieldNumber];
        while (data->buffer[offset] != 0)
        {
        number = number*10 + data->buffer[offset] - '0';
          offset++;
        }
        return number;
        }
        else return NULL;
}

char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{

    if (fieldNumber <= data->fieldCount & data->fieldType[fieldNumber] == 'a' )
      {
        return &data->buffer[data->fieldPosition[fieldNumber]];
      }
      else return NULL;
}

void parseFields(USER_DATA* data)
{
    data->fieldCount=0;
    int i;
    int offset=0;
    int state=0; //0: delimiter, 1: numeric, 2:alpha
  for ( i =0; i<=MAX_CHARS;i++)
    {
        if(data->buffer[i]==0)
               {
                   i=MAX_CHARS;
                   return;
               }
        if(data->buffer[i]>=48 && data->buffer[i]<=57 || data->buffer[i]==45 || data->buffer[i]==46)
        {
                   if(state!=1 && state!=2)
                   { data->fieldType[offset]='n';
                     data->fieldPosition[offset]=i;
                     data->fieldCount++;
                     offset++;
                     state=1;
                   }
                   else {goto cont;}
        }
        else if (data->buffer[i] >= 65 && data->buffer[i] <=90  )
        {
                   if(state!=2 & state!=1)
                   { data->fieldType[offset]='a';
                     data->fieldPosition[offset]=i;
                     data->fieldCount++;
                     offset++;
                     state=2;
                   }
                   else {goto cont;}
              }
        else if(data->buffer[i]>=97  && data->buffer[i]<=122 )
              {
            if(state!=2 && state!=1)
            { data->fieldType[offset]='a';
              data->fieldPosition[offset]=i;
              data->fieldCount++;
              offset++;
              state=2;
            }
            else {goto cont;}
              }
        else {state=0; data->buffer[i]=0;}
        cont:if(data->fieldCount==MAX_CHARS)
                {
                    i=MAX_CHARS;
                    return;
                }

    }

}

void getsUart0(USER_DATA* data)
{
    uint8_t count=0;
    char n;
    int i=1;
    while (i)
    {
    n= getcUart0();
    if(count== MAX_CHARS )
    {
        data->buffer[count]=0;
        i=1;
        return;
    }
    else if (n == 8 || n == 127 && count > 0)
    {
        count--;
    }
    else if (n == 13 )
      {
          data->buffer[count]=0;
          i=0;
          return;
      }
    else if (n>=32)
    {
        data->buffer[count]=n;
        count++;
    }

    }
}
