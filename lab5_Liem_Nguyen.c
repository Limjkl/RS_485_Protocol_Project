//Lab3b
//Liem Nguyen

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

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

// Port masks
#define DE_MASK 32

//define

#define MAX_CHARS 80
#define MAX_FIELDS 5
typedef struct _USER_DATA
{ char buffer[MAX_CHARS+1];
uint8_t fieldCount;
uint8_t fieldPosition[MAX_FIELDS];
char fieldType[MAX_FIELDS];
} USER_DATA;

#define NULL 0
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
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
    else if (n == 8 | n == 127 & count > 0)
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
        if(data->buffer[i]>=48 & data->buffer[i]<=57 | data->buffer[i]==45 | data->buffer[i]==46)
        {
                   if(state!=1 & state!=2)
                   { data->fieldType[offset]='n';
                     data->fieldPosition[offset]=i;
                     data->fieldCount++;
                     offset++;
                     state=1;
                   }
                   else {goto cont;}
        }
        else if (data->buffer[i] >= 65 & data->buffer[i] <=90  )
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
        else if(data->buffer[i]>=97  & data->buffer[i]<=122 )
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
        else {state=0; data->buffer[i]=0;}
        cont:if(data->fieldCount==MAX_CHARS)
                {
                    i=MAX_CHARS;
                    return;
                }

    }

}
char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
    if (fieldNumber <= data->fieldCount & data->fieldType[fieldNumber] == 'a' )
    {
      return &data->buffer[data->fieldPosition[fieldNumber]];
    }
    else return NULL;
}
int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    int number=0;
    int offset=0;
    if (fieldNumber <= data->fieldCount & data->fieldType[fieldNumber] == 'n' )
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
// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R1;
    _delay_cycles(3);

    // Configure LED pins
    GPIO_PORTB_DIR_R |= DE_MASK;  // bits 1 and 3 are outputs
    GPIO_PORTB_DR2R_R |= DE_MASK ; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTB_DEN_R |= DE_MASK ;  // enable DE pin
}

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
    bool valid;
    // Setup UART0 baud rate
  setUart0BaudRate(115200, 40e6);

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
      // set add, data as add and data are integers
      if ( isCommand( &data, "set", 2))
      {
      int32_t add = getFieldInteger(&data, 1);
      int32_t data1 = getFieldInteger(&data, 2);
      valid = true;
      // do something with this information
      }
      // alert ONOFF   alert ON or alert OFF are the expected commands
      if ( isCommand(&data, "alert", 1))
      {
      char* str = getFieldString(&data, 1);
      putsUart0(str);
      putcUart0('\r');
      putcUart0('\n');
      valid = true;
      // process the string with your custom strcmp instruction, then do something
      }
      // Process other commands here
      // Look for error
      if (!valid)
      {
       putsUart0("Invalid command\r\n");
      }
      }
}



