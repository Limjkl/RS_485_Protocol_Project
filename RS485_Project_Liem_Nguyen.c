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
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "clock.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"

// Bitband aliases
#define DE      (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 5*4))) //PB5, RS_485 IC enable pin
#define TX_LED  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))//PA6 Red LED
#define RX_LED  (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 5*4)))//PE5  Green LED
#define PIN_OUT  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))//PA4 pulse and square driving
#define BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4)))//PA3 pulse and square driving

// Port masks
#define RED_RGB_MASK 2
#define BLUE_RGB_MASK 4
#define GREEN_RGB_MASK 8
#define DE_MASK 32
#define RX_LED_MASK 32
#define TX_LED_MASK 64
#define PIN_OUT_MASK 16
#define BUTTON_MASK 8

//Global variables define
#define CS_PERIOD 2
#define MAX_TIMEOUT 2
#define NULL 0
#define MAX_CHARS 80
#define MAX_FIELDS 5
#define Default_ADD 0x01
#define MAX_Q 256
#define MAX_DATA 5
#define MAX_COUNT 3 // total number of attempt
#define min_backoff_time 100
#define T 70

typedef struct _USER_DATA
{
    char buffer[MAX_CHARS + 1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;

typedef struct _TX485_MSG
{
    uint8_t DST_ADDRESS;
    uint8_t SRC_ADDRESS;
    uint8_t COMMAND;
    uint8_t CHANNEL;
    uint8_t SIZE;
    uint8_t data[MAX_DATA];
    uint8_t SEQ_ID;
    uint8_t CHECKSUM;
    bool VALID;
    uint8_t TX_count;
} TX485_MSG;

typedef struct _action
{
    uint8_t command;
    uint16_t time1;
    uint16_t level1;
    uint16_t time2;
    uint16_t level2;
    uint16_t count;
    bool valid;
    bool phase;
    uint16_t delta_T;
} action;

action action_var;
char itoa_buffer[3] = { 32 };
uint8_t RX_index = 0;
uint8_t TX_index = 0;
uint8_t UI_index = 0;
TX485_MSG MSG_Q[MAX_Q] = { 0 };
TX485_MSG RX_MSG = { 0 };
char UIbuffer[MAX_CHARS + 1] = { 0 };
bool MSGinProgress = 0;
uint8_t TX_phase = 0;
uint8_t TX_LED_timeout = 0;
uint8_t RX_LED_timeout = 0;
uint8_t RX_phase = 0;
uint8_t SEQ_ID = 0;
uint8_t MY_ADD;
bool ack = 0;
bool random = 0;
bool carrier_sense = 0;
bool busy=0;
uint8_t test_cs=0;
bool test_done=0;
int32_t time_to_TX = 0;
//-----------------------------------------------------------------------------
// Prototype Functions Call
//-----------------------------------------------------------------------------

void getsUart0(USER_DATA *data);
void parseFields(USER_DATA *data);
char* getFieldString(USER_DATA *data, uint8_t fieldNumber);
int32_t getFieldInteger(USER_DATA *data, uint8_t fieldNumber);
bool isCommand(USER_DATA *data, const char strCommand[], uint8_t minArguments);
void initHw(); //timer init, eeprom init also in here
void sendRS485(uint8_t DST_ADD, uint8_t CMD, uint8_t Channel, uint8_t SIZE,uint8_t data[], bool ack);
void sendRS485Byte();
bool mystrcmp(char *str, const char strCommand[]);
void processdata(uint8_t DST_ADD, uint8_t SRC_ADD, uint8_t SEQ_ID, uint8_t CMD,uint8_t Channel, uint8_t SIZE, uint8_t proc_data[]);
void sendUImessage(char str[], uint8_t N1, uint8_t N2);
void itoa(int num, char *str, int base);
int32_t readeeprom(uint8_t pos);
void writeeeprom(uint8_t pos, uint8_t new_add);
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
#define DEBUG
int main(void)
{
    // Initialize hardware
    initHw();
    initUart0();
    initUart1();
    bool valid;
    USER_DATA data;
    if (readeeprom(0) == EEPROM_EERDWR_VALUE_M)
        {
            MY_ADD = Default_ADD;
        }
        else
        {
            MY_ADD = readeeprom(0);
        }
    RX_LED = 1;
    RX_LED_timeout = MAX_TIMEOUT;
    while (true)
    {
        if(!RX_MSG.VALID)
        {
        RX_MSG.CHANNEL=0;
        RX_MSG.CHECKSUM=0;
        RX_MSG.COMMAND=0;
        RX_MSG.DST_ADDRESS=0;
        RX_MSG.SEQ_ID=0;
        RX_MSG.SIZE=0;
        RX_MSG.SRC_ADDRESS=0;
        RX_MSG.data[0]=0;
        RX_MSG.data[1]=0;
        RX_MSG.data[2]=0;
        RX_MSG.data[3]=0;
        RX_MSG.data[4]=0;
        RX_MSG.VALID=0;
        }
        valid = 0;
        getsUart0(&data);

#ifdef DEBUG
        putsUart0(data.buffer);
        putcUart0('\r');
        putcUart0('\n');
#endif
        parseFields(&data);
        // Command evaluation
        // Reset command function
        if (isCommand(&data, "MYReset", 0))
        {
            while(!(UART1_RIS_R&UART_RIS_TXRIS) && !(UART0_RIS_R&UART_RIS_TXRIS))
                  {
                      NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;//reset
                  }
            valid = true;
            while (1);
        }
        // Reset command function for other nodes
        else if (isCommand(&data, "Reset", 1))
        {
            uint8_t add = getFieldInteger(&data, 1);
            sendRS485(add, 0x7F, 0, 0, 0, ack);
            valid = true;
        }
        // CS command base on ON or OFF set/clear a global bit
        else if (isCommand(&data, "cs", 1))
        {
            char *str = getFieldString(&data, 1);
            if (mystrcmp(str, "ON"))
            {
                carrier_sense = true;
                valid = true;
            }
            else if (mystrcmp(str, "OFF"))
            {
                carrier_sense = false;
                valid = true;
            }
        }
        //random command ON/OFF set/clear a global bit
        else if (isCommand(&data, "random", 1))
        {
            char *str = getFieldString(&data, 1);
            if (mystrcmp(str, "ON"))
            {
                random = true;      // global bit is set here
                valid = true;
            }
            else if (mystrcmp(str, "OFF"))
            {
                random = false;      // global bit is cleared here
                valid = true;
            }
        }
        //ack on off
        else if (isCommand(&data, "ack", 1))
        {
            char *str = getFieldString(&data, 1);
            if (mystrcmp(str, "ON"))
            {
                ack = 1;
                valid = true;
            }
            else if (mystrcmp(str, "OFF"))
            {
                ack = 0;
                valid = true;
            }
        }
        //read my add
        else if (isCommand(&data, "MYADD", 0))
        {
            sendUImessage(" My Address is %\r\n", readeeprom(0), 0);
            valid = true;
        }
        //set my add
        else if (isCommand(&data, "SET MY ADD", 3))
        {
            uint8_t add = getFieldInteger(&data, 3);
            writeeeprom(0, add);
            MY_ADD = readeeprom(0);
            sendUImessage("My New Address is %\r\n", MY_ADD, 0);
            valid = true;
        }
        // set command
        else if (isCommand(&data, "set", 3))//set command 0x00
        {
            uint8_t add = getFieldInteger(&data, 1);
            uint8_t channel = getFieldInteger(&data, 2);
            uint8_t value = getFieldInteger(&data, 3);
            uint8_t data[1];
            data[0] = value;
            sendRS485(add, 0x00, channel, 1, data, ack);
            valid = true;
        }
        //get command
        else if (isCommand(&data, "get", 2))//data request 0x30
        {
            uint8_t add = getFieldInteger(&data, 1);
            uint8_t channel = getFieldInteger(&data, 2);
            sendRS485(add, 0x30, channel, 0, 0, ack);
            valid = true;
        }
        //poll command
        else if (isCommand(&data, "poll", 0))      //poll request 0x78 for all add at 255
        {
            sendRS485(0xFF, 0x78, 0, 0, 0, ack);
            valid = true;
        }
        // RGB command
        else if (isCommand(&data, "RGB", 4))//RGB 0x48
        {
            uint8_t add = getFieldInteger(&data, 1);
            uint8_t RED_R = getFieldInteger(&data, 1);
            uint8_t BLUE_R = getFieldInteger(&data, 2);
            uint8_t GREEN_R = getFieldInteger(&data, 3);
            uint8_t data[3];
            data[0]=RED_R;
            data[1]=BLUE_R;
            data[2]=GREEN_R;
            sendRS485(add, 0x48, 0, 3, data, ack);
            valid = true;
        }
        //MYRGB command
        else if (isCommand(&data, "MYRGB", 3))
        {
            uint8_t RED_R = getFieldInteger(&data, 1);
            uint16_t RED_V = (RED_R / 100) * 1023; //range from 0 to 100 for brightness
            uint8_t BLUE_R = getFieldInteger(&data, 2);
            uint16_t BLUE_V = (BLUE_R / 100) * 1023;
            uint8_t GREEN_R = getFieldInteger(&data, 3);
            uint16_t GREEN_V = (GREEN_R / 100) * 1023;
            PWM1_2_CMPB_R = RED_V; //red
            PWM1_3_CMPA_R = BLUE_V; //blue
            PWM1_3_CMPB_R = GREEN_V; //green
            valid = true;
        }
        //sa command
        else if (isCommand(&data, "sa", 2)) //set address command 0x7a
        {
            uint8_t add = getFieldInteger(&data, 1);
            uint8_t newadd = getFieldInteger(&data, 2);
            uint8_t data[1];
            data[0] = newadd;
            sendRS485(add, 0x7A, 0, 1, data, ack);
            valid = true;
        }
        //square command
        else if (isCommand(&data, "square", 6))//square 1 2 3 10 10 1000 //pin PA4
        {
            uint8_t add = getFieldInteger(&data, 1);
            uint16_t Value1 = getFieldInteger(&data, 2);
            uint16_t Value2 = getFieldInteger(&data, 3);
            uint16_t Time1 = getFieldInteger(&data, 4);
            uint16_t Time2 = getFieldInteger(&data, 5);
            uint16_t Cycles = getFieldInteger(&data, 6);
            uint8_t data[8];
            data[0] = Time1 & 0xFF;
            data[1] = ((Time1 & 0xFF00) >> 8);
            data[2] = Time2 & 0xFF;
            data[3] = ((Time2 & 0xFF00) >> 8);
            data[4] = Cycles & 0xFF;
            data[5] = ((Cycles & 0xFF00) >> 8);
            data[6] = Value1;
            data[7]=Value2;
            PIN_OUT=1;
            action_var.command=0x03;
            action_var.time1=Time1;
            action_var.time2=Time2;
            action_var.count=Cycles;
            action_var.valid=1;
            action_var.phase=0;
            action_var.delta_T=0;
            sendRS485(Value1, 0x03, 0, 8, data, ack);
            valid = true;
        }
        //pulse command
        else if (isCommand(&data, "pulse", 3))//pulse 1 2 10 //pin PA4
        {
            uint8_t add = getFieldInteger(&data, 1);
            uint16_t Value = getFieldInteger(&data, 2);
            uint16_t Duration = getFieldInteger(&data, 3);
            uint8_t data[3];
            data[0]=Duration&0xFF;
            data[1]=((Duration&0xFF00)>>8);
            data[2]=Value;
            PIN_OUT=1;
            action_var.command=0x02;
            action_var.time1=Duration;
            action_var.valid=1;
            action_var.phase=0;
            action_var.delta_T=0;
            sendRS485(add, 0x02, 0, 3, data, ack);
            valid = true;
        }
        // Look for error
        if (!valid)
        {
            sendUImessage(" Invalid command\r\n", 0, 0);
        }
    }
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

int32_t readeeprom(uint8_t pos)
{
    EEPROM_EEBLOCK_R = pos;
    EEPROM_EEOFFSET_R = pos;
    return EEPROM_EERDWR_R;
}

void writeeeprom(uint8_t pos, uint8_t new_add)
{
    EEPROM_EEBLOCK_R = pos;
    EEPROM_EEOFFSET_R = pos;
    EEPROM_EERDWR_R = new_add;
}

void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks for port B,E,A
    SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R1 | SYSCTL_RCGCGPIO_R4 | SYSCTL_RCGCGPIO_R0|SYSCTL_RCGCGPIO_R5;
    // Enable clocks for timer1 and PWM1
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    _delay_cycles(3);
    //enable eeprom
    SYSCTL_RCGCEEPROM_R =SYSCTL_RCGCEEPROM_R0;
        _delay_cycles(6);
        while((EEPROM_EEDONE_R&EEPROM_EEDONE_WORKING));
        if(EEPROM_EESUPP_R&EEPROM_EESUPP_PRETRY || EEPROM_EESUPP_R&EEPROM_EESUPP_ERETRY)
        {
            sendUImessage("EEPROM init error\r\n", 0, 0);
        }

    // Configure LED and DE pins
    GPIO_PORTB_DIR_R |= DE_MASK;  // bits 5 are outputs
    GPIO_PORTB_DR2R_R |= DE_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTB_DEN_R |= DE_MASK;  // enable DE pin

    GPIO_PORTA_DIR_R |= TX_LED_MASK|PIN_OUT_MASK;  // bits 6 are outputs
    GPIO_PORTA_DIR_R &= ~BUTTON_MASK;//pa3 for input
    GPIO_PORTA_DR2R_R |= TX_LED_MASK|PIN_OUT_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R |= TX_LED_MASK|PIN_OUT_MASK|BUTTON_MASK;  // enable Red LED pin
    GPIO_PORTA_PUR_R |= BUTTON_MASK;

    GPIO_PORTE_DIR_R |= RX_LED_MASK;  // bits 5 are outputs
    GPIO_PORTE_DR2R_R |= RX_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTE_DEN_R |= RX_LED_MASK;  // enable Green LED pin
    // Configure RGB LED pins
    GPIO_PORTF_DIR_R |= RED_RGB_MASK | GREEN_RGB_MASK | BLUE_RGB_MASK;
    GPIO_PORTF_DEN_R |= RED_RGB_MASK | GREEN_RGB_MASK | BLUE_RGB_MASK;
    GPIO_PORTF_AFSEL_R |= RED_RGB_MASK | GREEN_RGB_MASK | BLUE_RGB_MASK;
    GPIO_PORTF_PCTL_R &= ~(GPIO_PCTL_PF1_M | GPIO_PCTL_PF2_M | GPIO_PCTL_PF3_M);
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF1_M1PWM5 | GPIO_PCTL_PF2_M1PWM6 | GPIO_PCTL_PF3_M1PWM7;
    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;      // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;    // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // configure for periodic mode (count down)
    TIMER1_TAILR_R = 400000; // set load value to 40e4 for 100 Hz interrupt rate at system clock of 40e6HZ
    TIMER1_IMR_R = TIMER_IMR_TATOIM; // turn-on interrupts for timeout in timer module
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    NVIC_EN0_R |= 1 << (INT_TIMER1A - 16); // turn-on interrupt 37 (TIMER1A) in NVIC
    // Configure PWM module
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM1 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM1_2_CTL_R = 0;         // turn-off PWM1 generator 2 (drives outs 4 and 5)
    PWM1_3_CTL_R = 0;         // turn-off PWM1 generator 3 (drives outs 6 and 7)
    PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
    // output 5 on PWM1, gen 2b, cmpb
    PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
    // output 6 on PWM1, gen 3a, cmpa
    PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
    // output 7 on PWM1, gen 3b, cmpb

    PWM1_2_LOAD_R = 1024; // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM1_3_LOAD_R = 1024;

    PWM1_2_CMPB_R = 0;               // red off (0=always low, 1023=always high)
    PWM1_3_CMPB_R = 0;                               // green off
    PWM1_3_CMPA_R = 0;                               // blue off

    PWM1_2_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM1 generator 2
    PWM1_3_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM1 generator 3
    PWM1_ENABLE_R = PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
}

bool mystrcmp(char *str, const char strCommand[])
{
    int i = 0;
    while (strCommand[i] != NULL)
    {
        if (strCommand[i] - str[i] != 0)
        {
            return 0;
        }
        i++;
    }
    return 1;
}

bool isCommand(USER_DATA *data, const char strCommand[], uint8_t minArguments)
{
    if (minArguments <= (data->fieldCount - 1))
    {
        int i = data->fieldPosition[0];             // first field
        int j = 0;
        while (data->buffer[i] != NULL)
        {
            if (strCommand[j] - data->buffer[i] != 0)
            {
                return 0;
            }
            i++;
            j++;
        }
        return 1;
    }
    else
        return 0;
}

int32_t getFieldInteger(USER_DATA *data, uint8_t fieldNumber)
{
    int number = 0;
    int offset = 0;
    if (fieldNumber <= data->fieldCount && data->fieldType[fieldNumber] == 'n')
    {
        offset = data->fieldPosition[fieldNumber];
        while (data->buffer[offset] != 0)
        {
            number = number * 10 + data->buffer[offset] - '0';
            offset++;
        }
        return number;
    }
    else
        return NULL;
}

char* getFieldString(USER_DATA *data, uint8_t fieldNumber)
{

    if (fieldNumber <= data->fieldCount & data->fieldType[fieldNumber] == 'a')
    {
        return &data->buffer[data->fieldPosition[fieldNumber]];
    }
    else
        return NULL;
}

void parseFields(USER_DATA *data)
{
    data->fieldCount = 0;
    int i;
    int offset = 0;
    int state = 0; //0: delimiter, 1: numeric, 2:alpha
    for (i = 0; i <= MAX_CHARS; i++)
    {
        if (data->buffer[i] == 0)
        {
            i = MAX_CHARS;
            return;
        }
        if (data->buffer[i] >= 48 && data->buffer[i] <= 57
                || data->buffer[i] == 45 || data->buffer[i] == 46)
        {
            if (state != 1 && state != 2)
            {
                data->fieldType[offset] = 'n';
                data->fieldPosition[offset] = i;
                data->fieldCount++;
                offset++;
                state = 1;
            }
            else
            {
                goto cont;
            }
        }
        else if (data->buffer[i] >= 65 && data->buffer[i] <= 90)
        {
            if (state != 2 & state != 1)
            {
                data->fieldType[offset] = 'a';
                data->fieldPosition[offset] = i;
                data->fieldCount++;
                offset++;
                state = 2;
            }
            else
            {
                goto cont;
            }
        }
        else if (data->buffer[i] >= 97 && data->buffer[i] <= 122)
        {
            if (state != 2 && state != 1)
            {
                data->fieldType[offset] = 'a';
                data->fieldPosition[offset] = i;
                data->fieldCount++;
                offset++;
                state = 2;
            }
            else
            {
                goto cont;
            }
        }
        else
        {
            state = 0;
            data->buffer[i] = 0;
        }
        cont: if (data->fieldCount == MAX_CHARS)
        {
            i = MAX_CHARS;
            return;
        }

    }

}

void getsUart0(USER_DATA *data)
{
    uint8_t count = 0;
    char n;
    int i = 1;
    while (i)
    {
        n = getcUart0();
        if (count == MAX_CHARS)
        {
            data->buffer[count] = 0;
            i = 1;
            return;
        }
        else if (n == 8 || n == 127 && count > 0)
        {
            count--;
        }
        else if (n == 13)
        {
            data->buffer[count] = 0;
            i = 0;
            return;
        }
        else if (n >= 32)
        {
            data->buffer[count] = n;
            count++;
        }

    }
}

void sendUImessage(char str[], uint8_t N1, uint8_t N2)
{
   //  while((UART0_RIS_R&UART_RIS_TXRIS));
    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t k = 0;
    uint8_t count = 0;
    UI_index = 0;
    for (i = 0; i <= MAX_CHARS; i++)
    {
        UIbuffer[i] = 0;
    }
    i = 0;
    while (str[k] != NULL)
    {
        if (str[k] == '%')
        {
            if (count == 0)
            {
                if (N1 > 9)
                {
                    itoa(N1, itoa_buffer, 10);
                    UIbuffer[i] = itoa_buffer[j];
                    j++;
                    i++;
                    while (j > 0 && j < 3)
                    {
                        UIbuffer[i] = itoa_buffer[j];
                        j++;
                        i++;
                    }
                    count++;
                }
                else
                {
                    UIbuffer[i] = N1 + '0';
                    count++;
                    i++;
                }
            }
            else if (count == 1)
            {
                if (N2 > 9)
                {
                    for (j = 0; j < 3; j++)
                    {
                        itoa_buffer[j] = 32;
                    }
                    j = 0;
                    itoa(N2, itoa_buffer, 10);
                    UIbuffer[i] = itoa_buffer[j];
                    j++;
                    i++;
                    while (j > 0 && j < 3)
                    {
                        UIbuffer[i] = itoa_buffer[j];
                        j++;
                        i++;
                    }
                    count++;
                }
                else
                {
                    UIbuffer[i] = N2 + '0';
                    count = 0;
                    i++;
                }
            }
        }
        else
        {
            UIbuffer[i] = str[k];
            i++;
        }
        k++;
    }

    if (!(UART0_RIS_R&UART_RIS_TXRIS))
    {
        //putcUart0(UIbuffer[UI_index]);
        UART0_DR_R = UIbuffer[UI_index];
        UI_index = (UI_index + 1) & 255;
    }

}

void itoa(int num, char *str, int base)
{
    int i = 2;
    while (num != 0)
    {
        int rem = num % base;
        if (rem > 9)
        {
            itoa_buffer[i--] = (rem - 10) + 'a';
        }
        else
        {
            itoa_buffer[i--] = rem + '0';
        }
        num = num / base;
    }
}

void timer1Isr()
{
    if (MSG_Q[TX_index].VALID)
    {
        if (time_to_TX > 0)
        {
            time_to_TX--;
        }
        else if (UART1_FR_R & UART_FR_TXFE && time_to_TX == 0)
        {
            if (!(UART1_RIS_R & UART_RIS_TXRIS))
            {
                sendRS485Byte();
            }

        }
    }
    if (!(UART0_RIS_R&UART_RIS_TXRIS))
        {
        if (UART0_FR_R & UART_FR_TXFE)
          {

              if (UIbuffer[UI_index] != NULL)
              {
                  // putcUart0(UIbuffer[UI_index]);
                  UART0_DR_R = UIbuffer[UI_index];
                  UI_index = (UI_index + 1) & 255;
              }
          }
        }
    if(test_cs>0)
    {
        test_cs--;
        if(test_cs==0)
        {
            test_done=1;
        }
    }
    if (TX_LED_timeout > 0)
    {
        TX_LED_timeout--;
        if (TX_LED_timeout == 0)
        {
            TX_LED = 0;
        }
    }
    if (RX_LED_timeout > 0)
    {
        RX_LED_timeout--;
        if (RX_LED_timeout == 0)
        {
            RX_LED = 0;
        }
    }
    if(action_var.command!=0 &&  action_var.valid)
    {
        action_var.delta_T++;
        if(action_var.phase==0)
        {
            if(action_var.delta_T==action_var.time1)
            {
                action_var.phase=1;
            }
            if (action_var.command == 0x02)
            {
                if (action_var.delta_T == action_var.time1)
                {
                    PIN_OUT = 0;
                    action_var.valid = 0;
                }
            }
            else if (action_var.command == 0x03)
            {
                if (action_var.delta_T == 1)
                {
                    PIN_OUT = 1;
                }
                if (action_var.delta_T == action_var.time1)
                {
                    PIN_OUT = 0;
                    action_var.delta_T=0;
                }
            }
        }
        else if (action_var.phase==1)
        {
            if(action_var.delta_T==action_var.time2)
            {
                action_var.delta_T=0;
                action_var.phase=0;
                action_var.count--;
                if(action_var.count==0)
                {
                    action_var.valid=0;
                }
            }
        }


    }
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}

void UART0ISR(void)
{
    //while(UART0_FR_R & UART_FR_TXFF); //checking if not busy
    if ((UART0_RIS_R&UART_RIS_TXRIS))
    {

        if (UIbuffer[UI_index] != NULL)
        {
            // putcUart0(UIbuffer[UI_index]);
            UART0_DR_R = UIbuffer[UI_index];
            UI_index = (UI_index + 1) & 255;
        }
    }
    UART0_ICR_R = UART_ICR_TXIC;
}

void UART1ISR(void)
{
    //change condition later since there should be no case when rx and tx happens at a same time
    if (UART1_RIS_R&UART_RIS_TXRIS) //transmit fifo is empty and rx fifo is not full
    {
        sendRS485Byte();
    }
    if (!(UART1_RIS_R&UART_RIS_TXRIS))
    {
        busy=1;
        UART1_ICR_R = UART_ICR_RXIC;               //clear interrupt flag
        bool parity_check = 0;

        if (RX_phase == 0)    //checking again if the Parity checking is correct
        {
            parity_check = UART1_RIS_R&UART_RIS_PERIS;
        }
        char RX = UART1_DR_R & 0xFF;          //save receive byte in RX variable

        if (RX_phase == 0 && parity_check && (RX == MY_ADD || RX==0xFF ) && RX_MSG.VALID==0)
        {
            RX_MSG.DST_ADDRESS=RX;
           // sendUImessage("DST_ADD is %r\n", RX_MSG.DST_ADDRESS, 0);
            RX_phase++;
            //  MSGinProgress=1;
        }
        else if (RX_phase > 0)
        {
            switch (RX_phase)
            {
            case 1:
                RX_MSG.SRC_ADDRESS = RX;
            //    sendUImessage("SRC_ADD is %r\n", RX_MSG.SRC_ADDRESS, 0);
                RX_phase++;
                break;
            case 2:
                RX_MSG.SEQ_ID = RX;
              //  sendUImessage("SEQ_ID is %r\n", RX_MSG.SEQ_ID, 0);
                RX_phase++;
                break;
            case 3:
                RX_MSG.COMMAND = RX;
             //   sendUImessage("Command is %r\n", RX_MSG.COMMAND, 0);
                RX_phase++;
                break;
            case 4:
                RX_MSG.CHANNEL = RX;
              //  sendUImessage("Channel is %r\n", RX_MSG.CHANNEL, 0);
                RX_phase++;
                break;
            case 5:
                RX_MSG.SIZE = RX;
            //    sendUImessage("SIZE is %r\n", RX_MSG.SIZE, 0);
                RX_phase++;
                break;
            default:
                if (RX_phase < 6 + RX_MSG.SIZE && RX_phase > 5)
                {
                    RX_MSG.data[RX_phase - 6] = RX;
                   // sendUImessage("data is %r\n", RX_MSG.data[RX_phase - 6], 0);
                    RX_phase++;
                    break;
                }
                else if (RX_phase == 6 + RX_MSG.SIZE
                        && RX_phase < 8 + RX_MSG.SIZE)
                {
                    RX_MSG.CHECKSUM = RX;
                 //   sendUImessage("RX checksum is %r\n", RX_MSG.CHECKSUM, 0);
                    RX_phase++;
                    uint8_t checksum = 0;
                    int counter;
                    checksum = RX_MSG.DST_ADDRESS + RX_MSG.SRC_ADDRESS
                            + RX_MSG.SEQ_ID + RX_MSG.COMMAND + RX_MSG.CHANNEL+ RX_MSG.SIZE;
                    for (counter = 0; counter < RX_MSG.SIZE; counter++)
                    {
                        checksum += RX_MSG.data[counter];
                    }
                    checksum = ~(checksum);
                    if ( checksum== RX_MSG.CHECKSUM)
                    {
                        RX_LED = 1;
                        RX_phase = 0;
                        RX_LED_timeout = MAX_TIMEOUT; // check period later
                        RX_MSG.VALID=1;
processdata(RX_MSG.SRC_ADDRESS, MY_ADD, RX_MSG.SEQ_ID,RX_MSG.COMMAND, RX_MSG.CHANNEL, RX_MSG.SIZE,RX_MSG.data); //process RX data
                        _delay_cycles(9);
                    }
                    else
                    {
                        RX_MSG.VALID=0;
                        RX_LED_timeout=0;
                        RX_LED = 1;
                        RX_phase = 0;
                        break;
                    }
                }
                   // sendUImessage("Calculated checksum is %r\n", ~checksum, 0);
            }
        }
    }
    UART1_ICR_R = UART_ICR_TXIC;
}

void processdata(uint8_t DST_ADD, uint8_t SRC_ADD, uint8_t SEQ_ID, uint8_t CMD, uint8_t Channel, uint8_t SIZE, uint8_t proc_data[])
{

    bool ack_rx= CMD&0x80;//check for MSB in CMD
    CMD &= ~0x80;// clear MSB of CMD
    if (ack_rx) //require an ack to be sent back
    {
        uint8_t ID[1];
        ID[0] = SEQ_ID;
        sendRS485(DST_ADD, 0x70, 0, 1, ID, 0); //resend ack
    }
    if (CMD == 0x70) //when receive an ack as response
    {
        while (SEQ_ID != MSG_Q[RX_index].SEQ_ID && DST_ADD != MSG_Q[RX_index].DST_ADDRESS) //matching rx seq_id and my add with the same message in MSG_Q
        {
            RX_index = (TX_index + 1) & 255;
        }
        sendUImessage("Message % ack received\r\n", SEQ_ID, 0);
        MSG_Q[RX_index].VALID = 0; //mark sent MSG as invalid
        MSGinProgress=0;//stop reTX
        time_to_TX = 0;
        RX_MSG.VALID=0;
    }
    else if(CMD==0x79)//poll response rx
    {
        sendUImessage("Poll response RX from add %\r\n", DST_ADD, 0);
        RX_MSG.VALID=0;
    }
    else if(CMD==0x78)//poll request rx
    {
        sendUImessage("poll command RX\r\n", 0, 0);
        uint8_t ID[1];
        ID[0] = MY_ADD;
        sendRS485(DST_ADD, 0x79, 0, 1, ID, 0); //send my current ADD
        RX_MSG.VALID=0;
    }
    else if (CMD==0x7F)//reset CMD RX
    {
        sendUImessage("Reset command RX\r\n", 0, 0);
        while(!(UART1_RIS_R&UART_RIS_TXRIS) && !(UART0_RIS_R&UART_RIS_TXRIS))
        {
            NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;//reset
        }
        while(1);
    }
    else if (CMD==0x31)//data report rx
    {
        sendUImessage("Channel % value is %\r\n", Channel, proc_data[0]);
        RX_MSG.VALID=0;
    }
    else if (CMD==0x30)//data request rx
    {
        sendUImessage("Get command RX\r\n", 0, 0);
        switch (Channel)
        {
            uint8_t ID[1];
        case 1:
            ID[0] = (PWM1_2_CMPB_R / 1023) * 100;
            sendRS485(DST_ADD, 0x31, 1, 1, ID, 0); //send data value of requested channel
            RX_MSG.VALID = 0;
            break;
        case 2:
            ID[0] = (PWM1_3_CMPA_R / 1023) * 100;
            sendRS485(DST_ADD, 0x31, 2, 1, ID, 0); //send data value of requested channel
            RX_MSG.VALID = 0;
            break;
        case 3:
            ID[0] = (PWM1_3_CMPB_R / 1023) * 100;
            sendRS485(DST_ADD, 0x31, 3, 1, ID, 0); //send data value of requested channel
            RX_MSG.VALID = 0;
            break;
        case 4:
            if(BUTTON==1)
            {
                ID[0]=1;
            }
            else ID[0]=0;
            sendRS485(DST_ADD, 0x31, 4, 1, ID, 0); //send data value of requested channel
            RX_MSG.VALID = 0;
            break;

        }
    }
    else if (CMD==0x03)//square rx
    {
        uint16_t Time1;
        // uint8_t Value1;
        //uint8_t Value2;
        uint16_t Time2;
        uint16_t Cycles;
        uint16_t MASK = 0x0000;
        //data[0] = Time1 & 0xFF;
        //data[1] = ((Time1 & 0xFF00) >> 8);
        Time1 = proc_data[0];
        Time1 = 8 << (MASK | proc_data[1]);
        MASK = 0x0000;
        // data[2] = Time2 & 0xFF;
        //data[3] = ((Time2 & 0xFF00) >> 8);
        Time2 = proc_data[2];
        Time2 = 8 << (MASK | proc_data[3]);
        MASK = 0x0000;
        // data[4] = Cycles & 0xFF;
        // data[5] = ((Cycles & 0xFF00) >> 8);
        Cycles = proc_data[4];
        Cycles = 8 << (MASK | proc_data[5]);
        MASK = 0x0000;
        // data[6] = Value1;
        //Value1=proc_data[6];
        // data[7] = Value2;
        //Value2=proc_data[7];
        PIN_OUT = 1;
        action_var.command = 0x03;
        action_var.time1 = Time1;
        action_var.time2 = Time2;
        action_var.count = Cycles;
        action_var.valid = 1;
        action_var.phase = 0;
        action_var.delta_T = 0;
        sendUImessage("Square command RX\r\n", 0, 0);
        RX_MSG.VALID = 0;
    }
    else if (CMD==0x02)//pulse rx
    {
        uint16_t Duration;
        uint16_t MASK = 0x0000;
        //uint8_t Value;
        Duration = proc_data[0]; //= Duration & 0xFF;
        Duration = 8 << (MASK | proc_data[1]); // = ((Duration & 0xFF00) >> 8);
        // Value = proc_data[2];
        PIN_OUT = 1;
        action_var.command = 0x02;
        action_var.time1 = Duration;
        action_var.valid = 1;
        action_var.phase = 0;
        action_var.delta_T = 0;
        sendUImessage("Pulse command RX\r\n", 0, 0);
        RX_MSG.VALID = 0;
    }
    else if (CMD==0x7A)//set new add rx
    {
        writeeeprom(0,proc_data[0]);
        MY_ADD = readeeprom(0);
        sendUImessage("My New Address is %\r\n", MY_ADD, 0);
        RX_MSG.VALID=0;
    }
    else if (CMD==0x48)//RGB rX
    {
        uint8_t RED_R =  proc_data[0];
        uint16_t RED_V = (RED_R / 100) * 1023; //range from 0 to 100 for brightness
        uint8_t BLUE_R =  proc_data[1];
        uint16_t BLUE_V = (BLUE_R / 100) * 1023;
        uint8_t GREEN_R =  proc_data[2];
        uint16_t GREEN_V = (GREEN_R / 100) * 1023;
        PWM1_2_CMPB_R = RED_V; //red
        PWM1_3_CMPA_R = BLUE_V; //blue
        PWM1_3_CMPB_R = GREEN_V; //green
        sendUImessage("RGB command RX\r\n", 0, 0);
        RX_MSG.VALID=0;
    }
    else if (CMD==0x00)//set command rx
    {
        sendUImessage("set command RX\r\n", 0, 0);
        switch (Channel)
        {
        case 1:
            PWM1_2_CMPB_R= proc_data[0];
            RX_MSG.VALID=0;
            break;
        case 2:
            PWM1_3_CMPA_R=proc_data[0];
            RX_MSG.VALID=0;
            break;
        case 3:
            PWM1_3_CMPB_R=proc_data[0];
            RX_MSG.VALID=0;
            break;
        case 4:
            GPIO_PORTA_PUR_R ^= BUTTON_MASK;
            RX_MSG.VALID=0;
            break;
        }
    }

}

void sendRS485(uint8_t DST_ADD, uint8_t CMD, uint8_t Channel, uint8_t SIZE, uint8_t data[], bool locack)
{

    while (MSG_Q[TX_index].VALID) // searching for invalid slot in MSG_Q
    {
        TX_index = (TX_index + 1) & 255;
    }
    if (!MSG_Q[TX_index].VALID)
    {
        uint8_t cs = 0;
        int i;
        if (locack)
        {
            MSG_Q[TX_index].COMMAND = CMD | 0x80;
        }
        else
            MSG_Q[TX_index].COMMAND = CMD;
        MSG_Q[TX_index].SRC_ADDRESS = MY_ADD;
        SEQ_ID = (SEQ_ID + 1) & 255;
        MSG_Q[TX_index].SEQ_ID = SEQ_ID;
        MSG_Q[TX_index].SIZE = SIZE;
        MSG_Q[TX_index].DST_ADDRESS = DST_ADD;
        MSG_Q[TX_index].CHANNEL = Channel;
        cs = MSG_Q[TX_index].DST_ADDRESS + MSG_Q[TX_index].SRC_ADDRESS
                + MSG_Q[TX_index].COMMAND + MSG_Q[TX_index].SEQ_ID+MSG_Q[TX_index].CHANNEL
                + MSG_Q[TX_index].SIZE;
        for (i = 0; i < SIZE; i++)
        {
            MSG_Q[TX_index].data[i] = data[i];
            cs += MSG_Q[TX_index].data[i];
        }
        cs = cs % 256;
        MSG_Q[TX_index].CHECKSUM = ~(cs);
        MSG_Q[TX_index].VALID = 1;
        MSG_Q[TX_index].TX_count = 0;
        sendUImessage(" Queuing Msg %\r\n", MSG_Q[TX_index].SEQ_ID, 0);
    }
    //need to add time to TX when setting up timer interrupt
    if (!(UART1_RIS_R & UART_RIS_TXRIS))
    {
        sendRS485Byte();
    }

}

void sendRS485Byte()
{
    bool cs_check=0;
    if (!MSGinProgress)
    {
        int count = 0 ;
        while (!MSG_Q[TX_index].VALID && time_to_TX==0 && count!=255) //need to add timer later
        {
            TX_index = (TX_index + 1) & 255;
            count++;
        }
        TX_phase = 0;
        MSGinProgress = 1; //carrier sense here???

    }
    if(carrier_sense)
    {
        if(test_done)
        {
            test_done=false;
            if(!busy)
            {
                cs_check=1;
            }
        }
        else
        {
            busy=false;
            test_done=false;
            if(test_cs==0)
            {
                test_cs=CS_PERIOD;
            }
        }
    }
    else
    {
        cs_check=1;
    }
    if (MSGinProgress && cs_check && MSG_Q[TX_index].VALID) // carrier sense check add in later
    {
        DE = 1;   //IC enable
        switch (TX_phase)
        {
        case 0:
            UART1_LCRH_R &= ~UART_LCRH_EPS; //set parity bit before sending ADD
            UART1_DR_R = MSG_Q[TX_index].DST_ADDRESS;
            TX_phase++;
            break;
        case 1:
            UART1_LCRH_R |= UART_LCRH_EPS; // clear parity bit for the rest of message
            UART1_DR_R = MSG_Q[TX_index].SRC_ADDRESS;
            TX_phase++;
            break;
        case 2:
            UART1_DR_R = MSG_Q[TX_index].SEQ_ID;
            TX_phase++;
            break;
        case 3:
            UART1_DR_R = MSG_Q[TX_index].COMMAND;
            TX_phase++;
            break;
        case 4:
            UART1_DR_R = MSG_Q[TX_index].CHANNEL;
            TX_phase++;
            break;
        case 5:
            UART1_DR_R = MSG_Q[TX_index].SIZE;
            TX_phase++;
            break;
        default:
            if (TX_phase < 6 + MSG_Q[TX_index].SIZE && TX_phase > 5)
            {
                UART1_DR_R = MSG_Q[TX_index].data[TX_phase - 6];
                TX_phase++;
                break;
            }
            else if (TX_phase == 6 + MSG_Q[TX_index].SIZE
                    && TX_phase < 8 + MSG_Q[TX_index].SIZE)
            {
                UART1_DR_R = MSG_Q[TX_index].CHECKSUM;
                TX_phase++;
                TX_LED = 1;
                TX_LED_timeout = MAX_TIMEOUT;         //change period later
                break;
            }
            else
            {
                if (!(UART1_DR_R = MSG_Q[TX_index].COMMAND&0x80))
                {
                    MSG_Q[TX_index].VALID = 0; //when done mark this message as invalid again
                }
                else if ((UART1_DR_R = MSG_Q[TX_index].COMMAND&0x80))
                {
                    if (MSG_Q[TX_index].TX_count < MAX_COUNT) // need more condition here?
                    {
                        sendUImessage("Transmitting message %, attempt %\r\n",
                                      MSG_Q[TX_index].SEQ_ID,
                                      MSG_Q[TX_index].TX_count + 1);
                        MSG_Q[TX_index].TX_count++;
                        if (!random)
                        {
                            time_to_TX = min_backoff_time
                                    + (pow(2, MSG_Q[TX_index].TX_count) * T);
                        }
                        else if (random)
                        {
                            srand(MY_ADD*MAX_COUNT);
                            uint8_t max_rand = (pow(2, MSG_Q[TX_index].TX_count)
                                    * T);
                            time_to_TX = min_backoff_time + (rand() % max_rand); //try some better random generator later
                        }
                    }
                    else if (MSG_Q[TX_index].TX_count == MAX_COUNT)
                    {
                        MSG_Q[TX_index].VALID = 0;
                        TX_LED_timeout = 0;
                        TX_LED = 1;
                        sendUImessage("Error sending message %\r\n",
                                      MSG_Q[TX_index].SEQ_ID, 0);
                    }

                }
                _delay_cycles(9);
                MSGinProgress = 0;         //done transmitting message
                DE = 0;         //turn on RS485 IC enable
                break;
            }
        }
    }
}
