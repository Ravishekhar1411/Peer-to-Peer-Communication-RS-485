#include <stdint.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC      : TM4C123GH6PM
// System Clock   : 40 MHz

// Hardware configuration:
// GREEN LED:
// PF3 drives an NPN transistor that powers the green LED

//-----------------------------------------------------------------------------
#define RED_LED        (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define BLUE_LED       (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define DE             (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))
#define RED_BOARD      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4)))
#define GREEN_BOARD    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 5*4)))

#define DEBUG_UART0_MSG

#define MAX_CHARS 80
#define MAX_DATA  20
#define MAX_MSGS 25
#define MAX_RETRIES 5
#define broadcast_add 255
#define T0 100
#define T 50
char str[MAX_CHARS + 1];
char field_type[20];
char debug_str[50];

uint32_t VALID_BIT_SET;

//DEVICE ADDRESS IS SET TO 2
uint8_t my_add = 2;
uint16_t timeout;
uint8_t count;
uint16_t RETRANS_TIMEOUT[MAX_MSGS];


//May be deleted
uint8_t Total_fields;
uint16_t k;
uint8_t field_position[20];


int8_t i;

uint8_t address;
uint8_t new_address;
uint8_t channel;

uint8_t value[25][10];
uint8_t CS_ENABLE;
uint8_t RANDOM_ENABLE;
uint8_t ACK_ON;
uint8_t DATA[MAX_MSGS][MAX_DATA];
uint8_t RX_DATA[MAX_MSGS];
uint8_t currentindex;
uint8_t TX_phase = 0;
uint8_t retranscount = 0;
uint8_t RXADDDATA;
uint8_t RXdata;
uint8_t j;
uint16_t D = 0;
uint8_t s;
uint8_t d = 0;
uint8_t receivephase = 0;
uint8_t l;
uint8_t i_for;



bool Inprogress;
uint8_t N = 0;
uint16_t old_receivephase;
uint16_t deadlock_timeout;
char str1[200];
uint8_t a;
uint8_t PUSH_BUTTON;
uint16_t retrans_timeout;

// Just for fix bugs
uint8_t data_index;
uint8_t checksum_flag;
uint8_t TX_ATTEMP_MAX = 5;
bool random_flag =0;
bool Transmit_control_flag = 0 ;
//
uint8_t Ack_Tx_attemp_count =0 ;
uint32_t   wait_time_out = 0;
uint8_t T_0 = 100;



char STR_TO_INT[8];


#ifdef DEBUG_UART0_MSG
       char STR_DEBUG[100];
#endif


typedef struct message_data
{
    uint8_t DEST_ADDRESS;           // 1.Message transmitted to Destination address(DEST_ADDRESS)
    uint8_t SENDER_ADDRESS;         // 2.Here Sender Address same as
    uint8_t CURRENT_SEQENCE_ID;     // 3.At Sender part index of message buffer of MAX_MSGS
    uint8_t COMMAND_MSG;            // 4.Command for Destination node
    uint8_t CHANNEL_ADDR_OF_DEST;   // 5.Channel at Destination address(DEST_ADDRESS)
    uint8_t MESSAGE_SIZE;           // 6.Total DATA_MSG has been transmitted
    uint8_t *DATA_MSG;              // 7.For this Sequence ID; Data to be sent, Max Data can be sent MAX_MSGS
    uint8_t CHECK_SUM;              // 8.Check sum
    bool    VALID_BIT;              // Valid bit for message is trasmitted or not
    bool    ACK_FROM_DEST_ADDR;     // It is up to user to Receive Ack from destination or not by enable this bit
    uint8_t RETRANS_COUNT;          // Total transmition count if no ACK received from Destination address(DEST_ADDRESS)
}   uart_message_data;

typedef struct device_buffer
{
    struct uart_message_data *message_buffer;
    uint8_t CURRENT_INDEX_BUFFER;
    uint8_t DEVICE_ADDRESS;
    bool    Send_packet_uart_command;
    bool    Receive_packet_uart_command;
    void(*_fn)();
    void(*_Sel_fn)();                // This function for selection of transmit_uart1 or receive_uart1
}   DEVICE_DATA_BUFFER;

uart_message_data ADDRESS_1_DATA[MAX_MSGS];
DEVICE_DATA_BUFFER DEVICE_DETAIL;

void Device_initialization()
{
    uint8_t i;
    DEVICE_DETAIL.DEVICE_ADDRESS = 2;          // Default address set to 2
    putsUart0("<<<DEVICE ADDRESS IS 2>>>>>>>>>> \r\n");
    DEVICE_DETAIL.CURRENT_INDEX_BUFFER = 0;
    DEVICE_DETAIL.message_buffer = ADDRESS_1_DATA;

    for( i = 0;i< MAX_MSGS;i++)
        ADDRESS_1_DATA[i].SENDER_ADDRESS = DEVICE_DETAIL.DEVICE_ADDRESS;
}

void sendpacket(uint8_t dest_add, uint8_t command, uint8_t channel, uint8_t size, uint8_t data[])
{

    uint16_t ADDDATA = 0;

    if (ADDRESS_1_DATA[DEVICE_DETAIL.CURRENT_INDEX_BUFFER].VALID_BIT == false)
    {
        ADDRESS_1_DATA[DEVICE_DETAIL.CURRENT_INDEX_BUFFER].DEST_ADDRESS = dest_add;
        ADDRESS_1_DATA[DEVICE_DETAIL.CURRENT_INDEX_BUFFER].CURRENT_SEQENCE_ID = DEVICE_DETAIL.CURRENT_INDEX_BUFFER;

        ADDRESS_1_DATA[DEVICE_DETAIL.CURRENT_INDEX_BUFFER].MESSAGE_SIZE = size;
        ADDRESS_1_DATA[DEVICE_DETAIL.CURRENT_INDEX_BUFFER].CHANNEL_ADDR_OF_DEST = channel;
        ADDRESS_1_DATA[DEVICE_DETAIL.CURRENT_INDEX_BUFFER].ACK_FROM_DEST_ADDR = false;

        if ( ACK_ON == true)
        {
          //  ADDRESS_1_DATA[DEVICE_DETAIL.CURRENT_INDEX_BUFFER].ACK_FROM_DEST_ADDR
            ADDRESS_1_DATA[DEVICE_DETAIL.CURRENT_INDEX_BUFFER].COMMAND_MSG = command | 0x80;
            ADDRESS_1_DATA[DEVICE_DETAIL.CURRENT_INDEX_BUFFER].ACK_FROM_DEST_ADDR = true;
        }
         else
            ADDRESS_1_DATA[DEVICE_DETAIL.CURRENT_INDEX_BUFFER].COMMAND_MSG = command;

        ADDRESS_1_DATA[DEVICE_DETAIL.CURRENT_INDEX_BUFFER].DATA_MSG = DATA[DEVICE_DETAIL.CURRENT_INDEX_BUFFER];

        for (d = 0; d < ADDRESS_1_DATA[DEVICE_DETAIL.CURRENT_INDEX_BUFFER].MESSAGE_SIZE; d++) {
            ADDRESS_1_DATA[DEVICE_DETAIL.CURRENT_INDEX_BUFFER].DATA_MSG[d] = data[d];
            ADDDATA += data[d];
        }

        ADDRESS_1_DATA[DEVICE_DETAIL.CURRENT_INDEX_BUFFER].CHECK_SUM = (~
                 (
                           ADDRESS_1_DATA[DEVICE_DETAIL.CURRENT_INDEX_BUFFER].DEST_ADDRESS             // Dest .Addr
                         + DEVICE_DETAIL.DEVICE_ADDRESS                                                // This Divice node address
                         + ADDRESS_1_DATA[DEVICE_DETAIL.CURRENT_INDEX_BUFFER].CURRENT_SEQENCE_ID       // Sequnce Id
                         + ADDRESS_1_DATA[DEVICE_DETAIL.CURRENT_INDEX_BUFFER].COMMAND_MSG              // Command
                         + ADDRESS_1_DATA[DEVICE_DETAIL.CURRENT_INDEX_BUFFER].MESSAGE_SIZE             // Size of message
                         + ADDDATA                                                                     // All data Sum
                         + ADDRESS_1_DATA[DEVICE_DETAIL.CURRENT_INDEX_BUFFER].CHANNEL_ADDR_OF_DEST     // Channel
                 ));

        ADDRESS_1_DATA[DEVICE_DETAIL.CURRENT_INDEX_BUFFER].RETRANS_COUNT = 0;
        ADDRESS_1_DATA[DEVICE_DETAIL.CURRENT_INDEX_BUFFER].VALID_BIT = true;                           //validating current msg

        VALID_BIT_SET |= 1 << DEVICE_DETAIL.CURRENT_INDEX_BUFFER;

        if(ADDRESS_1_DATA[DEVICE_DETAIL.CURRENT_INDEX_BUFFER].ACK_FROM_DEST_ADDR == false & !ACK_ON)
        {
           if(DEVICE_DETAIL.CURRENT_INDEX_BUFFER<MAX_MSGS)
               DEVICE_DETAIL.CURRENT_INDEX_BUFFER++;
        }
    }
}


// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port F and A peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOE;

    // Configure  green LED
    GPIO_PORTF_DIR_R  |= 0x08;  // make bit 3 an output
    GPIO_PORTF_DR2R_R |= 0x08;  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R  |= 0x08;  // enable LED

    // Configure blue LED
    GPIO_PORTF_DIR_R  |= 0x04;  // make bit 2 an output
    GPIO_PORTF_DR2R_R |= 0x04;  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R  |= 0x04;  // enable LED

    // Configure red LED
    GPIO_PORTF_DIR_R  |= 0x02;  // make bit 2 an output
    GPIO_PORTF_DR2R_R |= 0x02;  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R  |= 0x02;  // enable LED

    // Configure  board LEDs
    GPIO_PORTE_DIR_R  |= 0x30;  // make bit 4 an output
    GPIO_PORTE_DR2R_R |= 0x30;  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTE_DEN_R  |= 0x30;  // enable LED

    // Configure UART0 pins
    SYSCTL_RCGCUART_R  |= SYSCTL_RCGCUART_R0;       // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R   |= 3;                        // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                        // default, added for clarity
    GPIO_PORTA_PCTL_R   = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R  = 0;                                // turn-off UART0 to allow safe programming
    UART0_CC_R   = UART_CC_CS_SYSCLK;                // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R  = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // Configure UART1 pins
    SYSCTL_RCGCUART_R  |= SYSCTL_RCGCUART_R1;           // turn-on UART1
    GPIO_PORTC_DIR_R   |= 0x40;
    GPIO_PORTC_DEN_R   |= 0x70;                         // default, added for clarity
    GPIO_PORTC_AFSEL_R |= 0x30;                         // default, added for clarity
    GPIO_PORTC_PCTL_R  |= GPIO_PCTL_PC4_U1RX | GPIO_PCTL_PC5_U1TX;

    // Configure UART1 to 38400 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART1_CTL_R  = 0;                                     // turn-off UART0 to allow safe programming
    UART1_CC_R   = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
    UART1_IBRD_R = 65;                                    // r = 40 MHz / (Nx38400Hz), set floor(r)=65, where N=16
    UART1_FBRD_R = 7;                                     // round(fract(r)*64)=6
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;      // configure for 8N1 w/ 16-level FIFO
    UART1_CTL_R  = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // Configure Timer 1 as the time base
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;            // turn-on timer
    TIMER1_CTL_R       &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R       = TIMER_CFG_32_BIT_TIMER;          // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R      = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R     = 0x9C40;                          // set load value to 40e3 for 1 kHz interrupt rate
    TIMER1_IMR_R       = TIMER_IMR_TATOIM;                // turn-on interrupts
    NVIC_EN0_R         |= 1 << (INT_TIMER1A - 16);        // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R       |= TIMER_CTL_TAEN;                 // turn-on timer

}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");           // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");           // 6
    __asm("             CBZ  R1, WMS_DONE1");    // 5+1*3
    __asm("             NOP");                   // 5
    __asm("             NOP");                   // 5
    __asm("             B    WMS_LOOP1");        // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");           // 1
    __asm("             CBZ  R0, WMS_DONE0");    // 1
    __asm("             NOP");                   // 1
    __asm("             B    WMS_LOOP0");        // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");
}

void on_booard_led_blink(char * led_name)
{
            if(!strcmp(led_name,"GREEN_LED"))
            {
                GREEN_LED = 1;
                waitMicrosecond(250000);
                GREEN_LED = 0;
            }
            else if(!strcmp(led_name,"RED_LED"))
            {
                RED_LED = 1;
                waitMicrosecond(250000);
                RED_LED = 0;
            }
            else if(!strcmp(led_name,"BLUE_LED"))
            {
                BLUE_LED = 1;
                waitMicrosecond(250000);
                BLUE_LED = 0;
            }
}
// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}

// Blocking function that returns with serial data once the buffer is not empty
uint16_t getcUart1() {
    if (!(UART1_FR_R & UART_FR_RXFE))
        return (UART1_DR_R);
}

void PutsUart0_int(char *str_head, uint8_t *in )
// function convert uint8_t into string i.e. 1234 for in Interrupt service routine
{
    char str_dis[7];
    uint8_t temp  = (*in);
    ltoa(temp, str_dis,10);
    putsUart0(str_head);
    putsUart0(" : ");
    putsUart0(str_dis);
    putsUart0(" \n\r ");
}

// Blocking function that returns with serial data once the buffer is not empty
void getsUart0()
{
    putsUart0("getsUart0() in \r\n");
    putsUart0("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\r\n");
    for (s = 0; s < MAX_CHARS; s++) {
        str[s] = 0;
    }
    count = 0;
    char c;
    while (count <= 81) {
        c = getcUart0();
        putcUart0(c);
        str[count] = c;
        if (str[count] == 8)                       //   if backspace
                {
            if (count > 0) {
                count = count - 2;
            }
        } else if (str[count] == 0x20)             //  if space
                {
            str[count] = NULL;
        } else if (str[count] == 13)              //  if enter
                {
            str[count++] = 0;
            putsUart0("\n\r");
            break;
        }
        count++;
    }

    putsUart0("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\r\n");
}

void delimiters()
{

    k = 0;
    for (k = 0; k < count; k++)        // FOR 81 CHARACTERS
            {
        if ((str[k] >= 1 && str[k] <= 47) //converting delims to null
        || (str[k] >= 58 && str[k] <= 64) || (str[k] >= 123 && str[k] <= 127)) {
            str[k] = NULL;
        }

        if (str[k] >= 'A' && str[k] <= 'Z') //coverting uppercase letters to lowercase
                {
            str[k] = tolower(str[k]);

        }

    }

}
void Parsing()
{

    k = 0;
    Total_fields = 0;
    for (k = 0; k < 20; k++) {
        field_position[k] = 0;
        field_type[k] = 0;
    }

    for (k = 0; k < count; k++) {

        if (str[k] == 0 && str[k + 1] >= 'a' && str[k + 1] <= 'z') //if First character is NULL and then Alphabets
                {
            field_type[Total_fields] = 'a';
            field_position[Total_fields] = k + 1;
            Total_fields++;

        }

        if (str[k] >= 'a' && str[k] <= 'z' && Total_fields == 0) // if first charac is Alphabet
                {
            field_type[Total_fields] = 'a';
            field_position[Total_fields] = k;
            Total_fields++;

        }

        if (str[k] == 0 && str[k + 1] >= '0' && str[k + 1] <= '9') // if First charac is NULL and then numerics
                {
            field_type[Total_fields] = 'n';
            field_position[Total_fields] = k + 1;
            Total_fields++;

        }
        if (str[k] >= '0' && str[k] <= '9' && Total_fields == 0) // if first charac is Alphabet
                {
            field_type[Total_fields] = 'a';
            field_position[Total_fields] = k;
            Total_fields++;

        }

    }

}

char* getstring(uint8_t a)
{
    return (&str[field_position[a]]);
}
uint8_t getnumber(uint8_t a) {
    return (atoi(&str[field_position[a]]));
}
bool iscommand(char* strcmd, uint8_t (min_Args)) {

    return ((strcmp(&str[field_position[0]], strcmd)) == 0
            && (Total_fields > min_Args));
}

void debug_uart(char * str,uint8_t val)
{
    putsUart0("\r\n");
    memset(debug_str,0,50);
    putsUart0(str);
    sprintf(debug_str, " :  %u", val);
    putsUart0(debug_str);
}

void strcommand()
{


    if (iscommand("set", 3)) {

        address = getnumber(1);
        channel = getnumber(2);
        value[0][0] = getnumber(3);

        putsUart0("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\r\n");
        putsUart0("\n");
        PutsUart0_int("address ",&address);
        PutsUart0_int("channel ",&channel);
        PutsUart0_int("value[0][0] ",&value[0][0]);
        putsUart0("\n");
        putsUart0("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\r\n");

        sendpacket(address, 0x00, channel, 0x01, value[0]);
    }

    else if (iscommand("rgb", 4)) {

        address = getnumber(1);
        channel = getnumber(2);
        value[0][0] = getnumber(3);
        value[0][1] = getnumber(4);
        value[0][2] = getnumber(5);
        sendpacket(address, 0x29, channel, 3, value[0]);
    }

    else if (iscommand("cs", 1)) {

        char* x = getstring(1);

        if (strcmp(x, "on") == 0) {
            putsUart0("CS is on \r\n");
            CS_ENABLE = true;
        } else if (strcmp(x, "off") == 0) {
            putsUart0("CS is off \r\n");
            CS_ENABLE = false;
        } else
            putsUart0("command error");
    }

    else if (iscommand("random", 1)) {

        char* x = getstring(1);

        if (strcmp(x, "on") == 0) {
            putsUart0("Random is on \r\n");
            RANDOM_ENABLE = true;
        }
        else if (strcmp(x, "off") == 0) {
            putsUart0("Random is off \r\n");
            RANDOM_ENABLE = false;
        } else
            putsUart0("invalid command");
    }

    else if (iscommand("get", 2)) {

        address = getnumber(1);
        channel = getnumber(2);
        sendpacket(address, 1, channel, 0, 0);
    }

    else if (iscommand("reset", 1)) {

        putsUart0("Ready");
    }

    else if (iscommand("sa", 2)) {

        address = getnumber(1);
        new_address = getnumber(2);
        putsUart0("set A C V");
    }

    else if (iscommand("poll", 0)) {

        putsUart0("poll\n");
    }

    else if (iscommand("ack", 1))
    {
        char* x = getstring(1);
        if (strcmp(x, "on") == 0)
        {
            ACK_ON = true;
        }
        if (strcmp(x, "off") == 0)
        {
            ACK_ON = false;
        }
    }

    else if (iscommand("reset", 1)) {
        address = getnumber(1);
    }


}

void Timeout_send_ack()
{

    if(RANDOM_ENABLE == true )
    {
        Ack_Tx_attemp_count = 4;
        wait_time_out = T_0*(1<<Ack_Tx_attemp_count); // 800ms

      //  random_flag = true;

#ifdef DEBUG_UART0_MSG
        putsUart0("wait_time_out : ");
        ltoa(wait_time_out, STR_TO_INT,10);
        putsUart0(STR_TO_INT);
        putsUart0("\r\n");

        putsUart0("| RANDOM_ENABLE == true \n\r");
#endif

    }
    else if(!RANDOM_ENABLE & ADDRESS_1_DATA[currentindex].RETRANS_COUNT < TX_ATTEMP_MAX)
    {
        Ack_Tx_attemp_count++;
        ADDRESS_1_DATA[currentindex].RETRANS_COUNT++;
        wait_time_out = T_0*(1<<Ack_Tx_attemp_count);

#ifdef DEBUG_UART0_MSG
        sprintf(STR_DEBUG," Timeout_send_ack() > wait_time_out : %u  Ack_Tx_attemp_count : %u| RANDOM_ENABLE == false \r\n",wait_time_out,Ack_Tx_attemp_count);
        putsUart0(STR_DEBUG);
#endif

    }
    else if(!RANDOM_ENABLE & ADDRESS_1_DATA[currentindex].RETRANS_COUNT >= TX_ATTEMP_MAX)
    {

#ifdef DEBUG_UART0_MSG
        sprintf(STR_DEBUG," Timeout_send_ack() > Failed to get ACK | SEQ ID : %u \r\n",ADDRESS_1_DATA[currentindex].CURRENT_SEQENCE_ID);
        putsUart0(STR_DEBUG);
#endif

    }
}

void transmited_packets()
{

        putsUart0("********************************\r\n");
        putsUart0("TRANSMITTED DATA FROM ");
        PutsUart0_int(" NODE ",&ADDRESS_1_DATA[currentindex].SENDER_ADDRESS);
        putsUart0(" >>>> ");
        PutsUart0_int(" TRANSMIT NODE ",&ADDRESS_1_DATA[currentindex].DEST_ADDRESS);
        putsUart0("--------------------------------\r\n");
        PutsUart0_int(" TRANSMIT NODE ",&ADDRESS_1_DATA[currentindex].DEST_ADDRESS);
        PutsUart0_int("SOURCE ADDRESS ",&ADDRESS_1_DATA[currentindex].SENDER_ADDRESS);
        PutsUart0_int("SEQ ID         ",&ADDRESS_1_DATA[currentindex].CURRENT_SEQENCE_ID);
        PutsUart0_int("COMMAND        ",&ADDRESS_1_DATA[currentindex].COMMAND_MSG);
        PutsUart0_int("CHANNEL        ",&ADDRESS_1_DATA[currentindex].CHANNEL_ADDR_OF_DEST);
        PutsUart0_int("SIZE           ",&ADDRESS_1_DATA[currentindex].MESSAGE_SIZE);

        for(i_for =0; i_for < ADDRESS_1_DATA[currentindex].MESSAGE_SIZE;i_for++)
        {
        PutsUart0_int("DATA           ",&ADDRESS_1_DATA[currentindex].DATA_MSG[i_for]);
        }

        PutsUart0_int("CHECKSUM       ",&ADDRESS_1_DATA[currentindex].CHECK_SUM);
        PutsUart0_int("ACK SET OR NOT ",&ADDRESS_1_DATA[currentindex].ACK_FROM_DEST_ADDR);
        PutsUart0_int("VALID BIT SET  ",&ADDRESS_1_DATA[currentindex].VALID_BIT);

        putsUart0("********************************\r\n");
        putsUart0("\r\n");
}


void transmit_uart1()
{

        if (!Inprogress)
        {
            for (i = 0; i < MAX_MSGS; i++)
            {
                if ((ADDRESS_1_DATA[i].VALID_BIT == true))
                {
                    currentindex = i;
                    TX_phase = 0;
                    Inprogress = true;
                    break;
                }
            }
        }

        if (Inprogress == true)
        {
            DE = 1;
            if (TX_phase == 0) {

                if ((UART1_FR_R & UART_FR_BUSY) == 0 && (!CS_ENABLE || receivephase == 0)) {
                    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN | UART_LCRH_PEN | UART_LCRH_SPS; //9TH BIT SET (DEST ADD)
                    UART1_DR_R = ADDRESS_1_DATA[currentindex].DEST_ADDRESS;
                    TX_phase++;
                }
            }

            if (TX_phase == 1)
            {
                    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN | UART_LCRH_EPS | UART_LCRH_PEN | UART_LCRH_SPS; //9TH BIT CLEAR(SOURCE ADD)

                    UART1_DR_R = ADDRESS_1_DATA[currentindex].SENDER_ADDRESS;                // Sender node address
                    TX_phase++;

                    UART1_DR_R = ADDRESS_1_DATA[currentindex].CURRENT_SEQENCE_ID;            // Memory buffer SEQ ID
                    TX_phase++;

                    UART1_DR_R = ADDRESS_1_DATA[currentindex].COMMAND_MSG;                   // Command for Transmitter
                    TX_phase++;

                    UART1_DR_R = ADDRESS_1_DATA[currentindex].CHANNEL_ADDR_OF_DEST;          // Channel to set
                    TX_phase++;

                    UART1_DR_R = ADDRESS_1_DATA[currentindex].MESSAGE_SIZE;                  // Size of message
                    TX_phase++;

                    for (j = 0; j < ADDRESS_1_DATA[currentindex].MESSAGE_SIZE; j++)
                    {
                        UART1_DR_R =(uint32_t)ADDRESS_1_DATA[currentindex].DATA_MSG[j];       // Total data to be sent as per Size
                        TX_phase++;
                    }

                    UART1_DR_R = ADDRESS_1_DATA[currentindex].CHECK_SUM;                     // Checksum of message
                    TX_phase++;


            }
        }

        if (TX_phase == ADDRESS_1_DATA[currentindex].MESSAGE_SIZE + 7)
          {

              Inprogress = false;
              TX_phase = 0;
              transmited_packets();
              if (ADDRESS_1_DATA[currentindex].ACK_FROM_DEST_ADDR == false)
              {
                  ADDRESS_1_DATA[currentindex].VALID_BIT = false;
              }
              else
              {
                  Transmit_control_flag = true ;
                  DEVICE_DETAIL._fn= &Timeout_send_ack;
                  DEVICE_DETAIL._fn();

              }
          }

}

void print_received_packets()
{
#ifdef DEBUG_UART0_MSG

        putsUart0("********************************\r\n");

        PutsUart0_int("RECEIVED DATA FROM ",&RX_DATA[1]);
        putsUart0("--------------------------------\r\n");
        PutsUart0_int(" TRANSMIT NODE  ",&RX_DATA[0]);
        PutsUart0_int("SOURCE ADDRESS ",&RX_DATA[1]);
        PutsUart0_int("SEQ ID         ",&RX_DATA[2]);
        PutsUart0_int("COMMAND        ",&RX_DATA[3]);
        PutsUart0_int("CHANNEL        ",&RX_DATA[4]);
        PutsUart0_int("SIZE           ",&RX_DATA[5]);

        for(i_for =0; i_for < RX_DATA[5];i_for++)
        {
        PutsUart0_int("DATA           ",&RX_DATA[6+i_for]);
        }

        PutsUart0_int("CHECKSUM       ",&RX_DATA[receivephase]);

        putsUart0("********************************\r\n");
        putsUart0("\r\n");
#endif
}

void process_received_uart1_packet()
{
    // Make a function besed of function pointer for Respective command
    uint8_t RECEIVED_CMD = RX_DATA[3];

    //checksum check
     timeout = 100;
     checksum_flag =0 ;

     // Sender command for RGB Command
     if((RECEIVED_CMD & 0x7F) == 0x48)
     {

#ifdef DEBUG_UART0_MSG
         putsUart0("\rRGB received \n\r");
#endif

     }
    // Sender command for receiver node for ACK
    if((RECEIVED_CMD & 0x80) ==0x80)
    {

#ifdef DEBUG_UART0_MSG
        putsUart0("\r SENDER ");
        PutsUart0_int(" NODE : ",&RX_DATA[2]);
        putsUart0(" DEMANDED ACK  \n\r");
#endif
        sendpacket(RX_DATA[1], 0x70, RX_DATA[4],1,(RX_DATA+2));
    }

    // Sender ACK from receiver
    if(RECEIVED_CMD  & 0x70)
    {
        // This indicate successful reception of ACK from Receiver
         if(RX_DATA[6] == ADDRESS_1_DATA[currentindex].CURRENT_SEQENCE_ID)
         {
             ADDRESS_1_DATA[currentindex].VALID_BIT = false;
             ADDRESS_1_DATA[currentindex].ACK_FROM_DEST_ADDR = false;
             DEVICE_DETAIL.CURRENT_INDEX_BUFFER++;
         }


         Transmit_control_flag = false;
         wait_time_out = 0;
         DEVICE_DETAIL._fn = NULL;

#ifdef DEBUG_UART0_MSG
         putsUart0("\r ACK RECEIVED FROM ");
         PutsUart0_int(" NODE :",&RX_DATA[1]);
#endif
         RECEIVED_CMD = 0;

    }
}

void Receive_uart1()
{

    checksum_flag = 0;

    if(DEVICE_DETAIL._fn)
    {
        wait_time_out--;
        if(wait_time_out<=0)
        {
            DEVICE_DETAIL._fn = NULL;

#ifdef DEBUG_UART0_MSG
       putsUart0("\r wait_time_out Failed to get ACK \n\r");
#endif
       Transmit_control_flag = 0;
       ADDRESS_1_DATA[currentindex].VALID_BIT = false;
        }
    }

        if ((UART1_FR_R & UART_FR_BUSY) == 0) {
            DE = 0;
            if (TX_phase == 0) {

                if ((UART1_FR_R & UART_FR_RXFE) == 0) {
                    D = UART1_DR_R;
                    RXdata = D & 0xFFF;
                    if (receivephase == 0) {
                        if (RXdata == DEVICE_DETAIL.DEVICE_ADDRESS) {     //&& (D & 0x200) == 0x200) {
                            RXADDDATA = 0;
                            RX_DATA[receivephase] = RXdata;
                            RXADDDATA += RX_DATA[receivephase]; //DESTINATION ADDRESS(PHASE 0)
                            receivephase++;
                        }
                    }

                    else if (receivephase == 1) {
                        RX_DATA[receivephase] = RXdata;  //SOURCE  ADDRESSS(PHASE 1)
                        RXADDDATA += RX_DATA[receivephase];
                        receivephase++;
                    }

                    else if (receivephase == 2) {
                        RX_DATA[receivephase] = RXdata;            //SEQ ID(PHASE 2)
                        RXADDDATA += RX_DATA[receivephase];
                        receivephase++;
                    }

                    else if (receivephase == 3) {
                        RX_DATA[receivephase] = RXdata;           //COMMAND(PHASE 3)
                        RXADDDATA += RX_DATA[receivephase];
                        receivephase++;
                    }

                    else if (receivephase == 4) {
                        RX_DATA[receivephase] = RXdata;           //CHANNEL(PHASE 4)
                        RXADDDATA += RX_DATA[receivephase];
                        receivephase++;
                    }

                    else if (receivephase == 5) {
                        RX_DATA[receivephase] = RXdata;              //SIZE(PHASE 5)
                        RXADDDATA += RX_DATA[receivephase];
                        data_index = RX_DATA[receivephase];
                        receivephase++;
                    }

                    else if (receivephase == 6|| data_index!=0) {                             //DATA
                        RX_DATA[receivephase] = RXdata;
                        data_index--;
                        RXADDDATA += RX_DATA[receivephase];
                        receivephase++;
                    }

                    else if (receivephase == (6 + RX_DATA[5])) {
                        RX_DATA[receivephase] = RXdata;
                        RXADDDATA = ~RXADDDATA;                   //CHECKSUM RECEIVE

#ifdef DEBUG_UART0_MSG
                        print_received_packets();
#endif


                        receivephase = 0;
                        checksum_flag =1 ;


                    }

                    if ((RXADDDATA == RX_DATA[6 + RX_DATA[5]])&&checksum_flag)
                    {
                        process_received_uart1_packet();
                        receivephase =0 ;
                    }

                }
            }

        }


}
void Timer1Isr() {

        Inprogress = false;
        if(!Transmit_control_flag)
           transmit_uart1();
        Receive_uart1();
        TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag

}

int main(void) {
// Initialize hardware


// Toggle GREEN LED every 500 millisecond
    initHw();
    on_booard_led_blink("GREEN_LED");

    Device_initialization();

 //   sprintf(STR_DEBUG," ****DEVICE  ADDRESS : %u *********\r\n",DEVICE_DETAIL.DEVICE_ADDRESS);
 //   putsUart0(STR_DEBUG);

    while (1)
    {
        getsUart0();
        delimiters();
        Parsing();
        strcommand();
    }
}
