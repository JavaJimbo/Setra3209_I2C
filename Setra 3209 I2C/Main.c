/***********************************************************************************************************
 * Setra 3209 Cal System
 * Written for PIC32MX795 using XC32 compiler V 1.30
 * Adapted from DC950 Tester Firmware
 * 
 * 04-23-18: Reading, writing to NVRAM and ERASE, RECALL, and STORE all work.
 *              Questions about CHARGE PUMP bits.
 * 04-25-18: 
 ************************************************************************************************************/

#define DIAGNOSTICS
#define true TRUE
#define false FALSE

#define CR 13
#define LF 10
#define BACKSPACE 8

#define RELAY_K3_ON() PORTSetBits(IOPORT_B, BIT_13)
#define RELAY_K2_ON() PORTSetBits(IOPORT_B, BIT_14)
#define RELAY_K1_ON() PORTSetBits(IOPORT_B, BIT_15)

#define RELAY_K3_OFF() PORTClearBits(IOPORT_B, BIT_13)
#define RELAY_K2_OFF() PORTClearBits(IOPORT_B, BIT_14)
#define RELAY_K1_OFF() PORTClearBits(IOPORT_B, BIT_15)

/** INCLUDES *******************************************************/
#include <XC.h>
#include "Delay.h"
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "Definitions.h"
#include "I2C_EEPROM_PIC32.h"

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

/** CONFIGURATION **************************************************/
#pragma config UPLLEN   = ON        // USB PLL Enabled
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier $$$$
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select
#pragma config DEBUG    = OFF


#define MAXPWM 6500

// UART FOR PC SERIAL PORT
#define HOSTuart UART2
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR
#define BUFFERSIZE 128



unsigned char HOSTRxBuffer[BUFFERSIZE];
unsigned char HOSTRxBufferFull = false;
unsigned char HOSTTxBuffer[BUFFERSIZE];
unsigned char HOSTTxBufferFull = false;

union {
	unsigned char byte[2];
	unsigned int  integer;	
} convert;
#define HighByte byte[1]
#define LowByte byte[0]

/** PRIVATE PROTOTYPES *********************************************/
void InitializeSystem(void);
unsigned char   processInputString(unsigned char *ptrBuffer);
unsigned char   executeCommand(unsigned char *ptrCommand, unsigned char *ptrValue);
unsigned char   setPWM(unsigned char *ptrPWMstring);
unsigned char   setOutput(unsigned char *ptrPin, unsigned char *ptrPinState);
unsigned char   diagnosticsPrintf(unsigned char *ptrString);
unsigned char   sendToUART(unsigned char UartID, unsigned char *ptrUARTpacket);
extern BOOL     CRCcheck(char *ptrPacket);
extern UINT16   CRCcalculate(char *ptrPacket, BOOL addCRCtoPacket);


unsigned char replyToHost(unsigned char *ptrMessage){
    if (ptrMessage == NULL) return (FALSE);
    if (strlen(ptrMessage) > BUFFERSIZE) return (FALSE);
    strcpy(HOSTTxBuffer, ptrMessage);
    CRCcalculate(HOSTTxBuffer, true);
    sendToUART(HOSTuart, HOSTTxBuffer);    
    return(TRUE);
}


#define EEDEVICE 0xA0
#define MAXBUFFER 256
unsigned char outBuffer[MAXBUFFER] = "This is exactly what I was spewing about";
unsigned char inBuffer[MAXBUFFER] = "";

unsigned char OpenPcapI2C();
unsigned char PCAPPowerOnReset();
unsigned char InitializePCAP();
unsigned char PcapWriteNVRAM(unsigned short startAddress, unsigned char *ptrData, unsigned short numBytes);
unsigned char PcapReadNVRAM(unsigned short startAddress, unsigned char *ptrData, unsigned char numBytes);
unsigned char RecallPcapNVRAM();
unsigned char StorePcapNVRAM();
unsigned char ErasePcapNVRAM();
unsigned char SendOpcode(unsigned char opcode);

#define RD 1
#define WR 0
#define WRITE_MEM 0xA0
#define READ_MEM 0x20
#define PCAP_DEVICE 0x50
#define POWER_ON_RESET 0x88
#define INITIALIZE 0x8A
#define WRITE_CONFIGURATION 0xA3
#define READ_CONFIGURATION 0x23

#define ACTIVATE_STORE 0x2D
#define STORE_NVRAM 0x96
#define ACTIVATE_RECALL 0x59
#define RECALL_NVRAM 0x99
#define ACTIVATE_ERASE 0xB8
#define ERASE_NVRAM 0x9C

#define CONFIG_ADDRESS_OFFSET 0x03C0
#define MEM_CTRL_REGISTER 0x36      // = 54 decimal
#define CHARGE_PUMP_REGISTER 0x3E   // = 62 decimnal


unsigned char SendOpcode(unsigned char opcode)
{       
    StartI2C(); // Send the Start Bit
    IdleI2C(); // Wait to complete    
    
    MasterWriteI2C(PCAP_DEVICE | WR); // Send EEPROM Device ID and WRITE Command    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) 
    {
        printf("\rNO ACK #1"); // Get ACK from EEPROM
        return false;
    }        
        
    MasterWriteI2C(opcode); // Send opcode
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) 
    {
        printf("\rNO ACK #2"); // Get ACK from EEPROM
        return false;
    }                

    StopI2C();
    IdleI2C();        
    return true;
}
  

void main(void){   
unsigned char ptrDataOut[MAXBUFFER];
unsigned short numBytes = 128;
unsigned char ptrDataIn[MAXBUFFER];        
unsigned short i = 0;    
    
    for (i = 0; i < numBytes; i++) ptrDataOut[i] = i * 5;
    
    InitializeSystem();   
   
    printf ("\rSTART UP");
    OpenPcapI2C();
    PCAPPowerOnReset(); 
    InitializePCAP();        
    
    /*
    printf ("\rERASE");
    DelayMs(100);
    ErasePcapNVRAM();
    DelayMs(100);    
    
    printf ("\rSET CHARGE PUMP BITS");
    ptrDataOut[0] = 0x00;
    ptrDataOut[1] = 0x00;    
    PcapWriteNVRAM(62 + CONFIG_ADDRESS_OFFSET, ptrDataOut, 2);   
    DelayMs(100);
    StorePcapNVRAM();
    
    while(1);
    */
    
    //printf ("\rWRITE NEW DATA AND STORE IN NVRAM");
    //PcapWriteNVRAM(0x00, ptrDataOut, numBytes);   
    //DelayMs(100);
    //StorePcapNVRAM();              
    
    printf ("\rRECALL NVRAM AND READ DATA");
    DelayMs(100);    
    RecallPcapNVRAM();
    
    DelayMs(100);    
    PcapReadNVRAM(0x00, ptrDataIn, numBytes);       
    printf("\rData: \r");    
    int j = 0;
    for (i = 0; i < numBytes; i++)    
    {
        printf("%X, ", ptrDataIn[i]);
        j++;
        if (j == 16)
        {
            j = 0;
            printf("\r");
        }        
    }
        
    while(1);
    
    while (1) {
        if (HOSTRxBufferFull) {
            HOSTRxBufferFull = false;               
            if (!CRCcheck(HOSTRxBuffer))
               replyToHost("CRC ERROR");                        
            else 
            if (!processInputString(HOSTRxBuffer))
                replyToHost("COMMAND ERROR");                                    
        }
    }
}

void InitializeSystem(void) {
    // Turn off JTAG so we get the pins back
    mJTAGPortEnable(false);

    // Set up HOST UART
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
#define SYS_FREQ 80000000
    UARTSetDataRate(HOSTuart, SYS_FREQ, 57600);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure HOST UART Interrupts
    INTEnable(INT_SOURCE_UART_TX(HOSTuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);


    PORTClearBits(IOPORT_B, BIT_12 | BIT_13 | BIT_14 | BIT_15);
    mPORTBSetPinsDigitalOut(BIT_12 | BIT_13 | BIT_14 | BIT_15);

    PORTSetPinsDigitalIn(IOPORT_B, BIT_1);
    PORTSetPinsDigitalOut(IOPORT_B, BIT_0);
    PORTSetBits(IOPORT_E, BIT_0);
    PORTSetPinsDigitalIn(IOPORT_D, BIT_15);  // Initialize FAULT signal input

    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();

}//end UserInit

unsigned char processInputString(unsigned char *ptrBuffer) {
    unsigned char *ptrCommand, *ptrValue;
    unsigned char delimiters[] = "$>[\r";

    ptrCommand = strtok(ptrBuffer, delimiters);
    if (!ptrCommand) return (false);

    ptrValue = strtok(NULL, delimiters);

    if (executeCommand(ptrCommand, ptrValue))
        return (true);
    return (false);
}

unsigned char executeCommand(unsigned char *ptrCommand, unsigned char *ptrValue) {
    if (strstr(ptrCommand, "PWM"))
        ;
    else if (strstr(ptrCommand, "TTL_IN")) {
        if (PORTReadBits(IOPORT_B, BIT_1)) replyToHost("OK");            
        else replyToHost("FAULT");        
        return true;
    }
    else if (strstr(ptrCommand, "FAULT_IN")){
        if (PORTReadBits(IOPORT_D, BIT_15)) replyToHost("HIGH");
        else replyToHost("LOW");
        return true;
    }
    else return false;
    
    replyToHost("OK");
    return true;
}


// HOST UART interrupt handler it is set at priority level 2
void __ISR(HOST_VECTOR, ipl2) IntHostUartHandler(void) {
    unsigned char ch;
    static unsigned short HOSTRxIndex = 0;

    if (HOSTbits.OERR || HOSTbits.FERR) {
        if (UARTReceivedDataIsAvailable(HOSTuart))
            ch = UARTGetDataByte(HOSTuart);
        HOSTbits.OERR = 0;
        HOSTRxIndex = 0;
    } else if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));
        if (UARTReceivedDataIsAvailable(HOSTuart)) {
            ch = toupper(UARTGetDataByte(HOSTuart));            
            if (ch == '$') HOSTRxIndex = 0;
            if (ch == LF || ch == 0);
            else if (ch == BACKSPACE) {
                while (!UARTTransmitterIsReady(HOSTuart));
                UARTSendDataByte(HOSTuart, ' ');
                while (!UARTTransmitterIsReady(HOSTuart));
                UARTSendDataByte(HOSTuart, BACKSPACE);
                if (HOSTRxIndex > 0) HOSTRxIndex--;
            } else if (ch == CR) {
                if (HOSTRxIndex < (BUFFERSIZE - 1)) {
                    HOSTRxBuffer[HOSTRxIndex] = CR;
                    HOSTRxBuffer[HOSTRxIndex + 1] = '\0'; 
                    HOSTRxBufferFull = true;
                }
                HOSTRxIndex = 0;
            }                
            else if (HOSTRxIndex < BUFFERSIZE)
                HOSTRxBuffer[HOSTRxIndex++] = ch;
        }
    }
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));
    }
}

unsigned char sendToUART(unsigned char UartID, unsigned char *ptrUARTpacket){
    short i;
    unsigned char ch;

    if (strlen(ptrUARTpacket) < BUFFERSIZE) {
        i = 0;
        do {
            ch = ptrUARTpacket[i++];
            if (!ch) break;
            while (!UARTTransmitterIsReady(UartID));
            UARTSendDataByte(UartID, ch);
        } while (i < BUFFERSIZE);
        return (true);
    }
    else return (false);
}


unsigned char OpenPcapI2C()
{
    OpenI2C(I2C_EN, 299);    
    return true;
}   

unsigned char PCAPPowerOnReset()
{
    StartI2C(); // Send the Start Bit
    IdleI2C(); // Wait to complete    

    MasterWriteI2C(PCAP_DEVICE | WR); // Send EEPROM Device ID and WRITE Command    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) printf("\rNO ACK #1"); // Get ACK from EEPROM    
        
    MasterWriteI2C(POWER_ON_RESET); // Send opcode and two high address bits
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) printf("\rNO ACK #2"); // Get ACK from EEPROM       
    
    StopI2C(); // Send the Stop condition
    IdleI2C(); // Wait to complete    
    return true;
}

unsigned char InitializePCAP()
{    
    StartI2C(); // Send the Start Bit
    IdleI2C(); // Wait to complete    

    MasterWriteI2C(PCAP_DEVICE | WR); // Send EEPROM Device ID and WRITE Command    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) 
    {
        printf("\rNO ACK #1"); // Get ACK from EEPROM
        return false;
    }
    
    MasterWriteI2C(INITIALIZE); // Send opcode and two high address bits
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT)
    {
        printf("\rNO ACK #2"); // Get ACK from EEPROM
        return false;
    }
    
    StopI2C(); // Send the Stop condition
    IdleI2C(); // Wait to complete
    return true;
}


unsigned char PcapWriteNVRAM(unsigned short startAddress, unsigned char *ptrData, unsigned short numBytes)
{
int i;    
    // WRITE DATA
    StartI2C(); // Send the Start Bit
    IdleI2C(); // Wait to complete    

    MasterWriteI2C(PCAP_DEVICE | WR); // Send EEPROM Device ID and WRITE Command    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) 
    {
        printf("\rNO ACK #1"); // Get ACK from EEPROM
        return false;
    }
    
    convert.integer = startAddress;
    MasterWriteI2C(WRITE_MEM | (convert.HighByte & 0x03)); // Send opcode ANDed with two high address bits
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) 
    {
        printf("\rNO ACK #2"); // Get ACK from EEPROM    
        return false;
    }

    MasterWriteI2C(convert.LowByte); // Send EEPROM low address byte
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT)
    {
        printf("\rNO ACK #3"); // Get ACK from EEPROM    
        return false;
    }

    // Now send block of data
    for (i = 0; i < numBytes; i++) {
        MasterWriteI2C(ptrData[i]);
        IdleI2C(); //Wait to complete
        if (I2CSTATbits.ACKSTAT)
        {
            printf("\rNO ACK #4"); // Get ACK from EEPROM 
            return false;
        }
    }    
    
    StopI2C(); // Send the Stop condition
    IdleI2C(); // Wait to complete
    return true;
}

unsigned char PcapReadNVRAM(unsigned short startAddress, unsigned char *ptrData, unsigned char numBytes) 
{
    int i;    
    
    //printf("\rSTART I2C");
    StartI2C(); // Send the Start Bit
    IdleI2C(); // Wait to complete    
    
    //printf("\rDEVICE WR");
    MasterWriteI2C(PCAP_DEVICE | WR); // Send EEPROM Device ID and WRITE Command    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) 
    {
        printf("\rNO ACK #1"); // Get ACK from EEPROM
        return false;
    }        
    
    //printf("\rREAD COMMAND");
    convert.integer = startAddress;
    MasterWriteI2C(READ_MEM | (convert.HighByte & 0x03)); // Send opcode ANDed with two high address bits
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) 
    {
        printf("\rNO ACK #2"); // Get ACK from EEPROM
        return false;
    }                

    //printf("\rSEND ADDRESS");
    MasterWriteI2C(convert.LowByte); // Send EEPROM low address byte
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) 
    {
        printf("\rNO ACK #3"); // Get ACK from EEPROM
        return false;
    }                
    
    //printf("\rRESTART");
    RestartI2C(); // Now send START sequence again:
    IdleI2C(); // Wait to complete

    //printf("\rDEVICE RD");
    MasterWriteI2C(PCAP_DEVICE | RD); // Send EEPROM Device ID and READ Command     
    IdleI2C(); // Wait to complete    
    if (I2CSTATbits.ACKSTAT) 
    {
        printf("\rNO ACK #4"); // Get ACK from EEPROM
        return false;
    }        
    
    //printf("\rREAD DATA");
    // Now receive data bytes:
    for (i = 0; i < numBytes; i++) {
        I2CCONbits.RCEN = 1;
        while (!DataRdyI2C()); // Wait for incoming data byte
        ptrData[i] = I2CRCV; // Read data byte from buffer

        if (i < numBytes - 1) {     // Master sends ACKS to EEPROM after each read
            I2CCONbits.ACKDT = 0;
            I2CCONbits.ACKEN = 1;
        } else {                    // But for last read, NACK is sent.            
            I2CCONbits.ACKDT = 1;
            I2CCONbits.ACKEN = 1;
        }
        while (I2CCONbits.ACKEN == 1); // Wait till ACK/NACK sequence is over 
    }    

    //printf("\rSTOP I2C");
    StopI2C();
    IdleI2C();        
    
    return true;
}


unsigned char StorePcapNVRAM()
{   
unsigned char command[1] = {ACTIVATE_STORE};

    PcapWriteNVRAM(MEM_CTRL_REGISTER + CONFIG_ADDRESS_OFFSET, command, 1);
    SendOpcode(STORE_NVRAM);         
    return true;
}

unsigned char ErasePcapNVRAM()
{
unsigned char command[1] =  {ACTIVATE_ERASE};

    PcapWriteNVRAM(MEM_CTRL_REGISTER + CONFIG_ADDRESS_OFFSET, command, 1);
    SendOpcode(ERASE_NVRAM);
    return true;
}

unsigned char RecallPcapNVRAM()
{   
unsigned char command[1] =  {ACTIVATE_RECALL};
 
    PcapWriteNVRAM(MEM_CTRL_REGISTER + CONFIG_ADDRESS_OFFSET, command, 1);
    SendOpcode(RECALL_NVRAM);     
    return true;
}
