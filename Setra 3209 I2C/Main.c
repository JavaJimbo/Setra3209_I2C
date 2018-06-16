/***********************************************************************************************************
 * Setra 3209 Cal System
 * Written for PIC32MX795 using XC32 compiler V 1.30
 * Adapted from DC950 Tester Firmware
 * 
 * 04-23-18: Reading, writing to NVRAM and ERASE, RECALL, and STORE all work.
 *              Questions about CHARGE PUMP bits.
 * 5-4-18:  Completed routines for writing and reading entire NVRAM memory MAP.           
 * 5-5-18:  Minor cleanup. Works with Setra3209CalSystem program.
 *          Responses to all commands from host should start with "$OK"
 * 5-6-18:	Swapped high and low address bytes.
 * 5-22-18: $OK is response. CRC check is re-enabled.
 * 5-23-18: NO ERASE WRITE STORE
 * 6-02-18: WARWICK: Fixed bug in RETURN RESULT routine that was only returning four data bytes
 *          Also added "<" to end of all returned strings.
 * 6-06-18: Setra: added leading ">"; Created $HELLO command; 
 *          Added routine for writing/reading firmware version number.
 * 6-07-18: Got rid of first and last characters '>' and '<'
 * 6-08-18:	Boxborough: bumped PCap baud rate up to 115200
 * 6-10-18: Warwick: 
 * 6-13-18: Boxborough: no new changes
 * 6-14-18: Tried I2C addressing.
 ************************************************************************************************************/

enum {
    STATE_START = 0,
    STATE_RESET,
    STATE_RECALL,
    STATE_READ,
    STATE_READY
};

// #define TEST_OUT LATBbits.LATB3
#define DIAGNOSTICS

#define CDC_COMMAND 0b10001100
#define RDC_COMMAND 0b10001110
#define DSP_COMMAND 0b10001101

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
#pragma config ICESEL   = ICS_PGx1      // ICE/ICD Comm Channel Select
#pragma config DEBUG    = ON


#define MAXPWM 6500

// UART FOR PC SERIAL PORT
#define HOSTuart UART2
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR

#define RD 1
#define WR 0
#define WRITE_MEM 0xA0
#define READ_MEM 0x20
#define READ_RESULT 0x40
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


#define MEM_CTRL_REGISTER 0x36      // = 54 decimal
// #define CHARGE_PUMP_REGISTER 0x3E   // = 62 decimnal

#define EEDEVICE 0xA0


/** PRIVATE PROTOTYPES *********************************************/
void InitializeSystem(void);
unsigned char   processInputString(unsigned char *ptrBuffer);
unsigned char   executeCommand(unsigned char *ptrCommand, short *ptrValue);
unsigned char   setPWM(unsigned char *ptrPWMstring);
unsigned char   setOutput(unsigned char *ptrPin, unsigned char *ptrPinState);
unsigned char   diagnosticsPrintf(unsigned char *ptrString);
unsigned char   sendToUART(unsigned char UartID, unsigned char *ptrUARTpacket);
extern BOOL     CRCcheck(char *ptrPacket);
extern UINT16   CRCcalculate(char *ptrPacket, BOOL addCRCtoPacket);
unsigned char   OpenPcapI2C();
unsigned char   PCAPPowerOnReset();
unsigned char   InitializePCAP();
unsigned char   PcapReadNVRAM(unsigned short startAddress, unsigned short numBytes, unsigned char *ptrData);
unsigned char   PcapWriteNVRAM(unsigned short startAddress, unsigned short numBytes, unsigned char *ptrData);
unsigned char   RecallPcapNVRAM();
unsigned char   StorePcapNVRAM();
unsigned char   ErasePcapNVRAM();
unsigned char   SendOpcode(unsigned char opcode);
unsigned char   TestRead(unsigned char *ptrReply);
unsigned char   replyToHost(unsigned char *ptrMessage);
unsigned char   PcapReadResultRegisters(unsigned char registerAddress, unsigned short numBytes, unsigned char *ptrData);
unsigned char   storeVersionString();

unsigned char HOSTRxBuffer[MAX_RX_BUFFERSIZE];
unsigned char HOSTRxBufferFull = false;
unsigned char HOSTTxBuffer[MAX_TX_BUFFERSIZE];
unsigned char HOSTTxBufferFull = false;

unsigned short NVRAMsize = 0;
unsigned char NVRAMdata[MAX_NVRAM_ADDRESS];
unsigned char CopyNVRAMdata[MAX_NVRAM_ADDRESS];


#define CHARGE_PUMP_LOW 1022
#define CHARGE_PUMP_HIGH 1023
//  #define chargePumpLowByte NVRAMdata[CHARGE_PUMP_LOW]
//  #define chargePumpHighByte NVRAMdata[CHARGE_PUMP_HIGH]

void InitializeNVRAM(void);
unsigned char PCAPstate = STATE_START;
#define MAXSTRING 128
unsigned char strPCapFirmwareVersion[MAXSTRING] = "";
unsigned char strDataBufferCopy[MAXSTRING];

void main(void){   
             
    InitializeSystem();       
    InitializeNVRAM();   
    OpenPcapI2C();    

    while(1) 
    {
        if (HOSTRxBufferFull) 
        {
            HOSTRxBufferFull = false;               
            if (!CRCcheck(HOSTRxBuffer))
               replyToHost("!CRC ERROR");                        
            else if (!processInputString(HOSTRxBuffer))
               replyToHost("!COMMAND STRING ERROR");
        }
    }
    
}

unsigned char sendToUART(unsigned char UartID, unsigned char *ptrUARTpacket)
{
    short i;
    unsigned char ch;

    if (strlen(ptrUARTpacket) < MAX_TX_BUFFERSIZE) {
        i = 0;
        do {
            ch = ptrUARTpacket[i++];
            if (!ch) break;
            while (!UARTTransmitterIsReady(UartID));
            UARTSendDataByte(UartID, ch);
        } while (i < MAX_TX_BUFFERSIZE);
        return (true);
    }
    else return (false);
}

void InitializeNVRAM()
{
    unsigned short i;
    for (i = 0; i < MAX_NVRAM_ADDRESS; i++)     
        CopyNVRAMdata[i] = NVRAMdata[i] = 0x00;    
}

void InitializeSystem(void) {
    // Turn off JTAG so we get the pins back
    mJTAGPortEnable(false);

    // Set up HOST UART
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
#define SYS_FREQ 80000000
    UARTSetDataRate(HOSTuart, SYS_FREQ, 115200);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure HOST UART Interrupts
    INTEnable(INT_SOURCE_UART_TX(HOSTuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);


    PORTClearBits(IOPORT_B, BIT_3 | BIT_12 | BIT_13 | BIT_14 | BIT_15);
    mPORTBSetPinsDigitalOut(BIT_3 | BIT_12 | BIT_13 | BIT_14 | BIT_15);


    PORTSetPinsDigitalIn(IOPORT_B, BIT_1);
    PORTSetPinsDigitalOut(IOPORT_B, BIT_0);
    PORTSetBits(IOPORT_E, BIT_0);
    PORTSetPinsDigitalIn(IOPORT_D, BIT_15);  // Initialize FAULT signal input

    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();

}//end UserInit

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
            // ch = toupper(UARTGetDataByte(HOSTuart));
            ch = UARTGetDataByte(HOSTuart);
            if (ch == '$') HOSTRxIndex = 0;
            if (ch == LF || ch == 0);
            else if (ch == BACKSPACE) {
                while (!UARTTransmitterIsReady(HOSTuart));
                UARTSendDataByte(HOSTuart, ' ');
                while (!UARTTransmitterIsReady(HOSTuart));
                UARTSendDataByte(HOSTuart, BACKSPACE);
                if (HOSTRxIndex > 0) HOSTRxIndex--;
            } else if (ch == CR) {
                if (HOSTRxIndex < (MAX_RX_BUFFERSIZE - 1)) {
                    HOSTRxBuffer[HOSTRxIndex] = CR;
                    HOSTRxBuffer[HOSTRxIndex + 1] = '\0'; 
                    HOSTRxBufferFull = true;
                }
                HOSTRxIndex = 0;
            }                
            else if (HOSTRxIndex < MAX_RX_BUFFERSIZE)
                HOSTRxBuffer[HOSTRxIndex++] = ch;
        }
    }
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));
    }
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
    if (I2CSTATbits.ACKSTAT) return false; // printf("\rNO ACK #1"); // Get ACK from EEPROM    
        
    MasterWriteI2C(POWER_ON_RESET); // Send opcode and two high address bits
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return false; // intf("\rNO ACK #2"); // Get ACK from EEPROM       
    
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
        //printf("\rNO ACK #1"); // Get ACK from EEPROM
        return false;
    }
    
    MasterWriteI2C(INITIALIZE); // Send opcode and two high address bits
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT)
    {
        //printf("\rNO ACK #2"); // Get ACK from EEPROM
        return false;
    }
    
    StopI2C(); // Send the Stop condition
    IdleI2C(); // Wait to complete
    return true;
}



unsigned char ErasePcapNVRAM()
{
unsigned char command[1] =  {ACTIVATE_ERASE};

    if (!PcapWriteNVRAM(MEM_CTRL_REGISTER + CONFIG_ADDRESS, 1, command))
        return false;
    if (!SendOpcode(ERASE_NVRAM))
        return false;
    return true;
}

unsigned char RecallPcapNVRAM()
{   
unsigned char command[1] =  {ACTIVATE_RECALL};
 
    if (!PcapWriteNVRAM(MEM_CTRL_REGISTER + CONFIG_ADDRESS, 1, command))
        return false;
    if (!SendOpcode(RECALL_NVRAM))
        return false;
    return true;
}

unsigned char executeCommand(unsigned char *ptrCommand, short *ptrData)
{
    char strHexValue[16];
    char strReply[MAX_TX_BUFFERSIZE] = "$OK";    
    unsigned short NVRAMstartAddress, address;
    unsigned short numDataBytes = 0;
    unsigned char registerAddress = 0;
    unsigned short i;    
    #define MAX_RESULT_REGISTERS 35
    unsigned char arrResultRegister[MAX_RESULT_REGISTERS];
    
    union {
        unsigned char byte[2];
        unsigned int  integer;	
    } convert;
    #define HighByte byte[1]
    #define LowByte byte[0]    

    if (!strcmp(ptrCommand, "HELLO"))
    {
        strcpy(strReply, "$OK HELLO_WORLD");
    }
    else if (!strcmp(ptrCommand, "WRITE_VERSION"))
    {       
        if (storeVersionString()) strcpy(strReply, "$OK ");
        else strcpy(strReply, "$ERROR Bad version string");
    }    
    else if (!strcmp(ptrCommand, "READ_VERSION"))
    {
        strcpy(strReply, "$OK ");
        strcat(strReply, strPCapFirmwareVersion);
    }        
    else if (!strcmp(ptrCommand, "POR"))
    {
        if (!PCAPPowerOnReset()) strcpy(strReply, "$I2C_ERROR");
    }
    else if (!strcmp(ptrCommand, "INIT"))
    {
        if (!InitializePCAP()) strcpy(strReply, "$I2C_ERROR");
    }
    else if (!strcmp(ptrCommand, "CDC"))  // $$$$
    {
        if (!SendOpcode(CDC_COMMAND)) strcpy(strReply, "$I2C_ERROR");
    }
    else if (!strcmp(ptrCommand, "RDC"))
    {
        if (!SendOpcode(RDC_COMMAND)) strcpy(strReply, "$I2C_ERROR");
    }    
    else if (!strcmp(ptrCommand, "DSP"))
    {
        if (!SendOpcode(DSP_COMMAND)) strcpy(strReply, "$I2C_ERROR");
    }
    else if (!strcmp(ptrCommand, "STORE"))
    {
        if (!StorePcapNVRAM()) strcpy(strReply, "$I2C_ERROR");        
    }
    else if (!strcmp(ptrCommand, "RECALL")) 
    {
        if (!RecallPcapNVRAM()) strcpy(strReply, "$I2C_ERROR");
    }
    else if (!strcmp(ptrCommand, "ERASE"))
    {
        if (!ErasePcapNVRAM()) strcpy(strReply, "$I2C_ERROR");        
    }    
    else if (!strcmp(ptrCommand, "TEST"))
    {
        unsigned char ReplyByte;
        if (!TestRead(&ReplyByte)) strcpy(strReply, "$I2C_ERROR");
        else if (ReplyByte != 0x11) sprintf(strReply, "$TEST_READ_ERROR");
    }
    else if (!strcmp(ptrCommand, "WCONFIG"))
    {
    }
    else if (!strcmp(ptrCommand, "RCONFIG"))
    {
    }        
    else if (!strcmp(ptrCommand, "PRINT"))
    {
        for (i = 0; i < MAX_NVRAM_ADDRESS; i++)
        {        
            if ((i % 16) == 0) 
            {
                DelayMs(2);
                printf("\r#%04X: %02X, ", i, NVRAMdata[i]);
            }
            else printf("%02X, ", NVRAMdata[i]);
        }
    }    
    // else if (!strcmp(ptrCommand, "CLEAR"))
    //    ClearNVRAMdata();
    else if (!strcmp(ptrCommand, "UPLOAD"))
    {        
        convert.byte[0] = (unsigned char) ptrData[1];
        convert.byte[1] = (unsigned char) ptrData[0];
        NVRAMstartAddress = convert.integer;   
        
        convert.byte[0] = (unsigned char) ptrData[3];
        convert.byte[1] = (unsigned char) ptrData[2];
        numDataBytes = convert.integer;       
        
        strReply[0] = '\0';        
        if ((NVRAMstartAddress + numDataBytes) > MAX_NVRAM_ADDRESS)
            strcpy(strReply, "$ADDRESS_OUT_OF_RANGE");
        else
        {
            strcpy(strReply, "$OK ");
            for (i = NVRAMstartAddress; i < (NVRAMstartAddress + numDataBytes); i++)  // $$$$
            {
                sprintf(strHexValue, " %02X", NVRAMdata[i]);
                strcat(strReply, strHexValue);
            }
        }
    }        
    else if (!strcmp(ptrCommand, "BOGUS_MAP"))
    {       
        unsigned char testData = 0;
        for (i = 0; i < 960; i++)
        {
            NVRAMdata[i] = testData++;
            if (testData > 254) testData = 0x0;
        }
    }
    else if (!strcmp(ptrCommand, "DOWNLOAD"))
    {                
        convert.byte[0] = (unsigned char) ptrData[1];
        convert.byte[1] = (unsigned char) ptrData[0];
        NVRAMstartAddress = convert.integer;   
        
        convert.byte[0] = (unsigned char) ptrData[3];
        convert.byte[1] = (unsigned char) ptrData[2];
        numDataBytes = convert.integer;               
        
        if ((NVRAMstartAddress + numDataBytes) > MAX_NVRAM_ADDRESS)
            strcpy(strReply, "$ADDRESS_OUT_OF_RANGE");
        else
        {            
            for (i = 0; i < numDataBytes; i++)
            {
                address = i + NVRAMstartAddress;
                {
                    // Copy downloaded data to any address in NVRAM
                    // EXCEPT for the two CHARGE PUMP bytes,
                    // which should never be overwritten by external data;
                    if (address != CHARGE_PUMP_LOW && address != CHARGE_PUMP_HIGH)
                        NVRAMdata[address] = (unsigned char) ptrData[i+4]; 
                }
            }
        }
    }
    else if (!strcmp(ptrCommand, "READ_MAP"))
    {                
        if (!PcapReadNVRAM(0x0000, MAX_NVRAM_ADDRESS, NVRAMdata))
            strcpy(strReply, "$I2C_ERROR");
    }       
    else if (!strcmp(ptrCommand, "WRITE_MAP"))
    {       
        for (i = 0; i < MAX_NVRAM_ADDRESS; i++)
            CopyNVRAMdata[i] = NVRAMdata[i];
        
        if (!PcapWriteNVRAM(0x0000, MAX_NVRAM_ADDRESS, NVRAMdata))
            strcpy(strReply, "$I2C_ERROR");        
    } 
    else if (!strcmp(ptrCommand, "CHECK_MAP"))
    {        
        for (i = 0; i < MAX_NVRAM_ADDRESS; i++)
        {
          if (NVRAMdata[i] != CopyNVRAMdata[i]) break;
        }
        if (i == MAX_NVRAM_ADDRESS) strcpy(strReply, "$OK NVRAM WRITE SUCCESS");
        else strcpy(strReply, "$ERROR NVRAM INCORRECT");
    }
    else if (!strcmp(ptrCommand, "READ"))
    {        
        convert.byte[0] = (unsigned char) ptrData[1];
        convert.byte[1] = (unsigned char) ptrData[0];
        NVRAMstartAddress = convert.integer;   
        
        convert.byte[0] = (unsigned char) ptrData[3];
        convert.byte[1] = (unsigned char) ptrData[2];
        numDataBytes = convert.integer;   
        
        if (PcapReadNVRAM(NVRAMstartAddress, numDataBytes, NVRAMdata))
        {
            // printf("DONE");
            strcpy(strReply, "$OK ");
            for (i = NVRAMstartAddress; i < (NVRAMstartAddress + numDataBytes); i++)
            {
                sprintf(strHexValue, "%02X ", NVRAMdata[i]);
                strcat(strReply, strHexValue);
            }                        
        }
        else strcpy(strReply, "$I2C_ERROR");
    }       
    
    else if (!strcmp(ptrCommand, "READ_RESULT"))
    {        
        numDataBytes = (unsigned char) ptrData[1];
        registerAddress = (unsigned char) ptrData[0];
        
        for (i = 0; i < MAX_RESULT_REGISTERS; i++) arrResultRegister[i] = 0;            
        
        if ((registerAddress + numDataBytes) > MAX_RESULT_REGISTERS)        
            strcpy(strReply, "$REGISTER_OUT_OF_RANGE");                    
        else if (PcapReadResultRegisters(registerAddress, numDataBytes, arrResultRegister))
        {     
            strcpy(strReply, "$OK ");
            for (i = registerAddress; i < (registerAddress + numDataBytes); i++)
            {
                sprintf(strHexValue, " %02X", arrResultRegister[i]);
                strcat(strReply, strHexValue);
            }                 
        }
        else  strcpy(strReply, "$PCAP ERROR");        
    }       
    
    else if (!strcmp(ptrCommand, "WRITE"))
    {          
        convert.byte[0] = (unsigned char) ptrData[1];
        convert.byte[1] = (unsigned char) ptrData[0];
        NVRAMstartAddress = convert.integer;   
        
        convert.byte[0] = (unsigned char) ptrData[3];
        convert.byte[1] = (unsigned char) ptrData[2];
        numDataBytes = convert.integer;   
        
        if ((NVRAMstartAddress + numDataBytes) >= MAX_NVRAM_ADDRESS)
        {
            strcpy(strReply, "$ERROR: ADRESS OUT OF BOUNDS");   
            return false;
        }        
        //if (!PcapWriteNVRAM(NVRAMstartAddress, numDataBytes, &ptrData[4]))
        //    strcpy(strReply, "!I2C_ERROR");
        for (i = NVRAMstartAddress; i < numDataBytes; i++)
        {
            // Copy data to any address in NVRAM
            // EXCEPT for the two CHARGE PUMP bytes,
            // which should never be overwritten by external data;
            if (i != CHARGE_PUMP_LOW && i != CHARGE_PUMP_HIGH)
                NVRAMdata[i] = (unsigned char) ptrData[i+4];
        }        
    }               
    else strcpy(strReply, "$ERROR UNRECOGNIZED COMMAND");
    // strcat(strReply, "<");
    replyToHost(strReply);
    return true;
}

#define TEST_READ_COMMAND 0x7E
unsigned char TestRead(unsigned char *ptrReply)
{       
    StartI2C(); // Send the Start Bit
    IdleI2C(); // Wait to complete    
    
    MasterWriteI2C(PCAP_DEVICE | WR); // Send EEPROM Device ID and WRITE Command    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) 
        return false;
        
    MasterWriteI2C(TEST_READ_COMMAND); // Send opcode
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) 
        return false;
    
    RestartI2C(); // Now send START sequence again:
    IdleI2C(); // Wait to complete

    MasterWriteI2C(PCAP_DEVICE | RD); // Send EEPROM Device ID and READ Command     
    IdleI2C(); // Wait to complete    
    if (I2CSTATbits.ACKSTAT) 
        return false;  
    
    // Now receive data byte:
    I2CCONbits.RCEN = 1;
    while (!DataRdyI2C()); // Wait for incoming data byte
    *ptrReply = I2CRCV; // Read data byte from buffer

    I2CCONbits.ACKDT = 1; // Send NACK to EEPROM
    I2CCONbits.ACKEN = 1;
    while (I2CCONbits.ACKEN == 1); // Wait till ACK/NACK sequence is over     

    StopI2C();
    IdleI2C();        
    return true;
}

unsigned char SendOpcode(unsigned char opcode)
{       
    StartI2C(); // Send the Start Bit
    IdleI2C(); // Wait to complete    
    
    MasterWriteI2C(PCAP_DEVICE | WR); // Send EEPROM Device ID and WRITE Command    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) 
    {
        //printf("\rNO ACK #1"); // Get ACK from EEPROM
        return false;
    }        
        
    MasterWriteI2C(opcode); // Send opcode
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) 
    {
        //printf("\rNO ACK #2"); // Get ACK from EEPROM
        return false;
    }                

    StopI2C();
    IdleI2C();        
    return true;
}

unsigned char replyToHost(unsigned char *ptrMessage){
    if (ptrMessage == NULL) return (FALSE);
    if (strlen(ptrMessage) > MAX_TX_BUFFERSIZE) return (FALSE);
    strcpy(HOSTTxBuffer, ptrMessage);
    CRCcalculate(HOSTTxBuffer, true);
    sendToUART(HOSTuart, HOSTTxBuffer);    
    return(TRUE);
}


unsigned char   PcapWriteNVRAM(unsigned short startAddress, unsigned short numBytes, unsigned char *ptrData)
{    
    union {
        unsigned char byte[2];
        unsigned int  integer;	
    } convert;
    #define HighByte byte[1]
    #define LowByte byte[0]    
    unsigned short i;
        
    // WRITE DATA
    StartI2C(); // Send the Start Bit
    IdleI2C(); // Wait to complete    

    MasterWriteI2C(PCAP_DEVICE | WR); // Send EEPROM Device ID and WRITE Command    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) 
    {
        //printf("\rNO ACK #1"); // Get ACK from EEPROM
        return false;
    }
    
    convert.integer = startAddress;
    MasterWriteI2C(WRITE_MEM | (convert.HighByte & 0x03)); // Send opcode ORed with two high address bits    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) 
    {
        //printf("\rNO ACK #2"); // Get ACK from EEPROM    
        return false;
    }

    MasterWriteI2C(convert.LowByte); // Send EEPROM low address byte    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT)
    {
        //printf("\rNO ACK #3"); // Get ACK from EEPROM    
        return false;
    }

    // Now send test data
    for (i = 0; i < numBytes; i++) 
    {
        MasterWriteI2C(ptrData[i]);        
        IdleI2C(); //Wait to complete
        if (I2CSTATbits.ACKSTAT)
        {
            //printf("\rNO ACK #4"); // Get ACK from EEPROM 
            return false;
        }        
    }    
    
    StopI2C(); // Send the Stop condition
    IdleI2C(); // Wait to complete
    
    // printf ("DONE.");

    return true;
}

unsigned char   PcapReadNVRAM(unsigned short startAddress, unsigned short numBytes, unsigned char *ptrData)
{
    unsigned short i;
        
    union {
        unsigned char byte[2];
        unsigned int  integer;	
    } convert;
    #define HighByte byte[1]
    #define LowByte byte[0]       
    
    // printf("\rReading %d bytes @ %d", numBytes, startAddress);
        
    StartI2C(); // Send the Start Bit
    IdleI2C(); // Wait to complete    
    
    MasterWriteI2C(PCAP_DEVICE | WR); // Send EEPROM Device ID and WRITE Command    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return false;
    
    convert.integer = startAddress;
    MasterWriteI2C(READ_MEM | (convert.HighByte & 0x03)); // Send opcode ORed with two high address bits
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return false;
    
    MasterWriteI2C(convert.LowByte); // Send EEPROM low address byte
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return false;

    RestartI2C(); // Now send START sequence again:
    IdleI2C(); // Wait to complete

    MasterWriteI2C(PCAP_DEVICE | RD); // Send EEPROM Device ID and READ Command     
    IdleI2C(); // Wait to complete    
    if (I2CSTATbits.ACKSTAT) return false;
    
    DelayMs(4);

    for (i = 0; i < numBytes; i++) 
    {
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
    StopI2C();
    IdleI2C();            
    return true;
}

unsigned char   PcapReadResultRegisters(unsigned char registerAddress, unsigned short numBytes, unsigned char *ptrData)
{
    unsigned short i;   
       
    for (i = 0; i < numBytes; i++) ptrData[i] = 0;    
   
    // printf("\rReading %d bytes @ %d", numBytes, startAddress);
        
    StartI2C(); // Send the Start Bit
    IdleI2C(); // Wait to complete    
    
    MasterWriteI2C(PCAP_DEVICE | WR); // Send EEPROM Device ID and WRITE Command    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return false;
        
    //Delay10us(100);
    
    MasterWriteI2C(READ_RESULT | (registerAddress & 0x3F)); // Send opcode ORed with six address bits
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return false;    
    
    //Delay10us(100);

    RestartI2C(); // Now send START sequence again:
    IdleI2C(); // Wait to complete

    MasterWriteI2C(PCAP_DEVICE | RD); // Send EEPROM Device ID and READ Command     
    IdleI2C(); // Wait to complete    
    if (I2CSTATbits.ACKSTAT) return false;
    
    // DelayMs(4); 

    for (i = 0; i < numBytes; i++) 
    {
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
    StopI2C();
    IdleI2C();            
    return true;
}

unsigned char StorePcapNVRAM()
{   
unsigned char command[1] = {ACTIVATE_STORE};

    if (!PcapWriteNVRAM(MEM_CTRL_REGISTER + CONFIG_ADDRESS, 1, command))
        return false;
    if (!SendOpcode(STORE_NVRAM))
        return false;
    return true;
}

unsigned char processInputString(unsigned char *ptrBuffer) 
{        
    unsigned char ptrCommand[64], *ptrToken;
    unsigned char delimiters[] = "$>[\r ";
    short dataIndex = 0, intValue = 0;
    short arrByteData[MAX_NVRAM_ADDRESS];      
    unsigned char dataFlag = true;
    
    if (strchr(ptrBuffer, '@')!= NULL)    
    {
        if (strlen(ptrBuffer) < MAXSTRING) strcpy(strDataBufferCopy, ptrBuffer);
        else strDataBufferCopy[0] = '\0';       
        dataFlag = false;
    }    

    ptrToken = strtok(ptrBuffer, delimiters);
    
    if (ptrToken==NULL) return (false);
    if (strlen(ptrToken) > 64) return false;        
    strcpy (ptrCommand, ptrToken);     
    if (dataFlag)
    {
        dataIndex = 0;
        do
        {
            ptrToken = strtok(NULL, delimiters);
            if (ptrToken)
            {         
                intValue = (int) strtol(ptrToken, NULL, 16);            
                if (dataIndex < MAX_NVRAM_ADDRESS)      
                {
                    arrByteData[dataIndex] = (short) intValue;
                    dataIndex++;
                }
                else return false;
            }        
        } while (ptrToken != NULL);
    }
    if (executeCommand(ptrCommand, arrByteData)) return (true);
    else return (false);
}

unsigned char storeVersionString()
{
int i, j, length;
char ch = 0;

    length = strlen(strDataBufferCopy);
    
    if (length == 0 || length > MAXSTRING) return false;    
    
    for (i = 0; i < length; i++)
    {
        ch = strDataBufferCopy[i];
        if (ch == '$') break;
    }

    if (ch != '$') return false;

    for (i = i; i < length; i++)
    {
        ch = strDataBufferCopy[i];
        if (ch == ' ') break;
    }

    if (ch != ' ') return false;

    j = 0;
    for (i = i; i < length; i++)
    {
        ch = strDataBufferCopy[i];
        if (ch == '[') break;
        if (j >= MAXSTRING-2) return false;
        strPCapFirmwareVersion[j++] = ch;        
    }
    strPCapFirmwareVersion[j] = '\0';
    if (ch != '[') return false;
    else return true;
}