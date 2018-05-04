/* 
 * File:   Definitions.h
 * Author: Jim
 *
 * Created on October 21, 2017, 3:15 PM
 */

#ifndef DEFINITIONS_H
#define	DEFINITIONS_H
// #define BUFFERSIZE 256 // was 128

#define CONFIG_ADDRESS 0x03C0
#define MAX_NVRAM_ADDRESS CONFIG_ADDRESS
#define MAXDATABYTES CONFIG_ADDRESS
#define MAX_RX_BUFFERSIZE (MAXDATABYTES * 4)
#define MAX_TX_BUFFERSIZE MAX_RX_BUFFERSIZE
#define MAXBUFFER MAX_TX_BUFFERSIZE


#define true TRUE
#define false FALSE

//#ifndef FALSE
//#define FALSE 0
//#define TRUE !FALSE
//#endif

#endif	/* DEFINITIONS_H */

