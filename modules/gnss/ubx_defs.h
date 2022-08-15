

#ifndef UBX_DEFS_H_
#define UBX_DEFS_H_

#include <stdint.h>
#include <stddef.h>

/*
* Aliases for uBlox defined types to make defining packets from the interface
* descriptions easier.
*/
//using U1 = uint8_t;
//using I1 = int8_t;
//using X1 = uint8_t;
//using U2 = uint16_t;
//using I2 = int16_t;
//using X2 = uint16_t;
//using U4 = uint32_t;
//using I4 = int32_t;
//using X4 = uint32_t;
//using R4 = float;
//using R8 = double;
//using CH = char;

typedef uint8_t U1;
typedef int8_t I1;
typedef uint8_t X1;
typedef uint16_t U2;
typedef int16_t I2;
typedef uint16_t X2;
typedef uint32_t U4;
typedef int32_t I4;
typedef uint32_t X4;
typedef float R4;
typedef double R8;
typedef char CH;




/* Classes */
const uint8_t UBX_ACK_CLS_ = 0x05;
const uint8_t UBX_CFG_CLS_ = 0x06;
const uint8_t UBX_INF_CLS_ = 0x04;
const uint8_t UBX_LOG_CLS_ = 0x21;
const uint8_t UBX_MGA_CLS_ = 0x13;
const uint8_t UBX_MON_CLS_ = 0x0a;
const uint8_t UBX_NAV_CLS_ = 0x01;
const uint8_t UBX_RXM_CLS_ = 0x02;
const uint8_t UBX_SEC_CLS_ = 0x27;
const uint8_t UBX_TIM_CLS_ = 0x0d;
const uint8_t UBX_UPD_CLS_ = 0x09;
/* Port definitions */
const uint8_t UBX_COM_PORT_I2C_ = 0;
const uint8_t UBX_COM_PORT_UART1_ = 1;
const uint8_t UBX_COM_PORT_UART2_ = 2;
const uint8_t UBX_COM_PORT_USB_ = 3;
const uint8_t UBX_COM_PORT_SPI_ = 4;
/* Port protocols */
const uint8_t UBX_COM_PROT_UBX_ = 0x01;
const uint8_t UBX_COM_PROT_NMEA_ = 0x02;
const uint8_t UBX_COM_PROT_RTCM_ = 0x04;
const uint8_t UBX_COM_PROT_RTCM3_ = 0x08;


#endif /* UBX_DEFS_H_ */
