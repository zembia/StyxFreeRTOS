#ifndef SPI_TOOLS_H
#define SPI_TOOLS_H

#include "xspips.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <stdint.h>
#include <stdbool.h>
#include "baseStructures.h"

/* W5100 Register Definitions */
#define W5100_MR        0x0000  // Mode Register
#define W5100_GAR       0x0001  // Gateway Address Register
#define W5100_SUBR      0x0005  // Subnet Mask Register
#define W5100_SHAR      0x0009  // Source Hardware Address Register
#define W5100_SIPR      0x000F  // Source IP Address Register
#define W5100_IR        0x0015  // Interrupt Register
#define W5100_IMR       0x0016  // Interrupt Mask Register
#define W5100_RTR       0x0017  // Retry Time Register
#define W5100_RCR       0x0019  // Retry Count Register
#define W5100_RMSR      0x001A  // RX Memory Size Register
#define W5100_TMSR      0x001B  // TX Memory Size Register
#define W5100_PATR      0x001C  // PPPoE Authentication Type Register
#define W5100_PTIMER    0x0028  // PPP LCP Request Timer Register
#define W5100_PMAGIC    0x0029  // PPP LCP Magic Number Register
#define W5100_UIPR      0x002A  // Unreachable IP Address Register
#define W5100_UPORT     0x002E  // Unreachable Port Register

/* Socket Register Base Addresses */
#define W5100_S0_BASE   0x0400
#define W5100_S1_BASE   0x0500
#define W5100_S2_BASE   0x0600
#define W5100_S3_BASE   0x0700

/* Socket Register Offsets */
#define W5100_Sn_MR     0x0000  // Socket Mode Register
#define W5100_Sn_CR     0x0001  // Socket Command Register
#define W5100_Sn_IR     0x0002  // Socket Interrupt Register
#define W5100_Sn_SR     0x0003  // Socket Status Register
#define W5100_Sn_PORT   0x0004  // Socket Source Port Register
#define W5100_Sn_DHAR   0x0006  // Socket Destination Hardware Address
#define W5100_Sn_DIPR   0x000C  // Socket Destination IP Address
#define W5100_Sn_DPORT  0x0010  // Socket Destination Port
#define W5100_Sn_MSSR   0x0012  // Socket Maximum Segment Size
#define W5100_Sn_PROTO  0x0014  // Socket Protocol (IP Raw mode)
#define W5100_Sn_TOS    0x0015  // Socket IP TOS
#define W5100_Sn_TTL    0x0016  // Socket IP TTL
#define W5100_Sn_TX_FSR 0x0020  // Socket TX Free Size
#define W5100_Sn_TX_RD  0x0022  // Socket TX Read Pointer
#define W5100_Sn_TX_WR  0x0024  // Socket TX Write Pointer
#define W5100_Sn_RX_RSR 0x0026  // Socket RX Received Size
#define W5100_Sn_RX_RD  0x0028  // Socket RX Read Pointer

/* Socket Commands */
#define W5100_SOCK_OPEN      0x01
#define W5100_SOCK_LISTEN    0x02
#define W5100_SOCK_CONNECT   0x04
#define W5100_SOCK_DISCON    0x08
#define W5100_SOCK_CLOSE     0x10
#define W5100_SOCK_SEND      0x20
#define W5100_SOCK_SEND_MAC  0x21
#define W5100_SOCK_SEND_KEEP 0x22
#define W5100_SOCK_RECV      0x40

/* Socket Status */
#define W5100_SNSR_CLOSED      0x00
#define W5100_SNSR_INIT        0x13
#define W5100_SNSR_LISTEN      0x14
#define W5100_SNSR_SYNSENT     0x15
#define W5100_SNSR_SYNRECV     0x16
#define W5100_SNSR_ESTABLISHED 0x17
#define W5100_SNSR_FIN_WAIT    0x18
#define W5100_SNSR_CLOSING     0x1A
#define W5100_SNSR_TIME_WAIT   0x1B
#define W5100_SNSR_CLOSE_WAIT  0x1C
#define W5100_SNSR_LAST_ACK    0x1D
#define W5100_SNSR_UDP         0x22
#define W5100_SNSR_IPRAW       0x32
#define W5100_SNSR_MACRAW      0x42
#define W5100_SNSR_PPPOE       0x5F

/* Socket Mode Register bits */
#define W5100_SnMR_CLOSE  0x00
#define W5100_SnMR_TCP    0x01
#define W5100_SnMR_UDP    0x02
#define W5100_SnMR_IPRAW  0x03
#define W5100_SnMR_MACRAW 0x04
#define W5100_SnMR_PPPOE  0x05
#define W5100_SnMR_ND     0x20  // No Delayed ACK
#define W5100_SnMR_MULTI  0x80  // Multicast

/* Memory Organization */
#define W5100_TXBUF_BASE 0x4000
#define W5100_RXBUF_BASE 0x6000
#define W5100_TXBUF_SIZE 0x0800  // 2KB per socket
#define W5100_RXBUF_SIZE 0x0800  // 2KB per socket

/* SPI Operation Codes */
#define W5100_SPI_OP_WRITE 0xF0
#define W5100_SPI_OP_READ  0x0F

/* Configuration Constants */
#define W5100_MAX_SOCK_NUM 4
#define W5100_SOCKETS      4

/* WRAPPER PACKET CONSTANTS */
#define WRAPPER_START_PKT 0x00
#define WRAPPER_MIDDLE_PKT 0x01
#define WRAPPER_END_PKT 0x02
#define WRAPPER_HEADER_SIZE 11
#define WRAPPER_PKT_SIZE 1460-WRAPPER_HEADER_SIZE

/* Chip variants */
typedef enum {
    W5100_CHIP_W5100 = 51,
    W5100_CHIP_W5200 = 52,
    W5100_CHIP_W5500 = 55,
    W5100_CHIP_UNKNOWN = 0
} W5100_ChipType;


/* Driver Configuration Structure */
typedef struct {
    XSpiPs *spi_instance;        // Pointer to initialized XSpiPs instance
    uint8_t cs_pin;              // Chip select pin (if using GPIO CS)
    bool use_hardware_cs;        // Use SPI controller CS or manual GPIO
    uint32_t spi_clk_hz;         // SPI clock frequency in Hz
    SemaphoreHandle_t spi_mutex; // Mutex for thread-safe SPI access
} W5100_Config;

/* Driver Handle Structure */
typedef struct {
    W5100_Config config;
    W5100_ChipType chip;
    bool initialized;
    uint16_t SBASE[W5100_MAX_SOCK_NUM]; // Socket TX buffer base addresses
    uint16_t RBASE[W5100_MAX_SOCK_NUM]; // Socket RX buffer base addresses
    uint16_t SSIZE;  // TX buffer size per socket
    uint16_t RSIZE;  // RX buffer size per socket
    uint16_t SMASK;  // TX buffer mask
    uint16_t RMASK;  // RX buffer mask
} W5100_Handle;



int W5100_Init(W5100_Handle *handle, W5100_Config *config);
int W5100_SoftReset(W5100_Handle *handle);
uint8_t W5100_ReadByte(W5100_Handle *handle, uint16_t addr);
void W5100_WriteByte(W5100_Handle *handle, uint16_t addr, uint8_t data);
void W5100_ReadBuffer(W5100_Handle *handle, uint16_t addr, uint8_t *buf, uint16_t len);
void W5100_WriteBuffer(W5100_Handle *handle, uint16_t addr, const uint8_t *buf, uint16_t len);
uint16_t W5100_ReadWord(W5100_Handle *handle, uint16_t addr);
void W5100_WriteWord(W5100_Handle *handle, uint16_t addr, uint16_t data);
void W5100_ExecCmdSn(W5100_Handle *handle, uint8_t sock, uint8_t cmd);
uint16_t W5100_GetTXFreeSize(W5100_Handle *handle, uint8_t sock);
uint16_t W5100_GetRXReceivedSize(W5100_Handle *handle, uint8_t sock);
void W5100_SendData(W5100_Handle *handle, uint8_t sock, const uint8_t *buf, uint16_t len);
void W5100_RecvData(W5100_Handle *handle, uint8_t sock, uint8_t *buf, uint16_t len);
void W5100_SetGatewayIP(W5100_Handle *handle, const uint8_t *addr);
void W5100_SetSubnetMask(W5100_Handle *handle, const uint8_t *addr);
void W5100_SetMACAddress(W5100_Handle *handle, const uint8_t *addr);
void W5100_SetIPAddress(W5100_Handle *handle, const uint8_t *addr);
void W5100_GetGatewayIP(W5100_Handle *handle, uint8_t *addr);
void W5100_GetSubnetMask(W5100_Handle *handle, uint8_t *addr);
void W5100_GetMACAddress(W5100_Handle *handle, uint8_t *addr);
void W5100_GetIPAddress(W5100_Handle *handle, uint8_t *addr);
void W5100_SetRetryTime(W5100_Handle *handle, uint16_t timeout);
void W5100_SetRetryCount(W5100_Handle *handle, uint8_t count);
void ethernetInit(operation_control_t *op);
void put13s(uint8_t *buf, uint32_t index, int16_t value);

#endif