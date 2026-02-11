#include "xparameters.h"
#include "xspips.h"
#include "xil_printf.h"
#include "W5100.h"
#include "xgpio.h"
#include "W5100dhcp.h"
#include "baseStructures.h"
#include "globaldefines.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#ifndef W5100_SnIR_CON
#define W5100_SnIR_CON 0x01
#define W5100_SnIR_DISCON 0x02
#define W5100_SnIR_RECV 0x04
#define W5100_SnIR_TIMEOUT 0x08
#define W5100_SnIR_SEND_OK 0x10
#endif

/* Private function prototypes */
static void W5100_SPIBegin(W5100_Handle *handle);
static void W5100_SPIEnd(W5100_Handle *handle);
static uint8_t W5100_SPITransfer(W5100_Handle *handle, uint8_t data);
static void W5100_InitSS(W5100_Handle *handle);
static void W5100_SetSS(W5100_Handle *handle);
static void W5100_ResetSS(W5100_Handle *handle);
static W5100_ChipType W5100_DetectChip(W5100_Handle *handle);
static void DHCP_Task(void *pvParameters);
static void TCP_Server_Task(void *pvParameters);

#define W5100_SPI_CLK_HZ 70000000 // 10 MHz SPI clock

__attribute__((section(".wave_buffers"))) uint8_t large_rx_buffer[EM_VECTOR_SIZE + 1024];

/* Network Configuration */
static uint8_t mac_addr[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
static uint8_t ip_addr[4] = {192, 168, 0, 177};
static uint8_t gateway[4] = {192, 168, 0, 1};
static uint8_t subnet[4] = {255, 255, 255, 0};

XSpiPs_Config *SpiConfig;
#define SS_PIN 0

XGpio csGpio;


uint8_t rx_buffer[1480];
uint8_t tx_buffer[1480];

static XSpiPs SpiInstance;

/**
 * @brief Initialize SPI controller
 */
static int InitSPI(XSpiPs *SpiInstancePtr)
{
    int Status;
    XSpiPs_Config *SpiConfig;

    int status;

    status = XGpio_Initialize(&csGpio, XPAR_AXI_GPIO_1_BASEADDR);

    if (status != XST_SUCCESS)
    {
        xil_printf("Gpio Initialization Failed\r\n");
    }
    XGpio_DiscreteWrite(&csGpio, 1, 1);

    /* Initialize the SPI driver */
    SpiConfig = XSpiPs_LookupConfig(XPAR_SPI0_BASEADDR);
    if (SpiConfig == NULL)
    {
        xil_printf("SPI LookupConfig failed\r\n");
        return XST_FAILURE;
    }

    Status = XSpiPs_CfgInitialize(SpiInstancePtr, SpiConfig, SpiConfig->BaseAddress);
    if (Status != XST_SUCCESS)
    {
        xil_printf("SPI CfgInitialize failed\r\n");
        return XST_FAILURE;
    }

    /* Perform a self-test */
    Status = XSpiPs_SelfTest(SpiInstancePtr);
    if (Status != XST_SUCCESS)
    {
        xil_printf("SPI SelfTest failed\r\n");
        return XST_FAILURE;
    }

    xil_printf("SPI initialized successfully\r\n");
    return XST_SUCCESS;
}

static W5100_Handle w5100_handle;
void ethernetInit(operation_control_t *op)
{
    int Status;
    W5100_Config config;

    /* Configure W5100 driver */
    config.spi_instance = &SpiInstance;
    config.cs_pin = 0; // Not used if using hardware CS
    config.use_hardware_cs = false;
    config.spi_clk_hz = W5100_SPI_CLK_HZ;
    config.spi_mutex = NULL; // Will be created b

    InitSPI(config.spi_instance);

    /* Initialize W5100 */
    Status = W5100_Init(&w5100_handle, &config);
    if (Status != XST_SUCCESS)
    {
        xil_printf("W5100 Init failed\r\n");
        return;
    }
    /* Set retry parameters */
    W5100_SetRetryTime(&w5100_handle, 2000); // 200ms
    W5100_SetRetryCount(&w5100_handle, 8);

    /* Configure network parameters */
    W5100_SetMACAddress(&w5100_handle, mac_addr);
    /*W5100_SetIPAddress(&w5100_handle, ip_addr);
    W5100_SetGatewayIP(&w5100_handle, gateway);
    W5100_SetSubnetMask(&w5100_handle, subnet);
    */

    /* Print configuration */
    uint8_t read_mac[6], read_ip[4], read_gw[4], read_subnet[4];

    W5100_GetMACAddress(&w5100_handle, read_mac);
    W5100_GetIPAddress(&w5100_handle, read_ip);
    W5100_GetGatewayIP(&w5100_handle, read_gw);
    W5100_GetSubnetMask(&w5100_handle, read_subnet);

    xil_printf("\r\n=== W5100 Network Configuration ===\r\n");
    xil_printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
               read_mac[0], read_mac[1], read_mac[2],
               read_mac[3], read_mac[4], read_mac[5]);
    xil_printf("IP:  %d.%d.%d.%d\r\n",
               read_ip[0], read_ip[1], read_ip[2], read_ip[3]);
    xil_printf("GW:  %d.%d.%d.%d\r\n",
               read_gw[0], read_gw[1], read_gw[2], read_gw[3]);
    xil_printf("SUB: %d.%d.%d.%d\r\n",
               read_subnet[0], read_subnet[1], read_subnet[2], read_subnet[3]);
    xil_printf("===================================\r\n\r\n");

    /* Create DHCP task */
    xTaskCreate(DHCP_Task, "DHCP", 1024, NULL, tskIDLE_PRIORITY + 2, NULL);
    // xTaskCreate(DHCP_Task, "DHCP", 2048, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(TCP_Server_Task, "TCP_Server", 1024, op, tskIDLE_PRIORITY + 1, NULL);
    // xTaskCreate(TCP_Server_Task, "TCP_Server", 2048, op, tskIDLE_PRIORITY + 1, NULL);

    return;
}

int W5100_Init(W5100_Handle *handle, W5100_Config *config)
{
    int status;

    if (!handle || !config || !config->spi_instance)
    {
        return XST_FAILURE;
    }

    /* Copy configuration */
    memcpy(&handle->config, config, sizeof(W5100_Config));

    /* Create mutex if not provided */
    if (handle->config.spi_mutex == NULL)
    {
        handle->config.spi_mutex = xSemaphoreCreateMutex();
        if (handle->config.spi_mutex == NULL)
        {
            xil_printf("W5100: Failed to create SPI mutex\r\n");
            return XST_FAILURE;
        }
    }

    /* Initialize chip select */
    W5100_InitSS(handle);

    /* Configure SPI for W5100 */
    XSpiPs_Config *spi_config = XSpiPs_LookupConfig(config->spi_instance->Config.BaseAddress);
    if (spi_config == NULL)
    {
        return XST_FAILURE;
    }

    /* Set SPI options: Mode 0 (CPOL=0, CPHA=0), Master mode */
    status = XSpiPs_SetOptions(config->spi_instance,
                               XSPIPS_MASTER_OPTION |
                                   XSPIPS_MANUAL_START_OPTION);
    if (status != XST_SUCCESS)
    {
        xil_printf("W5100: Failed to set SPI options\r\n");
        return status;
    }

    /* Set the SPI device as slave select (CS) */
    status = XSpiPs_SetSlaveSelect(config->spi_instance, 0);
    if (status != XST_SUCCESS)
    {
        xil_printf("W5100: Failed to set slave select\r\n");
        return status;
    }

    /* Set SPI clock prescaler */
    /* PS SPI clock = 166.67 MHz typically, divide to get desired frequency */
    u32 input_clk = XPAR_XSPIPS_0_SPI_CLK_FREQ_HZ;
    u8 prescaler = 0;
    u32 actual_clk = input_clk;

    /* Find appropriate prescaler */
    while (actual_clk > config->spi_clk_hz && prescaler < 6)
    {
        prescaler++;
        actual_clk = input_clk / (2 << prescaler);
    }
    /*
    status = XSpiPs_SetClkPrescaler(config->spi_instance,
                                    XSPIPS_CLK_PRESCALE_256);  */

    status = XSpiPs_SetClkPrescaler(config->spi_instance,
                                    XSPIPS_CLK_PRESCALE_8); /* Prescaler format */
                                                            // XSPIPS_CLK_PRESCALE_32);  /* Prescaler format */
    if (status != XST_SUCCESS)
    {
        xil_printf("W5100: Failed to set SPI clock prescaler\r\n");
        return status;
    }

    /* Software reset */
    W5100_SoftReset(handle);

    /* Detect chip type */
    handle->chip = W5100_DetectChip(handle);

    /* Initialize memory organization for W5100 (2KB per socket) */
    handle->SSIZE = 2048;
    handle->RSIZE = 2048;
    handle->SMASK = 0x07FF;
    handle->RMASK = 0x07FF;

    for (int i = 0; i < W5100_MAX_SOCK_NUM; i++)
    {
        handle->SBASE[i] = W5100_TXBUF_BASE + (W5100_TXBUF_SIZE * i);
        handle->RBASE[i] = W5100_RXBUF_BASE + (W5100_RXBUF_SIZE * i);
    }

    /* Configure RX/TX memory size (2KB each socket) */
    W5100_WriteByte(handle, W5100_RMSR, 0x55); // 2KB per socket
    W5100_WriteByte(handle, W5100_TMSR, 0x55); // 2KB per socket

    handle->initialized = true;

    xil_printf("W5100: Initialized successfully (Chip type: %d)\r\n", handle->chip);

    return XST_SUCCESS;
}

/**
 * @brief Software reset of W5100
 */
int W5100_SoftReset(W5100_Handle *handle)
{
    uint16_t count = 0;

    /* Write reset bit to mode register */
    W5100_WriteByte(handle, W5100_MR, 0x80);

    /* Wait for reset to complete */
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Wait for mode register to clear */
    while (W5100_ReadByte(handle, W5100_MR) != 0x00)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
        if (++count > 20)
        {
            xil_printf("W5100: Soft reset timeout\r\n");
            return XST_FAILURE;
        }
    }

    return XST_SUCCESS;
}

/**
 * @brief Read a single byte from W5100 register
 */
uint8_t W5100_ReadByte(W5100_Handle *handle, uint16_t addr)
{

    uint8_t tx[4] = {
        W5100_SPI_OP_READ,
        (addr >> 8) & 0xFF,
        addr & 0xFF,
        0};
    uint8_t rx[4];

    W5100_SetSS(handle);
    XSpiPs_PolledTransfer(handle->config.spi_instance, tx, rx, 4);
    W5100_ResetSS(handle);
    return rx[3];

    /*
    uint8_t ret;

    W5100_SPIBegin(handle);
    W5100_SetSS(handle);

    W5100_SPITransfer(handle, W5100_SPI_OP_READ);
    W5100_SPITransfer(handle, (addr >> 8) & 0xFF);
    W5100_SPITransfer(handle, addr & 0xFF);
    ret = W5100_SPITransfer(handle, 0);

    W5100_ResetSS(handle);
    W5100_SPIEnd(handle);

    return ret;*/
}

/**
 * @brief Write a single byte to W5100 register
 */
void W5100_WriteByte(W5100_Handle *handle, uint16_t addr, uint8_t data)
{
    /* W5100_SPIBegin(handle);
     W5100_SetSS(handle);

     W5100_SPITransfer(handle, W5100_SPI_OP_WRITE);
     W5100_SPITransfer(handle, (addr >> 8) & 0xFF);
     W5100_SPITransfer(handle, addr & 0xFF);
     W5100_SPITransfer(handle, data);

     W5100_ResetSS(handle);
     W5100_SPIEnd(handle);

     */

    uint8_t tx[4] = {
        W5100_SPI_OP_WRITE,
        (addr >> 8) & 0xFF,
        addr & 0xFF,
        data};
    uint8_t rx[4];
    W5100_SetSS(handle);
    XSpiPs_PolledTransfer(handle->config.spi_instance, tx, rx, 4);
    W5100_ResetSS(handle);
}

/**
 * @brief Read multiple bytes from W5100
 */
void W5100_ReadBuffer(W5100_Handle *handle, uint16_t addr, uint8_t *buf, uint16_t len)
{
    /*W5100_SPIBegin(handle);
    W5100_SetSS(handle);

    W5100_SPITransfer(handle, W5100_SPI_OP_READ);
    W5100_SPITransfer(handle, (addr >> 8) & 0xFF);
    W5100_SPITransfer(handle, addr & 0xFF);

    for (uint16_t i = 0; i < len; i++) {
        buf[i] = W5100_SPITransfer(handle, 0);
    }

    W5100_ResetSS(handle);
    W5100_SPIEnd(handle);*/
    
    uint8_t *tx;
    tx = calloc(3 + len, sizeof(uint8_t));
    uint8_t *rx;
    rx = calloc(3 + len, sizeof(uint8_t));


    tx[0] = W5100_SPI_OP_READ;
    tx[1] = addr >> 8;
    tx[2] = addr & 0xFF;
    memset(&tx[3], 0, len);
    W5100_SetSS(handle);
    XSpiPs_PolledTransfer(handle->config.spi_instance, tx, rx, 3 + len);
    W5100_ResetSS(handle);
    memcpy(buf, &rx[3], len);
}
/**
 * @brief Write multiple bytes to W5100
 */
void W5100_WriteBuffer(W5100_Handle *handle, uint16_t addr, const uint8_t *buf, uint16_t len)
{
    /*
    W5100_SPIBegin(handle);
    W5100_SetSS(handle);

    W5100_SPITransfer(handle, W5100_SPI_OP_WRITE);
    W5100_SPITransfer(handle, (addr >> 8) & 0xFF);
    W5100_SPITransfer(handle, addr & 0xFF);

    for (uint16_t i = 0; i < len; i++) {
        W5100_SPITransfer(handle, buf[i]);
    }

    W5100_ResetSS(handle);
    W5100_SPIEnd(handle);
    */
    uint8_t *tx;
    tx = calloc(3 + len, sizeof(uint8_t));
    uint8_t *rx;
    rx = calloc(3 + len, sizeof(uint8_t));

    tx[0] = W5100_SPI_OP_WRITE;
    tx[1] = addr >> 8;
    tx[2] = addr & 0xFF;
    memcpy(&tx[3], buf, len);
    W5100_SetSS(handle);
    XSpiPs_PolledTransfer(handle->config.spi_instance, tx, rx, 3 + len);
    W5100_ResetSS(handle);
}

/**
 * @brief Read 16-bit word from W5100
 */
uint16_t W5100_ReadWord(W5100_Handle *handle, uint16_t addr)
{

    uint8_t tx[5];
    uint8_t rx[5];

    tx[0] = W5100_SPI_OP_READ;
    tx[1] = addr >> 8;
    tx[2] = addr & 0xFF;
    tx[3] = 0;
    tx[4] = 0;
    W5100_SetSS(handle);
    XSpiPs_PolledTransfer(handle->config.spi_instance, tx, rx, 5);
    W5100_ResetSS(handle);
    uint16_t ret;

    ret = (rx[3] << 8) | rx[4];
    return ret;
    /*
    uint16_t ret;

    W5100_SPIBegin(handle);
    W5100_SetSS(handle);

    W5100_SPITransfer(handle, W5100_SPI_OP_READ);
    W5100_SPITransfer(handle, (addr >> 8) & 0xFF);
    W5100_SPITransfer(handle, addr & 0xFF);
    ret = W5100_SPITransfer(handle, 0) << 8;
    ret |= W5100_SPITransfer(handle, 0);

    W5100_ResetSS(handle);
    W5100_SPIEnd(handle);

    return ret;*/
}

/**
 * @brief Write 16-bit word to W5100
 */
void W5100_WriteWord(W5100_Handle *handle, uint16_t addr, uint16_t data)
{
    uint8_t tx[5] = {
        W5100_SPI_OP_WRITE,
        (addr >> 8) & 0xFF,
        addr & 0xFF,
        (data >> 8) & 0xFF,
        data & 0xFF};
    uint8_t rx[5];
    W5100_SetSS(handle);
    XSpiPs_PolledTransfer(handle->config.spi_instance, tx, rx, 5);
    W5100_ResetSS(handle);
    /*
        W5100_SPIBegin(handle);
        W5100_SetSS(handle);

        W5100_SPITransfer(handle, W5100_SPI_OP_WRITE);
        W5100_SPITransfer(handle, (addr >> 8) & 0xFF);
        W5100_SPITransfer(handle, addr & 0xFF);
        W5100_SPITransfer(handle, (data >> 8) & 0xFF);
        W5100_SPITransfer(handle, data & 0xFF);

        W5100_ResetSS(handle);
        W5100_SPIEnd(handle);*/
}

/**
 * @brief Execute socket command
 */
void W5100_ExecCmdSn(W5100_Handle *handle, uint8_t sock, uint8_t cmd)
{
    uint16_t base = W5100_S0_BASE + (sock * 0x0100);

    W5100_WriteByte(handle, base + W5100_Sn_CR, cmd);

    /* Wait for command to complete */
    while (W5100_ReadByte(handle, base + W5100_Sn_CR))
    {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/**
 * @brief Get socket TX free size
 */
uint16_t W5100_GetTXFreeSize(W5100_Handle *handle, uint8_t sock)
{
    uint16_t base = W5100_S0_BASE + (sock * 0x0100);
    uint16_t val1, val2;

    do
    {
        val1 = W5100_ReadWord(handle, base + W5100_Sn_TX_FSR);
        if (val1 != 0)
        {
            val2 = W5100_ReadWord(handle, base + W5100_Sn_TX_FSR);
        }
    } while (val1 != val2);

    return val1;
}

/**
 * @brief Get socket RX received size
 */
uint16_t W5100_GetRXReceivedSize(W5100_Handle *handle, uint8_t sock)
{
    uint16_t base = W5100_S0_BASE + (sock * 0x0100);
    uint16_t val1, val2;

    do
    {
        val1 = W5100_ReadWord(handle, base + W5100_Sn_RX_RSR);
        if (val1 != 0)
        {
            val2 = W5100_ReadWord(handle, base + W5100_Sn_RX_RSR);
        }
    } while (val1 != val2);

    return val1;
}

/**
 * @brief Send data buffer to socket TX buffer
 */
void W5100_SendData(W5100_Handle *handle, uint8_t sock, const uint8_t *buf, uint16_t len)
{
    uint16_t base = W5100_S0_BASE + (sock * 0x0100);
    uint16_t ptr = W5100_ReadWord(handle, base + W5100_Sn_TX_WR);
    uint16_t offset = ptr & handle->SMASK;
    uint16_t dstAddr = offset + handle->SBASE[sock];

    if (offset + len > handle->SSIZE)
    {
        /* Wrap around buffer */
        uint16_t size = handle->SSIZE - offset;
        W5100_WriteBuffer(handle, dstAddr, buf, size);
        W5100_WriteBuffer(handle, handle->SBASE[sock], buf + size, len - size);
    }
    else
    {
        W5100_WriteBuffer(handle, dstAddr, buf, len);
    }

    ptr += len;
    W5100_WriteWord(handle, base + W5100_Sn_TX_WR, ptr);
}

/**
 * @brief Receive data from socket RX buffer
 */
void W5100_RecvData(W5100_Handle *handle, uint8_t sock, uint8_t *buf, uint16_t len)
{
    uint16_t base = W5100_S0_BASE + (sock * 0x0100);
    uint16_t ptr = W5100_ReadWord(handle, base + W5100_Sn_RX_RD);
    uint16_t offset = ptr & handle->RMASK;
    uint16_t srcAddr = offset + handle->RBASE[sock];

    if (offset + len > handle->RSIZE)
    {
        /* Wrap around buffer */
        uint16_t size = handle->RSIZE - offset;
        W5100_ReadBuffer(handle, srcAddr, buf, size);
        W5100_ReadBuffer(handle, handle->RBASE[sock], buf + size, len - size);
    }
    else
    {
        W5100_ReadBuffer(handle, srcAddr, buf, len);
    }

    ptr += len;
    W5100_WriteWord(handle, base + W5100_Sn_RX_RD, ptr);
}

/**
 * @brief Set gateway IP address
 */
void W5100_SetGatewayIP(W5100_Handle *handle, const uint8_t *addr)
{
    W5100_WriteBuffer(handle, W5100_GAR, addr, 4);
}

/**
 * @brief Set subnet mask
 */
void W5100_SetSubnetMask(W5100_Handle *handle, const uint8_t *addr)
{
    W5100_WriteBuffer(handle, W5100_SUBR, addr, 4);
}

/**
 * @brief Set MAC address
 */
void W5100_SetMACAddress(W5100_Handle *handle, const uint8_t *addr)
{
    W5100_WriteBuffer(handle, W5100_SHAR, addr, 6);
}

/**
 * @brief Set IP address
 */
void W5100_SetIPAddress(W5100_Handle *handle, const uint8_t *addr)
{
    W5100_WriteBuffer(handle, W5100_SIPR, addr, 4);
}

/**
 * @brief Get gateway IP address
 */
void W5100_GetGatewayIP(W5100_Handle *handle, uint8_t *addr)
{
    W5100_ReadBuffer(handle, W5100_GAR, addr, 4);
}

/**
 * @brief Get subnet mask
 */
void W5100_GetSubnetMask(W5100_Handle *handle, uint8_t *addr)
{
    W5100_ReadBuffer(handle, W5100_SUBR, addr, 4);
}

/**
 * @brief Get MAC address
 */
void W5100_GetMACAddress(W5100_Handle *handle, uint8_t *addr)
{
    W5100_ReadBuffer(handle, W5100_SHAR, addr, 6);
}

/**
 * @brief Get IP address
 */
void W5100_GetIPAddress(W5100_Handle *handle, uint8_t *addr)
{
    W5100_ReadBuffer(handle, W5100_SIPR, addr, 4);
}

/**
 * @brief Set retry time
 */
void W5100_SetRetryTime(W5100_Handle *handle, uint16_t timeout)
{
    W5100_WriteWord(handle, W5100_RTR, timeout);
}

/**
 * @brief Set retry count
 */
void W5100_SetRetryCount(W5100_Handle *handle, uint8_t count)
{
    W5100_WriteByte(handle, W5100_RCR, count);
}

/* Private Functions */

/**
 * @brief Begin SPI transaction (acquire mutex)
 */
static void W5100_SPIBegin(W5100_Handle *handle)
{
    if (handle->config.spi_mutex != NULL)
    {
        xSemaphoreTake(handle->config.spi_mutex, portMAX_DELAY);
    }
}

/**
 * @brief End SPI transaction (release mutex)
 */
static void W5100_SPIEnd(W5100_Handle *handle)
{
    if (handle->config.spi_mutex != NULL)
    {
        xSemaphoreGive(handle->config.spi_mutex);
    }
}

/**
 * @brief Transfer a byte via SPI
 */
static uint8_t W5100_SPITransfer(W5100_Handle *handle, uint8_t data)
{
    uint8_t recv_byte;

    /* Polled transfer for single byte */
    XSpiPs_PolledTransfer(handle->config.spi_instance, &data, &recv_byte, 1);

    return recv_byte;
}

/**
 * @brief Initialize chip select pin
 */
static void W5100_InitSS(W5100_Handle *handle)
{
    if (handle->config.use_hardware_cs)
    {
        /* Hardware CS is managed by SPI controller */
        return;
    }

    /* For manual GPIO CS, initialize GPIO here */
    /* This would require XGpioPs driver initialization */
    /* Left as an exercise since it depends on your hardware setup */
}

/**
 * @brief Assert chip select (active low)
 */
static void W5100_SetSS(W5100_Handle *handle)
{
    if (handle->config.use_hardware_cs)
    {
        /* Hardware manages CS automatically */
        return;
    }

    /* For manual GPIO CS */
    /* XGpioPs_WritePin(&gpio_instance, handle->config.cs_pin, 0); */

    XGpio_DiscreteWrite(&csGpio, 1, 0);
}

/**
 * @brief Deassert chip select
 */
static void W5100_ResetSS(W5100_Handle *handle)
{
    if (handle->config.use_hardware_cs)
    {
        /* Hardware manages CS automatically */
        return;
    }

    /* For manual GPIO CS */
    /* XGpioPs_WritePin(&gpio_instance, handle->config.cs_pin, 1); */
    XGpio_DiscreteWrite(&csGpio, 1, 1);
}

/**
 * @brief Detect W5x00 chip type
 */
static W5100_ChipType W5100_DetectChip(W5100_Handle *handle)
{
    uint8_t chip_version;

    /* Try to read version register (W5500 specific) */
    chip_version = W5100_ReadByte(handle, 0x0039);

    if (chip_version == 0x04)
    {
        return W5100_CHIP_W5500;
    }
    else if (chip_version == 0x03)
    {
        return W5100_CHIP_W5200;
    }

    /* Assume W5100 if version register doesn't match */
    return W5100_CHIP_W5100;
}

static DHCP_Handle dhcp_handle;

static void DHCP_Task(void *pvParameters)
{
    int status;

    xil_printf("\r\n=== DHCP Task Started ===\r\n");

    /* Initialize DHCP client on socket 0 */
    status = DHCP_Init(&dhcp_handle, &w5100_handle, 0, mac_addr);
    if (status != XST_SUCCESS)
    {
        xil_printf("DHCP initialization failed\r\n");
        vTaskDelete(NULL);
        return;
    }

    /* Start DHCP process */
    status = DHCP_Start(&dhcp_handle);
    if (status != XST_SUCCESS)
    {
        xil_printf("Failed to start DHCP\r\n");
        vTaskDelete(NULL);
        return;
    }

    /* Main DHCP processing loop */
    while (1)
    {
        /* Process DHCP state machine */
        status = DHCP_Process(&dhcp_handle);

        DHCP_State state = DHCP_GetState(&dhcp_handle);

        if (state == DHCP_STATE_BOUND)
        {
            /* Successfully obtained IP address */
            xil_printf("DHCP: IP configuration obtained!\r\n");

            /* You can now use the network */
            /* For example, start a TCP server or client task here */

            /* Check for renewal every minute */
            while (state == DHCP_STATE_BOUND)
            {
                vTaskDelay(pdMS_TO_TICKS(60000)); // Check every minute

                if (DHCP_NeedsRenewal(&dhcp_handle))
                {
                    xil_printf("DHCP: Time to renew lease\r\n");
                    DHCP_Renew(&dhcp_handle);
                }

                state = DHCP_GetState(&dhcp_handle);
            }
        }
        else if (state == DHCP_STATE_FAILED)
        {
            xil_printf("DHCP: Failed to obtain IP address\r\n");
            xil_printf("DHCP: Retrying in 10 seconds...\r\n");

            vTaskDelay(pdMS_TO_TICKS(10000));

            /* Restart DHCP */
            DHCP_Start(&dhcp_handle);
        }

        /* Process more frequently while acquiring address */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static inline uint32_t read_u32_be(const uint8_t *b)
{
    return ((uint32_t)b[0] << 24) |
           ((uint32_t)b[1] << 16) |
           ((uint32_t)b[2] << 8) |
           ((uint32_t)b[3]);
}

/**
 * @brief Read big-endian 16-bit signed integer
 */
static inline int16_t read_i16_be(const uint8_t *b)
{
    return (int16_t)(((uint16_t)b[0] << 8) | b[1]);
}

/**
 * @brief Unpack 13-bit values from packed byte array
 * @param packed: Input packed bytes (13 bytes per 8 values)
 * @param unpacked: Output array for 13-bit values
 * @param num_values: Number of values to unpack
 *
 * Packing format: 8 values (v0..v7) of 13 bits each packed into 13 bytes LSB-first
 */
static void unpack_13bit_values(const uint8_t *packed, uint16_t *unpacked, uint32_t num_values)
{
    uint32_t num_groups = (num_values + 7) / 8; // ceil(num_values / 8)
    uint32_t value_idx = 0;

    for (uint32_t group = 0; group < num_groups; group++)
    {
        const uint8_t *group_data = &packed[group * 13];

        // Unpack 8 values from 13 bytes (104 bits)
        // Each value is 13 bits, packed LSB-first

        uint64_t bits = 0;

        // Load 13 bytes into a 64-bit buffer (we only use lower 104 bits)
        for (int i = 0; i < 13 && i < 8; i++)
        {
            bits |= ((uint64_t)group_data[i]) << (i * 8);
        }
        // Handle remaining bytes if any
        for (int i = 8; i < 13; i++)
        {
            bits |= ((uint64_t)group_data[i]) << (i * 8);
        }

        // Extract 8 values of 13 bits each
        for (int i = 0; i < 8 && value_idx < num_values; i++)
        {
            unpacked[value_idx++] = (uint16_t)(bits & 0x1FFF); // Mask 13 bits
            bits >>= 13;
        }
    }
}

/**
 * @brief Process command 0x0002: Vector + Header
 * @param op: Operation control structure
 * @param data: Complete command data (header + vector)
 * @param total_length: Total length of data
 * @return Response code
 */
/*
static uint8_t process_vector_header_cmd(operation_control_t *op, const uint8_t *data, uint32_t total_length)
{
   // Minimum length check (header only = 19 bytes)
   if (total_length < 19) {
       xil_printf("ERROR: Command 0x0002 too short (%d bytes)\r\n", total_length);
       return RESPONSE_ERROR_VECTOR_TOO_LARGE;
   }

   // Parse header (skip first 2 bytes which are the command code)
   uint8_t ring = data[2];
   uint8_t em_idx = data[3];
   uint8_t mode = data[4];
   uint8_t type = data[5];
   uint8_t duty = data[6];
   uint8_t freq_val = data[7];
   int16_t offset = read_i16_be(&data[8]);
   int16_t amplitude = read_i16_be(&data[10]);
   uint8_t static_mode = data[12];
   uint8_t frot_val = data[13];
   uint8_t cols = data[14];
   uint32_t vec_len = read_u32_be(&data[15]);

   // Validate parameters
   if (ring > 2) {
       xil_printf("ERROR: Invalid ring %d (must be 0-2)\r\n", ring);
       return RESPONSE_ERROR_INVALID_RING;
   }

   if (em_idx > 9) {
       xil_printf("ERROR: Invalid EM index %d (must be 0-9)\r\n", em_idx);
       return RESPONSE_ERROR_INVALID_EM;
   }

   if (mode > 1) {
       xil_printf("ERROR: Invalid mode %d (must be 0-1)\r\n", mode);
       return RESPONSE_ERROR_INVALID_MODE;
   }

   if (vec_len > MAX_SAMPLES_PER_EM) {
       xil_printf("ERROR: Vector too large %d (max %d)\r\n", vec_len, MAX_SAMPLES_PER_EM);
       return RESPONSE_ERROR_VECTOR_TOO_LARGE;
   }

   // Calculate expected vector size in bytes
   uint32_t num_groups = (vec_len + 7) / 8;  // ceil(vec_len / 8)
   uint32_t expected_vector_bytes = num_groups * 13;
   uint32_t expected_total_bytes = 19 + expected_vector_bytes;

   if (total_length < expected_total_bytes) {
       xil_printf("ERROR: Incomplete vector data. Expected %d bytes, got %d\r\n",
                  expected_total_bytes, total_length);
       return RESPONSE_ERROR_VECTOR_TOO_LARGE;
   }

   // Calculate actual EM array index (ring * 10 + em_idx)
   uint8_t em_array_idx = ring * 10 + em_idx;

   if (em_array_idx >= NUM_EM) {
       xil_printf("ERROR: Computed EM index %d exceeds array size %d\r\n",
                  em_array_idx, NUM_EM);
       return RESPONSE_ERROR_INVALID_EM;
   }

   xil_printf("=== Parsing Vector Header Command ===\r\n");
   xil_printf("Ring: %d, EM: %d (Array index: %d)\r\n", ring, em_idx, em_array_idx);
   xil_printf("Mode: %d, Type: %d, Duty: %d%%\r\n", mode, type, duty);
   xil_printf("Freq: %.1f Hz, Offset: %d, Amplitude: %d\r\n",
              freq_val / 10.0, offset, amplitude);
   xil_printf("Static mode: %d, Rot freq: %.1f Hz, Cols: %d\r\n",
              static_mode, frot_val / 10.0, cols);
   xil_printf("Vector length: %d values (%d bytes packed)\r\n",
              vec_len, expected_vector_bytes);

   // Acquire mutex for thread-safe access
   xSemaphoreTake(op->mutex, portMAX_DELAY);

   // Update EM configuration
   em_t *em = &op->em[em_array_idx];

   em->ring_idx = ring;
   em->em_idx = em_idx;
   em->mode = mode;
   em->type = type;
   em->duty_cycle = duty;
   em->frequency_factor = freq_val;
   em->magnetic_field_offset = offset;
   em->magnetic_field_amplitude = amplitude;
   em->static_mode = static_mode;
   em->rotational_frequency_factor = frot_val;
   em->coils_columns = cols;
   em->vector_length = vec_len;
   em->playbackIndex = 0;

   // Store packed vector data directly (no unpacking to save memory)
   if (vec_len > 0 && em->em_ctrl != NULL) {
       // Copy packed data directly to em_ctrl buffer
       memcpy(em->em_ctrl, &data[19], expected_vector_bytes);

       xil_printf("Stored %d bytes of packed vector data\r\n", expected_vector_bytes);
   }

   xSemaphoreGive(op->mutex);

   xil_printf("=== Vector Header Command Complete ===\r\n");

   return RESPONSE_SUCCESS;
}
*/

/**
 * @brief Process command 0x0002: Vector + Header
 * @param op: Operation control structure
 * @param data: Complete command data (header + vector)
 * @param total_length: Total length of data
 * @return Response code
 */
static uint8_t process_vector_header_cmd(operation_control_t *op,
                                         const uint8_t *data,
                                         uint32_t total_length)
{
    // Minimum length check (header only = 19 bytes)
    if (total_length < 19)
    {
        xil_printf("ERROR: Command 0x0002 too short (%d bytes)\r\n", total_length);
        return RESPONSE_ERROR_VECTOR_TOO_LARGE;
    }

    // Parse header (skip first 2 bytes which are the command code)
    uint8_t ring = data[2];
    uint8_t em_idx = data[3];
    uint8_t mode = data[4];
    uint8_t type = data[5];
    uint8_t duty = data[6];
    uint8_t freq_val = data[7];
    int16_t offset = read_i16_be(&data[8]);
    int16_t amplitude = read_i16_be(&data[10]);
    uint8_t static_mode = data[12];
    uint8_t frot_val = data[13];
    uint8_t cols = data[14];
    uint32_t vec_len = read_u32_be(&data[15]);

    // Validate parameters
    if (ring > 2)
    {
        xil_printf("ERROR: Invalid ring %d (must be 0-2)\r\n", ring);
        return RESPONSE_ERROR_INVALID_RING;
    }

    if (em_idx > 9)
    {
        xil_printf("ERROR: Invalid EM index %d (must be 0-9)\r\n", em_idx);
        return RESPONSE_ERROR_INVALID_EM;
    }

    if (mode > 1)
    {
        xil_printf("ERROR: Invalid mode %d (must be 0-1)\r\n", mode);
        return RESPONSE_ERROR_INVALID_MODE;
    }

    if (vec_len > MAX_SAMPLES_PER_EM)
    {
        xil_printf("ERROR: Vector too large %d (max %d)\r\n",
                   vec_len, MAX_SAMPLES_PER_EM);
        return RESPONSE_ERROR_VECTOR_TOO_LARGE;
    }

    // Calculate expected vector size in bytes
    uint32_t num_groups = (vec_len + 7) / 8; // ceil(vec_len / 8)
    uint32_t expected_vector_bytes = num_groups * 13;
    uint32_t expected_total_bytes = 19 + expected_vector_bytes;

    if (total_length < expected_total_bytes)
    {
        xil_printf("ERROR: Incomplete vector data. Expected %d bytes, got %d\r\n",
                   expected_total_bytes, total_length);
        return RESPONSE_ERROR_VECTOR_TOO_LARGE;
    }

    // Calculate actual EM array index (ring * 10 + em_idx)
    uint8_t em_array_idx = ring * 10 + em_idx;

    if (em_array_idx >= NUM_EM)
    {
        xil_printf("ERROR: Computed EM index %d exceeds array size %d\r\n",
                   em_array_idx, NUM_EM);
        return RESPONSE_ERROR_INVALID_EM;
    }

    xil_printf("=== Parsing Vector Header Command ===\r\n");
    xil_printf("Ring: %d, EM: %d (Array index: %d)\r\n",
               ring, em_idx, em_array_idx);
    xil_printf("Mode: %d, Type: %d, Duty: %d%%\r\n", mode, type, duty);
    xil_printf("Freq: %.1f Hz, Offset: %d, Amplitude: %d\r\n",
               freq_val / 10.0, offset, amplitude);
    xil_printf("Static mode: %d, Rot freq: %.1f Hz, Cols: %d\r\n",
               static_mode, frot_val / 10.0, cols);
    xil_printf("Vector length: %d values (%d bytes packed)\r\n",
               vec_len, expected_vector_bytes);

    // Acquire mutex for thread-safe access
    xSemaphoreTake(op->mutex, portMAX_DELAY);

    // Update EM configuration
    em_t *em = &op->em[em_array_idx];

    em->ring_idx = ring;
    em->em_idx = em_idx;
    em->mode = mode;
    em->type = type;
    em->duty_cycle = duty;
    em->frequency_factor = freq_val;
    em->magnetic_field_offset = offset;
    em->magnetic_field_amplitude = amplitude;
    em->static_mode = static_mode;
    em->rotational_frequency_factor = frot_val;
    em->coils_columns = cols;
    em->playbackIndex = 0;

    // Store packed vector data and loop it in-place
    if (vec_len > 0 && em->em_ctrl != NULL)
    {

        uint8_t *buf = em->em_ctrl;

        uint32_t original_samples = vec_len;
        uint32_t target_samples = MAX_SAMPLES_PER_EM; // EM_PLAYBACK_SAMPLES;

        // if (target_samples > MAX_SAMPLES_PER_EM) {
        //     target_samples = MAX_SAMPLES_PER_EM;
        // }

        uint32_t original_bytes =
            ((original_samples * 13) + 7) >> 3;

        // 1) Copy received packed data
        memcpy(buf, &data[19], original_bytes);

        // 2) Loop / expand samples in-place
        for (uint32_t i = original_samples; i < target_samples; i++)
        {
            int16_t v = get13s(buf, i % original_samples);
            put13s(buf, i, v);
        }

        // 3) Update vector length to expanded size
        em->vector_length = target_samples;

        uint32_t total_bytes =
            ((target_samples * 13) + 7) >> 3;

        xil_printf("Stored %d samples (%d bytes packed, looped from %d samples)\r\n",
                   target_samples, total_bytes, original_samples);
    }

    xSemaphoreGive(op->mutex);

    xil_printf("=== Vector Header Command Complete ===\r\n");

    return RESPONSE_SUCCESS;
}

/**
 * @brief Example TCP Server Task (starts after DHCP succeeds)
 */

static void TCP_Server_Task(void *pvParameters)
{
    operation_control_t *op =
        (operation_control_t *)pvParameters;
    
    uint8_t sock = 1;
    uint16_t server_port = 5005;
    uint16_t base = W5100_S0_BASE + (sock * 0x0100);
    uint8_t recv_buf[1500];
    uint8_t response[128];
    uint8_t response_length = 0;
    uint32_t tmp_wrapper_pkt_index = 0;
    uint32_t wrapper_pkt_index = 0;
    uint16_t tmp_wrapper_total_pkts = 0;
    uint16_t tmp_wrapper_pkt_length = 0;
    uint32_t wrapper_pkt_length = 0;
    bool wrapped_cmd_available = false;
    uint8_t packet_type;
    uint16_t recv_size = 0;
    uint16_t cmd = 0;
    // Wait for DHCP to complete
    while (DHCP_GetState(&dhcp_handle) != DHCP_STATE_BOUND)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    xil_printf("TCP Server: Starting on port %d\r\n", server_port);

    // Open socket
    W5100_WriteByte(&w5100_handle, base + W5100_Sn_MR, W5100_SnMR_TCP);
    W5100_WriteWord(&w5100_handle, base + W5100_Sn_PORT, server_port);
    W5100_ExecCmdSn(&w5100_handle, sock, W5100_SOCK_OPEN);

    // Wait for open
    vTaskDelay(pdMS_TO_TICKS(100));

    // Listen
    W5100_ExecCmdSn(&w5100_handle, sock, W5100_SOCK_LISTEN);

    xil_printf("TCP Server: Listening for connections...\r\n");

    while (1)
    {
        
        uint8_t status = W5100_ReadByte(&w5100_handle, base + W5100_Sn_SR);
        switch (status)
        {
        case W5100_SNSR_ESTABLISHED:
        
            // Print needed allow the firmware to accept new connections after disconnect
            // xil_printf("TCP Server: W5100_SNSR_ESTABLISHED\r\n");
            xil_printf("\a\a\a\a\a\a\a\a\a\a\a\a\a\a\a\a\a\a\a\a\a\a\a\a\a\a\a\a\a\a\a\a\a\a\a\a");
            // Client connected
            recv_size = W5100_GetRXReceivedSize(&w5100_handle, sock);

            if (recv_size > 0)
            {
                if (recv_size > sizeof(recv_buf) - 1)
                {
                    // could be a problem and should be handle differently
                    recv_size = sizeof(recv_buf) - 1;
                }

                W5100_RecvData(&w5100_handle, sock, recv_buf, recv_size);
                W5100_ExecCmdSn(&w5100_handle, sock, W5100_SOCK_RECV);

                // recv_buf[recv_size] = '\0';
                cmd = ((uint16_t)recv_buf[0] << 8) | recv_buf[1];

                // xil_printf("Received: %s\r\n", recv_buf);

                switch (cmd)
                {
                case CMD_CODE_PLAY:
                    // PLAY
                    xil_printf("Play cmd\r\n");
                    xSemaphoreTake(op->mutex, portMAX_DELAY);
                    op->cmd = CMD_PLAY;
                    xSemaphoreGive(op->mutex);

                    response[0] = RESPONSE_SUCCESS;
                    response_length = 1;
                    break;

                case CMD_CODE_PAUSE:
                    // PAUSE
                    xil_printf("Pause cmd\r\n");
                    xSemaphoreTake(op->mutex, portMAX_DELAY);
                    op->cmd = CMD_PAUSE;
                    xSemaphoreGive(op->mutex);

                    response[0] = RESPONSE_SUCCESS;
                    response_length = 1;
                    break;

                case CMD_CODE_STOP:
                    // STOP
                    xil_printf("Stop cmd\r\n");
                    xSemaphoreTake(op->mutex, portMAX_DELAY);
                    op->cmd = CMD_STOP;
                    xSemaphoreGive(op->mutex);

                    response[0] = RESPONSE_SUCCESS;
                    response_length = 1;
                    break;

                case CMD_CODE_CONFIG:
                    // Config
                    xil_printf("Config cmd\r\n");
                    xSemaphoreTake(op->mutex, portMAX_DELAY);
                    op->operationTime = read_u32_be(&recv_buf[2]);
                    op->delayTime = read_u32_be(&recv_buf[6]);
                    op->signalSamplePeriodMs = read_u32_be(&recv_buf[10]);
                    xSemaphoreGive(op->mutex);
                    xil_printf("operationTime: %d, delayTime: %d, signalSamplePeriodMs: %d\r\n", op->operationTime, op->delayTime, op->signalSamplePeriodMs);

                    response[0] = RESPONSE_SUCCESS;
                    response_length = 1;
                    break;
                case CMD_CODE_DISCOVER:
                    // DISCOVER
                    xil_printf("Discover cmd\r\n");
                    response[0] = RESPONSE_SUCCESS;
                    response[1] = (device_internal_id >> 24) & 0xFF;
                    response[2] = (device_internal_id >> 16) & 0xFF;
                    response[3] = (device_internal_id >> 8) & 0xFF;
                    response[4] = device_internal_id & 0xFF;
                    response[5] = VERSION_MAYOR;
                    response[6] = VERSION_MINOR;
                    response[7] = VERSION_PATCH;
                    response[8] = op->currentState;
                    response[9] = op->currentStage;
                    memcpy(response+10,(uint8_t*) & op->operationTime, sizeof(op->operationTime));
                    memcpy(response+14,(uint8_t*) & op->delayTime, sizeof(op->delayTime));
                    memcpy(response+18,(uint8_t*) & op->generalPlaybackIndex, sizeof(op->delayTime));
                    response_length = 22;
                    break;
                case CMD_CODE_READ_STATE:
                    xil_printf("Read state cmd\r\n");

                    break;

                // New parsing for wrapper header (WRAPPER_HEADER_SIZE = 11)
                case CMD_CODE_WRAPPER:
                    tmp_wrapper_pkt_index = ((uint32_t)recv_buf[3] << 24) |
                                            ((uint32_t)recv_buf[4] << 16) |
                                            ((uint32_t)recv_buf[5] << 8) |
                                            (uint32_t)recv_buf[6];
                    tmp_wrapper_total_pkts = ((uint16_t)recv_buf[7] << 8) | recv_buf[8];
                    tmp_wrapper_pkt_length = ((uint16_t)recv_buf[9] << 8) | recv_buf[10];
                    packet_type = recv_buf[2];

                    // // Use 32-bit state for offsets and lengths
                    // static uint32_t wrapper_pkt_index = 0;
                    // static uint32_t wrapper_pkt_length = 0;

                    // Handle START packet - reset state
                    if (packet_type == WRAPPER_START_PKT)
                    {
                        wrapper_pkt_index = 0;
                        wrapper_pkt_length = 0;
                        xil_printf("Wrapper START packet, length: %u\r\n", tmp_wrapper_pkt_length);
                    }
                    else
                    {
                        // Expected index is previous index + previous packet length
                        uint32_t expected_index = wrapper_pkt_index + wrapper_pkt_length;
                        if (tmp_wrapper_pkt_index != expected_index)
                        {
                            xil_printf("ERROR: Index mismatch. Expected: %u, Got: %u\r\n",
                                       expected_index, tmp_wrapper_pkt_index);
                            response[0] = RESPONSE_ERROR_WRONG_IDX_WRAPPER_PKT;
                            response_length = 1;
                            break;
                        }
                    }

                    // Check bounds before memcpy (ensure no overflow)
                    if ((uint64_t)tmp_wrapper_pkt_index + (uint64_t)tmp_wrapper_pkt_length > sizeof(large_rx_buffer))
                    {
                        xil_printf("ERROR: Buffer overflow! Index: %u, Length: %u, Buffer size: %u\r\n",
                                   tmp_wrapper_pkt_index, tmp_wrapper_pkt_length, (uint32_t)sizeof(large_rx_buffer));
                        response[0] = RESPONSE_ERROR_VECTOR_TOO_LARGE;
                        response_length = 1;
                        break;
                    }

                    // Copy data to large buffer
                    memcpy(&large_rx_buffer[tmp_wrapper_pkt_index],
                           &recv_buf[WRAPPER_HEADER_SIZE],
                           tmp_wrapper_pkt_length);

                    // Update state for next packet
                    wrapper_pkt_index = tmp_wrapper_pkt_index;
                    wrapper_pkt_length = tmp_wrapper_pkt_length;

                    // Mark command available if this is the END packet
                    if (packet_type == WRAPPER_END_PKT)
                    {
                        wrapped_cmd_available = true;
                        xil_printf("Wrapper END packet. Total data: %u bytes\r\n",
                                   wrapper_pkt_index + wrapper_pkt_length);
                    }

                    response[0] = RESPONSE_SUCCESS;
                    response_length = 1;
                    break;

                default:
                    // Unknown command
                    xil_printf("Unknown command: %04x\r\n", cmd);
                    response_length = 0;
                    break;
                }

                // Echo response
                if (response_length > 0)
                {
                    W5100_SendData(&w5100_handle, sock, (uint8_t *)response, response_length);
                    W5100_ExecCmdSn(&w5100_handle, sock, W5100_SOCK_SEND);
                }
            }

            if (wrapped_cmd_available)
            {
                cmd = ((uint16_t)large_rx_buffer[0] << 8) | large_rx_buffer[1];

                xil_printf("Processing wrapped command: 0x%04X\r\n", cmd);

                switch (cmd)
                {
                case CMD_CODE_VECTOR_HEADER:
                    // Vector + Header command
                    response[0] = process_vector_header_cmd(op, large_rx_buffer, wrapper_pkt_index + wrapper_pkt_length);
                    response_length = 1;
                    break;

                default:
                    xil_printf("Unknown wrapped command: 0x%04X\r\n", cmd);
                    response[0] = RESPONSE_ERROR_INVALID_MODE;
                    response_length = 1;
                    break;
                }

                // Send response
                W5100_SendData(&w5100_handle, sock, response, response_length);
                W5100_ExecCmdSn(&w5100_handle, sock, W5100_SOCK_SEND);

                // Reset wrapper state for next transfer
                wrapped_cmd_available = false;
                wrapper_pkt_index = 0;
                wrapper_pkt_length = 0;
            }
            break;
        
        case W5100_SNSR_CLOSE_WAIT:
            // Client disconnected
            xil_printf("TCP Server: Client disconnected\r\n");
            W5100_ExecCmdSn(&w5100_handle, sock, W5100_SOCK_DISCON);
            vTaskDelay(pdMS_TO_TICKS(100));

            // Reset wrapper state on disconnect
            wrapped_cmd_available = false;
            wrapper_pkt_index = 0;
            wrapper_pkt_length = 0;

            // Reopen and listen
            W5100_WriteByte(&w5100_handle, base + W5100_Sn_MR, W5100_SnMR_TCP);
            W5100_WriteWord(&w5100_handle, base + W5100_Sn_PORT, server_port);
            W5100_ExecCmdSn(&w5100_handle, sock, W5100_SOCK_OPEN);
            vTaskDelay(pdMS_TO_TICKS(100));
            W5100_ExecCmdSn(&w5100_handle, sock, W5100_SOCK_LISTEN);
            xil_printf("TCP Server: Listening again...\r\n");
            break;
        default:
            // Other statuses can be handled as needed
            //xil_printf("TCP Server: UNKNOWN STATUS 0x%02X\r\n", status);
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void expand_looped_13bit_samples(uint8_t *buf,
                                 uint32_t original_samples,
                                 uint32_t total_samples)
{
    for (uint32_t i = original_samples; i < total_samples; i++)
    {
        int16_t v = get13s(buf, i % original_samples);
        put13s(buf, i, v);
    }
}

void put13s(uint8_t *buf, uint32_t index, int16_t value)
{
    uint32_t bit_offset = index * 13;
    uint32_t byte_offset = bit_offset >> 3;
    uint32_t bit_shift = bit_offset & 0x7;

    uint32_t raw = (uint16_t)value & 0x1FFF;

    // Align to MSB side of 32-bit window
    raw <<= (19 - bit_shift);

    uint32_t mask = (uint32_t)0x1FFF << (19 - bit_shift);

    uint32_t cur =
        ((uint32_t)buf[byte_offset] << 24) |
        ((uint32_t)buf[byte_offset + 1] << 16) |
        ((uint32_t)buf[byte_offset + 2] << 8) |
        ((uint32_t)buf[byte_offset + 3]);

    cur = (cur & ~mask) | raw;

    buf[byte_offset] = (cur >> 24) & 0xFF;
    buf[byte_offset + 1] = (cur >> 16) & 0xFF;
    buf[byte_offset + 2] = (cur >> 8) & 0xFF;
    buf[byte_offset + 3] = cur & 0xFF;
}
