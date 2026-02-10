/**
 * @file w5100_dhcp.c
 * @brief DHCP Client Implementation for W5100
 */

#include "W5100dhcp.h"
#include "xil_printf.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdlib.h>

/* DHCP Message Structure */
#define DHCP_HEADER_SIZE    236
#define DHCP_MAGIC_COOKIE   0x63825363

/* Private function prototypes */
static void DHCP_SendDiscover(DHCP_Handle *dhcp);
static void DHCP_SendRequest(DHCP_Handle *dhcp);
static void DHCP_SendRelease(DHCP_Handle *dhcp);
static int DHCP_ProcessMessage(DHCP_Handle *dhcp);
static void DHCP_ParseOptions(DHCP_Handle *dhcp, uint8_t *options, uint16_t len);
static uint32_t DHCP_GetXID(void);
static uint16_t DHCP_GetSocketBase(uint8_t sock);

/* Debug helper - enable/disable with this flag */
#define DHCP_DEBUG_PACKETS 0

#if DHCP_DEBUG_PACKETS
static void DHCP_PrintPacket(const char *prefix, uint8_t *data, uint16_t len)
{
    xil_printf("%s (%d bytes):\r\n", prefix, len);
    for (uint16_t i = 0; i < len && i < 64; i++) {  // Print first 64 bytes
        if (i % 16 == 0) xil_printf("%04X: ", i);
        xil_printf("%02X ", data[i]);
        if ((i + 1) % 16 == 0) xil_printf("\r\n");
    }
    if (len % 16 != 0) xil_printf("\r\n");
}
#else
#define DHCP_PrintPacket(prefix, data, len) /* nothing */
#endif

/**
 * @brief Initialize DHCP client
 */
int DHCP_Init(DHCP_Handle *dhcp, W5100_Handle *w5100, uint8_t socket, const uint8_t *mac)
{
    if (!dhcp || !w5100 || !mac || socket >= W5100_MAX_SOCK_NUM) {
        return XST_FAILURE;
    }
    
    memset(dhcp, 0, sizeof(DHCP_Handle));
    
    dhcp->w5100 = w5100;
    dhcp->socket = socket;
    memcpy(dhcp->mac, mac, 6);
    dhcp->state = DHCP_STATE_INIT;
    dhcp->xid = DHCP_GetXID();
    
    xil_printf("DHCP: Initialized on socket %d\r\n", socket);
    
    return XST_SUCCESS;
}

/**
 * @brief Start DHCP process
 */
int DHCP_Start(DHCP_Handle *dhcp)
{
    uint16_t base = DHCP_GetSocketBase(dhcp->socket);
    
    xil_printf("DHCP: Starting DHCP process...\r\n");
    
    /* Set temporary IP (0.0.0.0) */
    uint8_t temp_ip[4] = {0, 0, 0, 0};
    W5100_SetIPAddress(dhcp->w5100, temp_ip);
    
    /* Close socket if already open */
    uint8_t status = W5100_ReadByte(dhcp->w5100, base + W5100_Sn_SR);
    if (status != W5100_SNSR_CLOSED) {
        W5100_ExecCmdSn(dhcp->w5100, dhcp->socket, W5100_SOCK_CLOSE);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    /* Open UDP socket for DHCP */
    W5100_WriteByte(dhcp->w5100, base + W5100_Sn_MR, W5100_SnMR_UDP);
    W5100_WriteWord(dhcp->w5100, base + W5100_Sn_PORT, DHCP_CLIENT_PORT);
    W5100_ExecCmdSn(dhcp->w5100, dhcp->socket, W5100_SOCK_OPEN);
    
    /* Wait for socket to open */
    int timeout = 100;
    while (timeout-- > 0) {
        status = W5100_ReadByte(dhcp->w5100, base + W5100_Sn_SR);
        if (status == W5100_SNSR_UDP) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    if (status != W5100_SNSR_UDP) {
        xil_printf("DHCP: Failed to open UDP socket\r\n");
        return XST_FAILURE;
    }
    
    xil_printf("DHCP: UDP socket opened\r\n");
    
    /* Send DHCP Discover */
    dhcp->state = DHCP_STATE_SELECTING;
    dhcp->retry_count = 0;
    dhcp->timeout = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
    
    DHCP_SendDiscover(dhcp);
    
    return XST_SUCCESS;
}

/**
 * @brief Process DHCP state machine
 */
int DHCP_Process(DHCP_Handle *dhcp)
{
    uint16_t base = DHCP_GetSocketBase(dhcp->socket);
    uint32_t now = xTaskGetTickCount();
    
    /* Check for timeout */
    if (now > dhcp->timeout) {
        if (dhcp->retry_count >= 3) {
            xil_printf("DHCP: Failed after %d retries\r\n", dhcp->retry_count);
            dhcp->state = DHCP_STATE_FAILED;
            return XST_FAILURE;
        }
        
        dhcp->retry_count++;
        dhcp->timeout = now + pdMS_TO_TICKS(5000);
        
        xil_printf("DHCP: Retry %d\r\n", dhcp->retry_count);
        
        if (dhcp->state == DHCP_STATE_SELECTING) {
            DHCP_SendDiscover(dhcp);
        } else if (dhcp->state == DHCP_STATE_REQUESTING) {
            DHCP_SendRequest(dhcp);
        }
    }
    
    /* Check for received data */
    uint16_t recv_size = W5100_GetRXReceivedSize(dhcp->w5100, dhcp->socket);
    if (recv_size > 0) {
        int result = DHCP_ProcessMessage(dhcp);
        return result;
    }
    
    return XST_SUCCESS;
}

/**
 * @brief Send DHCP Discover message
 */
static void DHCP_SendDiscover(DHCP_Handle *dhcp)
{
    uint8_t buffer[312];  // DHCP header + options
    uint16_t pos = 0;
    uint16_t base = DHCP_GetSocketBase(dhcp->socket);
    
    memset(buffer, 0, sizeof(buffer));
    
    xil_printf("DHCP: Sending DISCOVER (XID: 0x%08X)\r\n", dhcp->xid);
    
    /* DHCP Header */
    buffer[pos++] = 0x01;  // Op: Boot Request
    buffer[pos++] = 0x01;  // HType: Ethernet
    buffer[pos++] = 0x06;  // HLen: 6
    buffer[pos++] = 0x00;  // Hops: 0
    
    /* Transaction ID */
    buffer[pos++] = (dhcp->xid >> 24) & 0xFF;
    buffer[pos++] = (dhcp->xid >> 16) & 0xFF;
    buffer[pos++] = (dhcp->xid >> 8) & 0xFF;
    buffer[pos++] = dhcp->xid & 0xFF;
    
    /* Seconds, Flags */
    pos += 4;  // All zeros
    
    /* Client IP, Your IP, Server IP, Gateway IP */
    pos += 16;  // All zeros
    
    /* Client MAC address */
    memcpy(&buffer[pos], dhcp->mac, 6);
    pos += 16;  // MAC + padding
    
    /* Server hostname, Boot filename */
    pos += 192;  // All zeros
    
    /* Magic cookie */
    buffer[pos++] = 0x63;
    buffer[pos++] = 0x82;
    buffer[pos++] = 0x53;
    buffer[pos++] = 0x63;
    
    /* DHCP Options */
    
    /* Option: DHCP Message Type (Discover) */
    buffer[pos++] = DHCP_OPT_MSG_TYPE;
    buffer[pos++] = 1;
    buffer[pos++] = DHCP_DISCOVER;
    
    /* Option: Client Identifier */
    buffer[pos++] = DHCP_OPT_CLIENT_ID;
    buffer[pos++] = 7;
    buffer[pos++] = 0x01;  // Hardware type: Ethernet
    memcpy(&buffer[pos], dhcp->mac, 6);
    pos += 6;
    
    /* Option: Parameter Request List */
    buffer[pos++] = DHCP_OPT_PARAM_REQUEST;
    buffer[pos++] = 4;
    buffer[pos++] = DHCP_OPT_SUBNET_MASK;
    buffer[pos++] = DHCP_OPT_ROUTER;
    buffer[pos++] = DHCP_OPT_DNS_SERVER;
    buffer[pos++] = DHCP_OPT_DOMAIN_NAME;
    
    /* Option: End */
    buffer[pos++] = DHCP_OPT_END;
    
    /* Set destination IP (broadcast) and port */
    uint8_t broadcast_ip[4] = {255, 255, 255, 255};
    W5100_WriteBuffer(dhcp->w5100, base + W5100_Sn_DIPR, broadcast_ip, 4);
    W5100_WriteWord(dhcp->w5100, base + W5100_Sn_DPORT, DHCP_SERVER_PORT);
    
    /* Send data */
    W5100_SendData(dhcp->w5100, dhcp->socket, buffer, pos);
    W5100_ExecCmdSn(dhcp->w5100, dhcp->socket, W5100_SOCK_SEND);
    
    xil_printf("DHCP: DISCOVER sent (%d bytes)\r\n", pos);
}

/**
 * @brief Send DHCP Request message
 */
static void DHCP_SendRequest(DHCP_Handle *dhcp)
{
    uint8_t buffer[312];
    uint16_t pos = 0;
    uint16_t base = DHCP_GetSocketBase(dhcp->socket);
    
    memset(buffer, 0, sizeof(buffer));
    
    xil_printf("DHCP: Sending REQUEST for IP %d.%d.%d.%d\r\n",
               dhcp->config.assigned_ip[0], dhcp->config.assigned_ip[1],
               dhcp->config.assigned_ip[2], dhcp->config.assigned_ip[3]);
    
    /* DHCP Header */
    buffer[pos++] = 0x01;  // Op: Boot Request
    buffer[pos++] = 0x01;  // HType: Ethernet
    buffer[pos++] = 0x06;  // HLen: 6
    buffer[pos++] = 0x00;  // Hops: 0
    
    /* Transaction ID */
    buffer[pos++] = (dhcp->xid >> 24) & 0xFF;
    buffer[pos++] = (dhcp->xid >> 16) & 0xFF;
    buffer[pos++] = (dhcp->xid >> 8) & 0xFF;
    buffer[pos++] = dhcp->xid & 0xFF;
    
    /* Seconds, Flags */
    pos += 4;
    
    /* Client IP, Your IP, Server IP, Gateway IP */
    pos += 16;
    
    /* Client MAC address */
    memcpy(&buffer[pos], dhcp->mac, 6);
    pos += 16;
    
    /* Server hostname, Boot filename */
    pos += 192;
    
    /* Magic cookie */
    buffer[pos++] = 0x63;
    buffer[pos++] = 0x82;
    buffer[pos++] = 0x53;
    buffer[pos++] = 0x63;
    
    /* DHCP Options */
    
    /* Option: DHCP Message Type (Request) */
    buffer[pos++] = DHCP_OPT_MSG_TYPE;
    buffer[pos++] = 1;
    buffer[pos++] = DHCP_REQUEST;
    
    /* Option: Client Identifier */
    buffer[pos++] = DHCP_OPT_CLIENT_ID;
    buffer[pos++] = 7;
    buffer[pos++] = 0x01;
    memcpy(&buffer[pos], dhcp->mac, 6);
    pos += 6;
    
    /* Option: Requested IP Address */
    buffer[pos++] = DHCP_OPT_REQUESTED_IP;
    buffer[pos++] = 4;
    memcpy(&buffer[pos], dhcp->config.assigned_ip, 4);
    pos += 4;
    
    /* Option: Server Identifier */
    buffer[pos++] = DHCP_OPT_SERVER_ID;
    buffer[pos++] = 4;
    memcpy(&buffer[pos], dhcp->config.dhcp_server, 4);
    pos += 4;
    
    /* Option: Parameter Request List */
    buffer[pos++] = DHCP_OPT_PARAM_REQUEST;
    buffer[pos++] = 4;
    buffer[pos++] = DHCP_OPT_SUBNET_MASK;
    buffer[pos++] = DHCP_OPT_ROUTER;
    buffer[pos++] = DHCP_OPT_DNS_SERVER;
    buffer[pos++] = DHCP_OPT_DOMAIN_NAME;
    
    /* Option: End */
    buffer[pos++] = DHCP_OPT_END;
    
    /* Set destination IP (broadcast) and port */
    uint8_t broadcast_ip[4] = {255, 255, 255, 255};
    W5100_WriteBuffer(dhcp->w5100, base + W5100_Sn_DIPR, broadcast_ip, 4);
    W5100_WriteWord(dhcp->w5100, base + W5100_Sn_DPORT, DHCP_SERVER_PORT);
    
    /* Send data */
    W5100_SendData(dhcp->w5100, dhcp->socket, buffer, pos);
    W5100_ExecCmdSn(dhcp->w5100, dhcp->socket, W5100_SOCK_SEND);
    
    xil_printf("DHCP: REQUEST sent (%d bytes)\r\n", pos);
}

/**
 * @brief Process received DHCP message
 */
static int DHCP_ProcessMessage(DHCP_Handle *dhcp)
{
    uint8_t buffer[548];  // Max DHCP packet size
    uint16_t recv_size = W5100_GetRXReceivedSize(dhcp->w5100, dhcp->socket);
    uint16_t base = DHCP_GetSocketBase(dhcp->socket);
    
    if (recv_size == 0) {
        return XST_SUCCESS;
    }
    
    if (recv_size > sizeof(buffer)) {
        recv_size = sizeof(buffer);
    }
    
    /* For UDP, W5100 stores packets with an 8-byte header:
     * [0-3]: Source IP (4 bytes)
     * [4-5]: Source Port (2 bytes)
     * [6-7]: Data Length (2 bytes)
     * [8+]:  Actual UDP data
     */
    
    /* Read the entire packet including UDP header */
    W5100_RecvData(dhcp->w5100, dhcp->socket, buffer, recv_size);
    W5100_ExecCmdSn(dhcp->w5100, dhcp->socket, W5100_SOCK_RECV);
    
    /* Debug: Print raw packet */
    DHCP_PrintPacket("DHCP Raw Packet", buffer, recv_size > 64 ? 64 : recv_size);
    
    /* Parse UDP header */
    uint8_t src_ip[4];
    uint16_t src_port;
    uint16_t data_len;
    
    src_ip[0] = buffer[0];
    src_ip[1] = buffer[1];
    src_ip[2] = buffer[2];
    src_ip[3] = buffer[3];
    src_port = (buffer[4] << 8) | buffer[5];
    data_len = (buffer[6] << 8) | buffer[7];
    
    xil_printf("DHCP: Received from %d.%d.%d.%d:%d, length=%d\r\n",
               src_ip[0], src_ip[1], src_ip[2], src_ip[3], src_port, data_len);
    
    /* Check if we have enough data */
    if (data_len < 236) {  // Minimum DHCP message size
        xil_printf("DHCP: Message too short (%d bytes)\r\n", data_len);
        return XST_SUCCESS;
    }
    
    /* Actual DHCP message starts at offset 8 */
    uint8_t *dhcp_msg = &buffer[8];
    
    /* Verify it's a boot reply */
    if (dhcp_msg[0] != 0x02) {  // Op: Boot Reply
        xil_printf("DHCP: Not a boot reply (Op=0x%02X)\r\n", dhcp_msg[0]);
        return XST_SUCCESS;
    }
    
    /* Check transaction ID */
    uint32_t recv_xid = ((uint32_t)dhcp_msg[4] << 24) | ((uint32_t)dhcp_msg[5] << 16) |
                        ((uint32_t)dhcp_msg[6] << 8) | dhcp_msg[7];
    
    if (recv_xid != dhcp->xid) {
        xil_printf("DHCP: XID mismatch (got 0x%08X, expected 0x%08X)\r\n", recv_xid, dhcp->xid);
        return XST_SUCCESS;
    }
    
    xil_printf("DHCP: Valid response received (XID match)\r\n");
    
    /* Extract offered IP address (YIAddr field at offset 16) */
    memcpy(dhcp->config.assigned_ip, &dhcp_msg[16], 4);
    
    xil_printf("DHCP: Offered IP: %d.%d.%d.%d\r\n",
               dhcp->config.assigned_ip[0], dhcp->config.assigned_ip[1],
               dhcp->config.assigned_ip[2], dhcp->config.assigned_ip[3]);
    
    /* Parse options (starts after magic cookie at offset 236) */
    if (data_len > 240 && 
        dhcp_msg[236] == 0x63 && dhcp_msg[237] == 0x82 && 
        dhcp_msg[238] == 0x53 && dhcp_msg[239] == 0x63) {
        xil_printf("DHCP: Magic cookie found, parsing options...\r\n");
        DHCP_ParseOptions(dhcp, &dhcp_msg[240], data_len - 240);
    } else {
        xil_printf("DHCP: No valid magic cookie found\r\n");
    }
    
    return XST_SUCCESS;
}

/**
 * @brief Parse DHCP options
 */
static void DHCP_ParseOptions(DHCP_Handle *dhcp, uint8_t *options, uint16_t len)
{
    uint16_t pos = 0;
    uint8_t msg_type = 0;
    
    while (pos < len) {
        uint8_t option = options[pos++];
        
        if (option == DHCP_OPT_END) {
            break;
        }
        
        if (option == 0) {  // Pad
            continue;
        }
        
        uint8_t opt_len = options[pos++];
        
        switch (option) {
            case DHCP_OPT_MSG_TYPE:
                msg_type = options[pos];
                xil_printf("DHCP: Message type: %d\r\n", msg_type);
                break;
                
            case DHCP_OPT_SUBNET_MASK:
                memcpy(dhcp->config.subnet_mask, &options[pos], 4);
                xil_printf("DHCP: Subnet mask: %d.%d.%d.%d\r\n",
                           dhcp->config.subnet_mask[0], dhcp->config.subnet_mask[1],
                           dhcp->config.subnet_mask[2], dhcp->config.subnet_mask[3]);
                break;
                
            case DHCP_OPT_ROUTER:
                memcpy(dhcp->config.gateway, &options[pos], 4);
                xil_printf("DHCP: Gateway: %d.%d.%d.%d\r\n",
                           dhcp->config.gateway[0], dhcp->config.gateway[1],
                           dhcp->config.gateway[2], dhcp->config.gateway[3]);
                break;
                
            case DHCP_OPT_DNS_SERVER:
                memcpy(dhcp->config.dns_server, &options[pos], 4);
                xil_printf("DHCP: DNS: %d.%d.%d.%d\r\n",
                           dhcp->config.dns_server[0], dhcp->config.dns_server[1],
                           dhcp->config.dns_server[2], dhcp->config.dns_server[3]);
                break;
                
            case DHCP_OPT_SERVER_ID:
                memcpy(dhcp->config.dhcp_server, &options[pos], 4);
                xil_printf("DHCP: Server: %d.%d.%d.%d\r\n",
                           dhcp->config.dhcp_server[0], dhcp->config.dhcp_server[1],
                           dhcp->config.dhcp_server[2], dhcp->config.dhcp_server[3]);
                break;
                
            case DHCP_OPT_LEASE_TIME:
                dhcp->config.lease_time = ((uint32_t)options[pos] << 24) |
                                         ((uint32_t)options[pos+1] << 16) |
                                         ((uint32_t)options[pos+2] << 8) |
                                         options[pos+3];
                xil_printf("DHCP: Lease time: %lu seconds\r\n", dhcp->config.lease_time);
                break;
                
            case DHCP_OPT_RENEWAL_TIME:
                dhcp->config.renewal_time = ((uint32_t)options[pos] << 24) |
                                           ((uint32_t)options[pos+1] << 16) |
                                           ((uint32_t)options[pos+2] << 8) |
                                           options[pos+3];
                break;
                
            case DHCP_OPT_REBINDING_TIME:
                dhcp->config.rebinding_time = ((uint32_t)options[pos] << 24) |
                                             ((uint32_t)options[pos+1] << 16) |
                                             ((uint32_t)options[pos+2] << 8) |
                                             options[pos+3];
                break;
        }
        
        pos += opt_len;
    }
    
    /* Process based on message type */
    if (msg_type == DHCP_OFFER && dhcp->state == DHCP_STATE_SELECTING) {
        xil_printf("DHCP: Received OFFER for %d.%d.%d.%d\r\n",
                   dhcp->config.assigned_ip[0], dhcp->config.assigned_ip[1],
                   dhcp->config.assigned_ip[2], dhcp->config.assigned_ip[3]);
        
        dhcp->state = DHCP_STATE_REQUESTING;
        dhcp->retry_count = 0;
        dhcp->timeout = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
        DHCP_SendRequest(dhcp);
        
    } else if (msg_type == DHCP_ACK && dhcp->state == DHCP_STATE_REQUESTING) {
        xil_printf("\r\n=== DHCP Configuration Successful ===\r\n");
        xil_printf("IP Address:   %d.%d.%d.%d\r\n",
                   dhcp->config.assigned_ip[0], dhcp->config.assigned_ip[1],
                   dhcp->config.assigned_ip[2], dhcp->config.assigned_ip[3]);
        xil_printf("Subnet Mask:  %d.%d.%d.%d\r\n",
                   dhcp->config.subnet_mask[0], dhcp->config.subnet_mask[1],
                   dhcp->config.subnet_mask[2], dhcp->config.subnet_mask[3]);
        xil_printf("Gateway:      %d.%d.%d.%d\r\n",
                   dhcp->config.gateway[0], dhcp->config.gateway[1],
                   dhcp->config.gateway[2], dhcp->config.gateway[3]);
        xil_printf("DNS Server:   %d.%d.%d.%d\r\n",
                   dhcp->config.dns_server[0], dhcp->config.dns_server[1],
                   dhcp->config.dns_server[2], dhcp->config.dns_server[3]);
        xil_printf("Lease Time:   %lu seconds\r\n", dhcp->config.lease_time);
        xil_printf("=====================================\r\n\r\n");
        
        /* Apply configuration to W5100 */
        W5100_SetIPAddress(dhcp->w5100, dhcp->config.assigned_ip);
        W5100_SetSubnetMask(dhcp->w5100, dhcp->config.subnet_mask);
        W5100_SetGatewayIP(dhcp->w5100, dhcp->config.gateway);
        
        dhcp->state = DHCP_STATE_BOUND;
        dhcp->config.lease_obtained = xTaskGetTickCount();
        
    } else if (msg_type == DHCP_NAK) {
        xil_printf("DHCP: Received NAK, restarting\r\n");
        dhcp->state = DHCP_STATE_INIT;
        DHCP_Start(dhcp);
    }
}

/**
 * @brief Check if DHCP lease needs renewal
 */
bool DHCP_NeedsRenewal(DHCP_Handle *dhcp)
{
    if (dhcp->state != DHCP_STATE_BOUND) {
        return false;
    }
    
    uint32_t elapsed = (xTaskGetTickCount() - dhcp->config.lease_obtained) / configTICK_RATE_HZ;
    
    return (elapsed >= dhcp->config.renewal_time);
}

/**
 * @brief Renew DHCP lease
 */
int DHCP_Renew(DHCP_Handle *dhcp)
{
    xil_printf("DHCP: Renewing lease\r\n");
    
    dhcp->state = DHCP_STATE_RENEWING;
    dhcp->retry_count = 0;
    dhcp->timeout = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
    
    DHCP_SendRequest(dhcp);
    
    return XST_SUCCESS;
}

/**
 * @brief Release DHCP lease
 */
int DHCP_Release(DHCP_Handle *dhcp)
{
    if (dhcp->state != DHCP_STATE_BOUND) {
        return XST_FAILURE;
    }
    
    xil_printf("DHCP: Releasing lease\r\n");
    
    DHCP_SendRelease(dhcp);
    dhcp->state = DHCP_STATE_RELEASED;
    
    return XST_SUCCESS;
}

/**
 * @brief Send DHCP Release message
 */
static void DHCP_SendRelease(DHCP_Handle *dhcp)
{
    uint8_t buffer[312];
    uint16_t pos = 0;
    uint16_t base = DHCP_GetSocketBase(dhcp->socket);
    
    memset(buffer, 0, sizeof(buffer));
    
    /* DHCP Header */
    buffer[pos++] = 0x01;
    buffer[pos++] = 0x01;
    buffer[pos++] = 0x06;
    buffer[pos++] = 0x00;
    
    /* Transaction ID */
    buffer[pos++] = (dhcp->xid >> 24) & 0xFF;
    buffer[pos++] = (dhcp->xid >> 16) & 0xFF;
    buffer[pos++] = (dhcp->xid >> 8) & 0xFF;
    buffer[pos++] = dhcp->xid & 0xFF;
    
    pos += 4;  // Seconds, Flags
    
    /* Client IP (current IP) */
    memcpy(&buffer[pos], dhcp->config.assigned_ip, 4);
    pos += 16;  // + Your IP, Server IP, Gateway IP
    
    /* Client MAC */
    memcpy(&buffer[pos], dhcp->mac, 6);
    pos += 16;
    
    pos += 192;  // Server hostname, Boot filename
    
    /* Magic cookie */
    buffer[pos++] = 0x63;
    buffer[pos++] = 0x82;
    buffer[pos++] = 0x53;
    buffer[pos++] = 0x63;
    
    /* DHCP Options */
    buffer[pos++] = DHCP_OPT_MSG_TYPE;
    buffer[pos++] = 1;
    buffer[pos++] = DHCP_RELEASE;
    
    buffer[pos++] = DHCP_OPT_SERVER_ID;
    buffer[pos++] = 4;
    memcpy(&buffer[pos], dhcp->config.dhcp_server, 4);
    pos += 4;
    
    buffer[pos++] = DHCP_OPT_END;
    
    /* Send to DHCP server (unicast) */
    W5100_WriteBuffer(dhcp->w5100, base + W5100_Sn_DIPR, dhcp->config.dhcp_server, 4);
    W5100_WriteWord(dhcp->w5100, base + W5100_Sn_DPORT, DHCP_SERVER_PORT);
    
    W5100_SendData(dhcp->w5100, dhcp->socket, buffer, pos);
    W5100_ExecCmdSn(dhcp->w5100, dhcp->socket, W5100_SOCK_SEND);
}

/**
 * @brief Get current DHCP state
 */
DHCP_State DHCP_GetState(DHCP_Handle *dhcp)
{
    return dhcp->state;
}

/**
 * @brief Get DHCP configuration
 */
const DHCP_Config* DHCP_GetConfig(DHCP_Handle *dhcp)
{
    return &dhcp->config;
}

/**
 * @brief Generate transaction ID
 */
static uint32_t DHCP_GetXID(void)
{
    static uint32_t xid = 0;
    
    if (xid == 0) {
        /* Use tick count as initial seed */
        xid = xTaskGetTickCount();
    }
    
    xid += 0x12345678;
    
    return xid;
}

/**
 * @brief Get socket register base address
 */
static uint16_t DHCP_GetSocketBase(uint8_t sock)
{
    return W5100_S0_BASE + (sock * 0x0100);
}