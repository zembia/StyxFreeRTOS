#ifndef W5100_DHCP_H
#define W5100_DHCP_H


#include <stdint.h>
#include <stdbool.h>
#include "W5100.h"


/* DHCP Message Types */
#define DHCP_DISCOVER   1
#define DHCP_OFFER      2
#define DHCP_REQUEST    3
#define DHCP_DECLINE    4
#define DHCP_ACK        5
#define DHCP_NAK        6
#define DHCP_RELEASE    7
#define DHCP_INFORM     8

/* DHCP Options */
#define DHCP_OPT_SUBNET_MASK        1
#define DHCP_OPT_ROUTER             3
#define DHCP_OPT_DNS_SERVER         6
#define DHCP_OPT_HOSTNAME           12
#define DHCP_OPT_DOMAIN_NAME        15
#define DHCP_OPT_REQUESTED_IP       50
#define DHCP_OPT_LEASE_TIME         51
#define DHCP_OPT_MSG_TYPE           53
#define DHCP_OPT_SERVER_ID          54
#define DHCP_OPT_PARAM_REQUEST      55
#define DHCP_OPT_RENEWAL_TIME       58
#define DHCP_OPT_REBINDING_TIME     59
#define DHCP_OPT_CLIENT_ID          61
#define DHCP_OPT_END                255

/* DHCP Ports */
#define DHCP_SERVER_PORT    67
#define DHCP_CLIENT_PORT    68


/* DHCP States */
typedef enum {
    DHCP_STATE_INIT,
    DHCP_STATE_SELECTING,
    DHCP_STATE_REQUESTING,
    DHCP_STATE_BOUND,
    DHCP_STATE_RENEWING,
    DHCP_STATE_REBINDING,
    DHCP_STATE_RELEASED,
    DHCP_STATE_FAILED
} DHCP_State;



/* DHCP Configuration Structure */
typedef struct {
    uint8_t assigned_ip[4];      // IP address assigned by DHCP
    uint8_t subnet_mask[4];      // Subnet mask
    uint8_t gateway[4];          // Default gateway
    uint8_t dns_server[4];       // DNS server
    uint8_t dhcp_server[4];      // DHCP server IP
    uint32_t lease_time;         // Lease time in seconds
    uint32_t renewal_time;       // T1 - Time to start renewal
    uint32_t rebinding_time;     // T2 - Time to start rebinding
    uint32_t lease_obtained;     // Tick count when lease was obtained
} DHCP_Config;

/* DHCP Handle Structure */
typedef struct {
    W5100_Handle *w5100;         // Pointer to W5100 handle
    uint8_t socket;              // UDP socket for DHCP
    DHCP_State state;            // Current DHCP state
    DHCP_Config config;          // DHCP configuration
    uint32_t xid;                // Transaction ID
    uint8_t mac[6];              // MAC address
    uint32_t timeout;            // Timeout for current operation
    uint8_t retry_count;         // Number of retries
} DHCP_Handle;


/**
 * @brief Initialize DHCP client
 * @param dhcp Pointer to DHCP handle
 * @param w5100 Pointer to initialized W5100 handle
 * @param socket Socket number to use for DHCP (0-3)
 * @param mac MAC address (6 bytes)
 * @return XST_SUCCESS on success
 */
int DHCP_Init(DHCP_Handle *dhcp, W5100_Handle *w5100, uint8_t socket, const uint8_t *mac);

/**
 * @brief Start DHCP process to obtain IP address
 * @param dhcp Pointer to DHCP handle
 * @return XST_SUCCESS if DHCP succeeded
 */
int DHCP_Start(DHCP_Handle *dhcp);

/**
 * @brief Process DHCP state machine (call periodically)
 * @param dhcp Pointer to DHCP handle
 * @return XST_SUCCESS if still processing, XST_FAILURE if failed
 */
int DHCP_Process(DHCP_Handle *dhcp);

/**
 * @brief Renew DHCP lease
 * @param dhcp Pointer to DHCP handle
 * @return XST_SUCCESS on success
 */
int DHCP_Renew(DHCP_Handle *dhcp);

/**
 * @brief Release DHCP lease
 * @param dhcp Pointer to DHCP handle
 * @return XST_SUCCESS on success
 */
int DHCP_Release(DHCP_Handle *dhcp);

/**
 * @brief Check if DHCP lease needs renewal
 * @param dhcp Pointer to DHCP handle
 * @return true if renewal is needed
 */
bool DHCP_NeedsRenewal(DHCP_Handle *dhcp);

/**
 * @brief Get current DHCP state
 * @param dhcp Pointer to DHCP handle
 * @return Current DHCP state
 */
DHCP_State DHCP_GetState(DHCP_Handle *dhcp);

/**
 * @brief Get DHCP configuration
 * @param dhcp Pointer to DHCP handle
 * @return Pointer to DHCP configuration (valid when state is BOUND)
 */
const DHCP_Config* DHCP_GetConfig(DHCP_Handle *dhcp);


#endif