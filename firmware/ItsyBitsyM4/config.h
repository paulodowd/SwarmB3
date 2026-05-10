
// 27/03/26: I've directly copied this in from Swarm-B2 without
// changes so this will need a revision. Many fields don't 
// make sense for SwarmB3

// The following provide the default configuration
// of the communication board.

#ifndef IRCOMM_CONFIG_H
#define IRCOMM_CONFIG_H


/*
 * General (top-level) config
 */
#define BAUD                    9600
#define BROADCAST               0
#define BIDIRECTIONAL           0 
#define BEARING_UPDATE_US       10000 // 100ms
#define BEARING_ALPHA           0.25  // 
#define TX_PREAMBLE_BYTE        0x55 // 0b01010101, 'U'


/*
 * Config assigned to each receiver
 */
#define RX_OVERRUN              true  // allow for rx message to complete? 
#define RX_ENABLED              1
#define RX_TIMEOUT_MULTI        8
#define RX_SATURATION_US        8000 // 8ms no bytes
#define RX_DESATURATION_US      2000 // off for 2ms

/*
 * Config assigned to each transmitter
 */
#define TX_INTERVAL_MOD         10
#define TX_REPEAT               1
#define TX_PREDICT_MULTI        0
#define TX_DEFER_MULTI          0
#define TX_PREAMBLE_REPEAT      4
#define TX_INTERVAL_MS          100
#define TX_BASE_MS              100
#define TX_LEN                  0






#endif
