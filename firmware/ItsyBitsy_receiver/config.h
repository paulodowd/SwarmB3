
// 27/03/26: I've directly copied this in from Swarm-B2 without
// changes so this will need a revision. Many fields don't 
// make sense for SwarmB3

// The following provide the default configuration
// of the communication board.

#ifndef IRCOMM_CONFIG_H
#define IRCOMM_CONFIG_H

// If we know we are going to send and receive
// messages of a certain length then we could
// configure the device for a more optimal
// rx_delay and tx_delay.  To try and have the
// board do this for itself, set the below to
// true.  Set true, the board will look at the
// length of the message to transmit and the
// length of messages received, and setup the
// tx_delay and rx_delay to be appropriate for
// which ever of the two is longest.
// If set to false, the board will use the
// #defines set above for tx/rx_delay _bias _mod.
#define RX_PREDICT_MULTIPLIER 1   // how many message-size to wait?
#define RX_DESYNC             false
#define RX_OVERRUN            true  // allow for rx message to complete? 
#define RX_DEFAULT_MSG_LEN    MAX_MSG // 36 is worst case
#define MS_PER_BYTE_58KHZ     1.2   // 58khz
#define US_PER_BYTE_58KHZ     1250   // 58khz

// A rough estimate of how many ms per byte 
// during the transmit/receive process.
#define RX_TIMEOUT_MULTI       4

#define RX_DEFAULT_PERIOD       (MS_PER_BYTE_58KHZ*RX_DEFAULT_MSG_LEN)     

#define RX_SATURATION_US       8000// I measured 8000us for ambient noise




// What system should we use for the tranmission?
// Periodic: Decouples receiving from transmission, meaning
//          that tranmission happens every (x)ms.
// Interleaved: Means that after a receiver is rotated, we
//          also transmit.  This means that if a robot is
//          receiving messages well, the tranmission rate
//          would also increase.  Seems complicated.
#define TX_PREDICT_PERIOD     false
#define TX_PREDICT_MULTI      8
#define TX_DEFER_MULTI        4
#define TX_PREAMBLE_REPEAT    4
#define TX_PREAMBLE_BYTE      0x55 // 0b01010101

// When set in TX_MODE_PERIODIC
// How long should the robot wait before doing another
// transmission?  
// For 38khz, a 32byte message will take approximately 
// 80ms to transmit/receive.
// For 58khz, a 32byte message will take approximately 
// 39ms to transmit/receive

#define TX_DEFAULT_PERIOD (160) // in ms, 0 disables tx

// Should we try to break synchrony between robots
// by randomising the tx period?
#define TX_DESYNC  1

 // Try to minimise interference by only transmitting
 // when no other IR transmission has been detected
 // If IR detected, will try again on next iteration
#define TX_DEFER   0

// How many times should we repeat the transmission
// of a message? This should be set as a positive
// no zero value (1+)
#define TX_DEFAULT_REPEAT 3



// How often should the bearing estimate be updated?
#define UPDATE_BEARING_MS  250


// I2C constrains the message payload to 32
// bytes.  
#define MAX_MSG 32

// When we encode a message, the worst case
// is 32 bytes of payload, and each byte is
// escaped with a byte. We also need to add
// the start, length and CRC bytes (+4).
#define MAX_TX_BUF (MAX_MSG * 2) + 4


#endif
