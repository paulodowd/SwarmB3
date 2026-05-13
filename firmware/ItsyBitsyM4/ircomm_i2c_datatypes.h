
#ifndef IRCOMM_I2C_DATATYPES_H
#define IRCOMM_I2C_DATATYPES_H

#pragma pack(push, 1) 

#define IRCOMM_I2C_ADDR  0x11


// hard i2c constraints
// message payload to 32 bytes.  
#define MAX_MSG 32

// When we encode a message, the worst case
// is 32 bytes of payload, and each byte is
// escaped with a byte. We also need to add
// the start (+1), length (+1) and escaped 
// CRC bytes (2*2=4). 
#define MAX_TX_BUF (MAX_MSG * 2) + 6

// This 1 byte struct is used to change the i2c
// operation.  For example, to reset counts on
// the board, or to transfer a message to or from
// the board, etc.
typedef uint8_t ir_mode_t;

// These flags are used to request specific data
// from this board, or cause specific functions.
// ir_mode.mode should be set with one of these
// flags.
#define MODE_NOT_SET              0

// Messaging
#define MODE_REPORT_STATUS      1    
#define MODE_REPORT_SIZE_MSG0     2       
#define MODE_REPORT_SIZE_MSG1     3
#define MODE_REPORT_SIZE_MSG2     4
#define MODE_REPORT_SIZE_MSG3     5
#define MODE_REPORT_MSG0          6
#define MODE_REPORT_MSG1          7
#define MODE_REPORT_MSG2          8
#define MODE_REPORT_MSG3          9
#define MODE_CLEAR_MSG0           10
#define MODE_CLEAR_MSG1           11
#define MODE_CLEAR_MSG2           12
#define MODE_CLEAR_MSG3           13
#define MODE_SET_MSG_ALL          14
#define MODE_SET_MSG_0            15
#define MODE_SET_MSG_1            16
#define MODE_SET_MSG_2            17
#define MODE_SET_MSG_3            18

// Bearing
#define MODE_REPORT_ACTIVITY      19
#define MODE_REPORT_VECTORS       20
#define MODE_REPORT_BEARING       21

// Resets
#define MODE_FULL_RESET           22
#define MODE_RESET_METRICS        23

// Timing
#define MODE_REPORT_TX_TIMINGS    24
#define MODE_REPORT_MSG_TIMINGS   25
#define MODE_REPORT_BYTE_TIMINGS  26
#define MODE_REPORT_TX_COUNTS     27

// Errors
#define MODE_REPORT_CRC           28
#define MODE_REPORT_FRAME_ERRS    29
#define MODE_REPORT_ERRORS        30
#define MODE_REPORT_SATURATION    31

// Config
#define MODE_SET_RX_CONFIG_0      32
#define MODE_SET_RX_CONFIG_1      33
#define MODE_SET_RX_CONFIG_2      34
#define MODE_SET_RX_CONFIG_3      35
#define MODE_SET_TX_CONFIG_0      36
#define MODE_SET_TX_CONFIG_1      37
#define MODE_SET_TX_CONFIG_2      38
#define MODE_SET_TX_CONFIG_3      39
#define MODE_SET_GEN_CONFIG       40

#define MODE_GET_RX_CONFIG_0      41
#define MODE_GET_RX_CONFIG_1      42
#define MODE_GET_RX_CONFIG_2      43
#define MODE_GET_RX_CONFIG_3      44
#define MODE_GET_TX_CONFIG_0      45
#define MODE_GET_TX_CONFIG_1      46
#define MODE_GET_TX_CONFIG_2      47
#define MODE_GET_TX_CONFIG_3      48
#define MODE_GET_GEN_CONFIG       49

#define MODE_REPORT_SENSORS       50

#define MAX_MODE                  51



typedef uint8_t ir_status_t;

// Counts for each type of error
// per receiver.
// [ rx ][ error ]
typedef struct  {  
  uint16_t type[4][4];
} ir_errors_t;

// Contains pass/fail count for the
// crc decoded at the end of each message.
typedef struct  {
  uint32_t fail[4];   // 4 * 4 = 16bytes
  uint32_t pass[4];   // 4 * 4 = 16bytes
} ir_crc_t;

// Used to store a count of frame errors at
// the UART hardware level.
typedef struct  {
  uint32_t rx[4];       // 4x4 = 16 bytes
} ir_frame_errors_t;

// Contains a simple count of byte activity
// per receiver.  Used to estimate bearing
// elsewhere.
typedef struct {    
  uint32_t rx[4];
} ir_activity_t;

// Used to periodically create component
// vectors for a bearing estimation, 
// drawn from the activity struct.
typedef struct {
  float rx[4];         // 4x4 = 16bytes
} ir_vectors_t;

// Contains the latest bearing estimation
// components.
// Theta: angle estimate.
// Mag: resultant magnitude. If 1, theta
//      is very confident. If 0, counts for
//      each receiver have cancelled out.
// Sum: Pre-normalised sum of rx counts used.
typedef struct  {
  uint32_t us_ts;     // 4 bytes
  float theta;                // 4
  float mag;                  // 4
  float sum;                  // 4
} ir_bearing_t;



// Used to count how often the receivers 
// are power cycled due to prolonged period
// of inactivity.
typedef struct  {
  uint32_t rx[4];
} ir_saturation_t;


// To find out if a message is ready
// to collect.
// 0: no message.
// <33: message length. 
typedef struct {  // 1 byte
  uint8_t n_bytes;
} ir_msg_length_t;


typedef struct {
  uint32_t last_ts_ms[4]; // 4x4 bytes
  uint16_t duration_ms[4]; // 4x2 bytes
} ir_tx_timings_t;

typedef struct {
  uint32_t sent[4];       // 4x4=16
  uint32_t deferred[4];   // 4x4=16
} ir_tx_counts_t;


// To find out the relative timing of
// message activity
typedef struct { // 32 bytes
  uint32_t dt_ms[4];           // 16 bytes
  uint32_t ts_ms[4];            // 16 bytes
} ir_msg_timings_t;

// To find out the relative timing of
// byte activity (not full messages 
// correctly received)
typedef struct { // 32 bytes
  uint32_t dt_us[4];           // 16 bytes
  uint32_t ts_us[4];            // 16 bytes
} ir_byte_timings_t;

// Used to report back readings from the
// extra sensors that can be mounted on
// the communication board
typedef struct {
  int16_t ldr[3];     // 6 bytes
  int16_t prox[2];    // 4 bytes
} ir_sensors_t;


// Struct to contain the configuration
// for transmission.
typedef struct {      // total = 17 bytes
  uint32_t repeat;            // 4: how many repeated IR transmissions?
  uint8_t  predict_multi;     // 1: how many multiples of tx_len to use with predict?
  uint8_t  defer_multi;       // 1: how many multiples of ms since rx to cancel a tx?
  uint8_t  preamble_repeat;   // 1: how many repeated preamble bytes before transmission?
  uint32_t interval_ms;       // 4: periodic:  current ms period to send messages
  uint32_t base_ms;           // 4: min tx period allowable
  uint8_t  interval_mod;
  uint8_t  csma_multi;
  uint8_t  len;               // 1: how long is the message to transmit?
} ir_tx_params_t;


// Struct to contain the configuration
// for reception.
typedef struct {       // total = 8 bytes.
  union {                           // 1 bytes
    uint8_t all_flags;             // to access all flags at once
    struct {
      uint8_t overrun         : 1; // complete recieve outside period?
      uint8_t enabled         : 1; // receiver available to use?
      uint8_t reserved        : 6; // randomise rx cycling
    } bits;
  } flags;
  uint8_t   timeout_multi;      //  1: If we haven't received a consecutive byte, timeout
  uint16_t  saturation_us;      //  2: Rx seems to saturate, watch for 0 byte activity.
  uint16_t  desaturation_us;    //  2: How long to desaturation for?
} ir_rx_params_t;    

typedef struct {
  union {                           // 1 bytes
    uint8_t all_flags;             // to access all flags at once
    struct {
      uint8_t broadcast       : 1; // tx combined, or independent?
      uint8_t bidirectional   : 1;
      uint8_t reserved        : 6; // not used
    } bits;
  } flags;

  uint16_t bearing_update_us; // 2: how often to update bearing
  float    bearing_alpha;     // 4: filter co-efficient for bearing/vectors
  uint8_t  preamble_byte;     // 1
  uint16_t baud;              // 2
} ir_params_t;


typedef struct {
  ir_crc_t          crc;
  ir_activity_t     activity;
  ir_saturation_t   saturation;
  ir_errors_t       errors;    // error types for each recevier
  ir_frame_errors_t frame_errors; // this one needs integrating with channel[]
  ir_msg_timings_t  msg_timings;
  ir_byte_timings_t byte_timings;
  ir_vectors_t      vectors;
  ir_bearing_t      bearing;
  ir_sensors_t      sensors;
  ir_tx_timings_t   tx_timings;
  ir_tx_counts_t    tx_counts;
} ir_metrics_t;    

// On this new board, each rx demodulator and
// pair of IR LEDs are attached to independent
// uart interfaces. I think that means that
// they could be set up to transmit different
// length messages, and they could each be
// receiving different length messages.
// Therefore, I think the easiest way to
// represent this in the overall config is
// to make the tx and rx structs into
// arrays, 1 for each uart.
// We will still need a top-most level config
// to decide if the board is going to use all
// tx independently, or in broadcast, etc
typedef struct {
  ir_status_t status;
  uint8_t         msg_len[4];
  uint8_t         msg[4][MAX_MSG];
  uint8_t         tx_buf[4][MAX_TX_BUF];
  ir_params_t     general;
  ir_tx_params_t  tx[4];          // 11 bytes
  ir_rx_params_t  rx[4];          // 20 bytes
} ir_config_t;

#pragma pack(pop)
#endif
