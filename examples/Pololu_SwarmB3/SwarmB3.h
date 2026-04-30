#ifndef SWARMB2_H
#define SWARMB2_H

#include <Arduino.h>
#include "ircomm_i2c_datatypes.h"

class SwarmB3_c {

  public:

    // Persistent cache of board settings
    // Because the settings are numerous
    // and would require some informed 
    // decision making, I think it is best
    // if the user edits these directly
    // and simply called get/set below.
    volatile ir_rx_params_t rx_settings[4];
    volatile ir_tx_params_t tx_settings[4];
    volatile ir_params_t    general;

    SwarmB3_c() {

    }

    // Fetches current config from board
    // storing into cache
    void init();

    // Basic commands
    void getRxSettings(int which);
    void getTxSettings(int which);
    void setRxSettings(int which);
    void setTxSettings(int which);
    void getGeneralSettings();
    void setGeneralSettings();
    void printTxSettings(int which);
    void printRxSettings(int which);
    void resetMetrics();
    void configureDefault();
    void updateSettings();

    // Messaging operations
    uint8_t getMsgLength( int which_rx );
    void    setIRMessage( uint8_t * payload, int len, int which );
    int     getIRMessage( uint8_t * msg_buf, int rx );

    void printStatus();
    void printAnyMessage();
    void printBearing();
    void printVectors();

    // Functions for board metrics
    // Refer to ircomm_i2c.h for datatypes
    ir_status_t       getStatus();
    ir_bearing_t      getBearing();
    ir_vectors_t      getVectors();
    ir_activity_t     getActivity();
    ir_saturation_t   getSaturation();
    ir_msg_timings_t  getMsgTimings();
    ir_byte_timings_t getByteTimings();
    ir_errors_t       getErrors();
    ir_crc_t          getCRC();
    ir_sensors_t      getSensors();
};

#endif
