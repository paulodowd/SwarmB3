#include "SwarmB3.h"
#include <Wire.h>
#include <stdint.h>
#include <limits.h>

void SwarmB3_c::init() {
  memset( (uint8_t*)&rx_settings, 0, sizeof( rx_settings ));
  memset( (uint8_t*)&tx_settings, 0, sizeof( tx_settings ));
  memset( (ir_params_t*)&general, 0, sizeof( ir_params_t));

  // Populate the board config cache ready for
  // any subsequent user changes
  for ( int i = 0; i < 4; i++ ) {
    getRxSettings(i);
    delay(1);
    getTxSettings(i);
    delay(1);
  }
  getGeneralSettings();
//  general.flags.bits.broadcast = 1;
//  setGeneralSettings();
  // Ensure we have default settings
  //configureDefault();
}


void SwarmB3_c::configureDefault() {
  //
  //  // Settings for receiving
  //  rx_settings.flags.bits.cycle_on_rx    = true;
  //  rx_settings.flags.bits.desync         = true;
  //  rx_settings.flags.bits.overrun        = true;
  //  rx_settings.flags.bits.rand_rx        = false;
  //  rx_settings.flags.bits.rx0            = true;
  //  rx_settings.flags.bits.rx1            = true;
  //  rx_settings.flags.bits.rx2            = true;
  //  rx_settings.flags.bits.rx3            = true;
  //  rx_settings.skip_multi                = 0;
  //  rx_settings.predict_multi             = 0;
  //  rx_settings.index                     = 0;
  //  rx_settings.period_base_ms            = 60;
  //  rx_settings.timeout_multi             = 0;
  //  rx_settings.saturation_us             = 20000;
  //  setRxSettings();
  //
  //  // Settings for tranmission
  //  tx_settings.flags.bits.desync = 1;
  //  tx_settings.repeat            = 3;
  //  tx_settings.predict_multi     = 4;
  //  tx_settings.defer_multi       = 1;
  //  tx_settings.preamble_repeat   = 4;
  //  tx_settings.period_base_ms    = 170;
  //  setTxSettings();
  //
  //  resetMetrics();
}

void SwarmB3_c::updateSettings() {

  //  setRxSettings();

  //  setTxSettings();

  resetMetrics();
}


// This function is used to fetch a message
// from one of the four receivers on the
// IR communication board. This function will
// complete the check of whether or not there
// is a message available.
// Return:
//  -1    invalid request
//   0    no message available
//  [1:32]valid message length
int SwarmB3_c::getIRMessage( uint8_t * received, int which_rx ) {

  // Bad request
  if ( which_rx < 0 || which_rx > 3 ) return -1;

  // Get the message status from the board (1 byte)
  ir_status_t ir_status = getStatus();

  // If the status bit for this rx is set
  if ( ir_status & (1 << which_rx) ) {

    // First, check if there is a message
    // available (+ve non-zero length)
    uint8_t len = getMsgLength( which_rx );

    // valid message len is 1:32. 32 is the
    // maximum number of uint8_ts in an i2c
    // transaction on Arduino
    if ( len > 0 && len < 33 ) {

      ir_mode_t mode;

      // Format mode request for which receiver
      if ( which_rx == 0 ) {
        mode = MODE_REPORT_MSG0;
      } else if ( which_rx == 1 ) {
        mode = MODE_REPORT_MSG1;
      }  else if ( which_rx == 2 ) {
        mode = MODE_REPORT_MSG2;
      }  else if ( which_rx == 3 ) {
        mode = MODE_REPORT_MSG3;
      } else {
        // error caught above.
      }

      // Set mode to send across the message
      Wire.beginTransmission( IRCOMM_I2C_ADDR );
      Wire.write( (uint8_t*)&mode, sizeof( mode ));
      Wire.endTransmission();

      // Read across uint8_ts using the anticipated len.
      // Store into buffer provided as function argument
      Wire.requestFrom( IRCOMM_I2C_ADDR, len );
      Wire.readBytes( (uint8_t*)received, len );

      // Let the user know how many bytes were
      // received.
      return (int)len;

    } else {
      // Flag that no bytes were available
      return 0;
    }
  }
  return 0;

}

void SwarmB3_c::printStatus() {

  ir_status_t ir_status = getStatus();

  Serial.print("Msg[0:3], Activity[4:7]: ");

  for ( int i = 0; i < 8; i++ ) {
    if ( ir_status & (1 << i) ) {
      Serial.print("1 ");
    } else {
      Serial.print("0 ");
    }
  }
  Serial.println();
}


void SwarmB3_c::printAnyMessage() {

  char buf[32];

  // First, check if any messages are available
  ir_status_t ir_status = getStatus();

  // Message available is first 4 bits
  for ( int i = 0; i < 4; i++ ) {
    if ( ir_status & (1 << i) ) {
      memset( (void*)&buf, 0, sizeof(buf) );
      int len = getIRMessage( (uint8_t*)buf, i );
      Serial.print("Rx");
      Serial.print( i );
      Serial.print(" len=");
      Serial.print(len);
      Serial.print(":");
      Serial.print( buf );
      Serial.print(" t=");
      Serial.println( millis ());
    }
  }


}

// Queries the IR communication board to see if
// a message has been stored for a receiver.
// There are 4 receivers, [0:3]
// Return:
//  -1    Invalid receiver request
//   0    No message available
//  [0:32]Available message length
uint8_t SwarmB3_c::getMsgLength( int which_rx ) {

  ir_mode_t mode;

  if ( which_rx == 0 ) {
    mode = MODE_REPORT_SIZE_MSG0;
  } else if ( which_rx == 1 ) {
    mode = MODE_REPORT_SIZE_MSG1;
  } else if ( which_rx == 2 ) {
    mode = MODE_REPORT_SIZE_MSG2;
  } else if ( which_rx == 3 ) {
    mode = MODE_REPORT_SIZE_MSG3;
  } else {
    // Invalid receiver
    return -1;
  }

  // Set mode to read back how many uint8_ts are
  // available.
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&mode, sizeof( mode ));
  Wire.endTransmission();

  // We'll use this struct to check how many
  // uint8_ts are available of a message.
  // 0 uint8_ts means no message.
  ir_msg_length_t msg_len;

  // Request the message size to be sent across into
  // msg_status
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( msg_len ));
  Wire.readBytes( (uint8_t*)&msg_len, sizeof( msg_len ));

  return msg_len.n_bytes;// 0 = error, else [1 : 32]
}

//
void SwarmB3_c::setIRMessage( uint8_t * payload, int len, int which ) {
  // Message must be length [1:32]
  if ( len <= 32 && len > 0) {

    ir_mode_t mode;

    // Set mode to set a new IR Message
    switch ( which ) {
      case 0: mode = MODE_SET_MSG_0;
        break;
      case 1: mode = MODE_SET_MSG_1;
        break;
      case 2: mode = MODE_SET_MSG_2;
        break;
      case 3: mode = MODE_SET_MSG_3;
        break;
      default: return;
    }

    Wire.beginTransmission( IRCOMM_I2C_ADDR );
    Wire.write( (uint8_t*)&mode, sizeof( mode));
    Wire.endTransmission();

    // The communication board will always default
    // to waiting to receive a message to transmit
    // so we don't need to change the mode.
    Wire.beginTransmission( IRCOMM_I2C_ADDR );
    Wire.write( (uint8_t*)payload, len);
    Wire.endTransmission();
  }
}

void SwarmB3_c::printBearing() {
  ir_bearing_t bearing = getBearing();
  Serial.print( bearing.mag, 2);
  Serial.print(",");
  Serial.print( bearing.theta, 2 );
  Serial.print(",");
  Serial.println( bearing.sum, 2);
}
void SwarmB3_c::printVectors() {
  ir_vectors_t vectors = getVectors();
  for( int i = 0; i < 4; i++ ) {
    Serial.print( vectors.rx[i] );
    Serial.print(",");
  }
  Serial.println();
}

void SwarmB3_c::getGeneralSettings() {
  ir_mode_t mode;

  mode = MODE_GET_GEN_CONFIG;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&mode, sizeof( mode));
  Wire.endTransmission();

  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( ir_params_t ));
  Wire.readBytes( (uint8_t*)&general, sizeof( ir_params_t ));

}
void SwarmB3_c::setGeneralSettings() {
  ir_mode_t mode;

  mode = MODE_SET_GEN_CONFIG;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&mode, sizeof( mode));
  Wire.endTransmission();

  // Now send the struct
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&general, sizeof( ir_params_t ));
  Wire.endTransmission();
}


ir_bearing_t SwarmB3_c::getBearing() {
  ir_mode_t mode;

  mode = MODE_REPORT_BEARING;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&mode, sizeof( mode));
  Wire.endTransmission();

  ir_bearing_t bearing;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( bearing ));
  Wire.readBytes( (uint8_t*)&bearing, sizeof( bearing ));

  return bearing;
}

void SwarmB3_c::resetMetrics() {
  ir_mode_t mode;

  // Set i2c mode.
  mode = MODE_RESET_METRICS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&mode, sizeof( mode));
  Wire.endTransmission();
}


void SwarmB3_c::getRxSettings(int which ) {
  ir_mode_t mode;

  // Set i2c mode.
  switch ( which ) {
    case 0: mode = MODE_GET_RX_CONFIG_0;
      break;
    case 1: mode = MODE_GET_RX_CONFIG_1;
      break;
    case 2: mode = MODE_GET_RX_CONFIG_2;
      break;
    case 3: mode = MODE_GET_RX_CONFIG_3;
      break;
    default: return;
  }

  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&mode, sizeof( mode));
  Wire.endTransmission();
  
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( ir_rx_params_t ));
  Wire.readBytes( (uint8_t*)&rx_settings[which], sizeof( ir_rx_params_t ));
  
}


void SwarmB3_c::getTxSettings(int which) {
  ir_mode_t mode;

  switch ( which ) {
    case 0: mode = MODE_GET_TX_CONFIG_0;
      break;
    case 1: mode = MODE_GET_TX_CONFIG_1;
      break;
    case 2: mode = MODE_GET_TX_CONFIG_2;
      break;
    case 3: mode = MODE_GET_TX_CONFIG_3;
      break;
    default: return;
  }

  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&mode, sizeof( mode));
  Wire.endTransmission();

  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( ir_tx_params_t ));
  Wire.readBytes( (uint8_t*)&tx_settings[which], sizeof( ir_tx_params_t ));
}


void SwarmB3_c::setRxSettings( int which ) {

  ir_mode_t mode;

  switch ( which ) {
    case 0: mode = MODE_SET_RX_CONFIG_0;
      break;
    case 1: mode = MODE_SET_RX_CONFIG_1;
      break;
    case 2: mode = MODE_SET_RX_CONFIG_2;
      break;
    case 3: mode = MODE_SET_RX_CONFIG_3;
      break;
    default: return;
  }
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&mode, sizeof( mode));
  Wire.endTransmission();

  // Now send the struct
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&rx_settings[which], sizeof( ir_rx_params_t ));
  Wire.endTransmission();
}


void SwarmB3_c::setTxSettings( int which ) {

  ir_mode_t mode;

  switch ( which ) {
    case 0: mode = MODE_SET_TX_CONFIG_0;
      break;
    case 1: mode = MODE_SET_TX_CONFIG_1;
      break;
    case 2: mode = MODE_SET_TX_CONFIG_2;
      break;
    case 3: mode = MODE_SET_TX_CONFIG_3;
      break;
    default: return;
  }
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&mode, sizeof( mode));
  Wire.endTransmission();

  // Now send the struct
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&tx_settings[which], sizeof( ir_tx_params_t ));
  Wire.endTransmission();
}


/*
     uint32_t repeat;            // 4: how many repeated IR transmissions?
  uint8_t  predict_multi;     // 1: how many multiples of tx_len to use with predict?
  uint8_t  defer_multi;       // 1: how many multiples of ms since rx to cancel a tx?
  uint8_t  preamble_repeat;   // 1: how many repeated preamble bytes before transmission?
  uint32_t interval_ms;       // 4: periodic:  current ms period to send messages
  uint32_t base_ms;           // 4: min tx period allowable
  uint8_t  interval_mod;
  uint8_t  len;
*/
void SwarmB3_c::printTxSettings(int which ) {
  Serial.print("Tx"); Serial.print(which); Serial.println(":");
  Serial.print(" - Repeat:"); Serial.println( tx_settings[which].repeat );
  Serial.print(" - Predict:"); Serial.println( tx_settings[which].predict_multi );
  Serial.print(" - Defer:"); Serial.println( tx_settings[which].defer_multi );
  Serial.print(" - Preamble:"); Serial.println( tx_settings[which].preamble_repeat );
  Serial.print(" - Interval:"); Serial.println( tx_settings[which].interval_ms );
  Serial.print(" - InterMod:"); Serial.println( tx_settings[which].interval_mod );
  Serial.print(" - len:"); Serial.println( tx_settings[which].len);
}

/*

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
*/
void SwarmB3_c::printRxSettings(int which) {
  Serial.print("Rx"); Serial.print(which); Serial.println(":");
  Serial.print(" - Overrun:"); Serial.println( rx_settings[which].flags.bits.overrun );
  Serial.print(" - Enabled:"); Serial.println( rx_settings[which].flags.bits.enabled );
  Serial.print(" - TimoutM:"); Serial.println( rx_settings[which].timeout_multi );
  Serial.print(" - Sat us:"); Serial.println( rx_settings[which].saturation_us );
  Serial.print(" - DeSat us:"); Serial.println( rx_settings[which].desaturation_us );
}



ir_status_t   SwarmB3_c::getStatus() {
  // Set correct more
  ir_mode_t mode;
  mode = MODE_REPORT_STATUS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&mode, sizeof( mode));
  Wire.endTransmission();

  // Get data
  ir_status_t ir_status;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( ir_status));
  Wire.readBytes( (uint8_t*)&ir_status, sizeof( ir_status));

  // Return result
  return ir_status;

}

ir_vectors_t      SwarmB3_c::getVectors() {
  // Set correct more
  ir_mode_t mode;
  mode = MODE_REPORT_VECTORS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&mode, sizeof( mode));
  Wire.endTransmission();

  // Get data
  ir_vectors_t vectors;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( vectors));
  Wire.readBytes( (uint8_t*)&vectors, sizeof( vectors ));

  // Return result
  return vectors;
}

ir_activity_t     SwarmB3_c::getActivity() {
  ir_mode_t mode;
  mode = MODE_REPORT_ACTIVITY;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&mode, sizeof( mode ));
  Wire.endTransmission();

  ir_activity_t activity;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( activity ));
  Wire.readBytes( (uint8_t*)&activity, sizeof( activity ));

  return activity;
}

ir_saturation_t   SwarmB3_c::getSaturation() {
  // Set correct more
  ir_mode_t mode;
  mode = MODE_REPORT_SATURATION;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&mode, sizeof( mode));
  Wire.endTransmission();

  // Get data
  ir_saturation_t saturation;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( saturation ));
  Wire.readBytes( (uint8_t*)&saturation, sizeof( saturation ));

  // Return result
  return saturation;
}

ir_msg_timings_t  SwarmB3_c::getMsgTimings() {
  // Set correct more
  ir_mode_t mode;
  mode = MODE_REPORT_MSG_TIMINGS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&mode, sizeof( mode));
  Wire.endTransmission();

  // Get data
  ir_msg_timings_t msg_timings;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( msg_timings ));
  Wire.readBytes( (uint8_t*)&msg_timings, sizeof( msg_timings ));

  // Return result
  return msg_timings;
}
ir_byte_timings_t SwarmB3_c::getByteTimings() {
  // Set correct more
  ir_mode_t mode;
  mode = MODE_REPORT_BYTE_TIMINGS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&mode, sizeof(mode ));
  Wire.endTransmission();

  // Get data
  ir_byte_timings_t byte_timings;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( byte_timings ));
  Wire.readBytes( (uint8_t*)&byte_timings, sizeof( byte_timings ));

  // Return result
  return byte_timings;

}


ir_errors_t       SwarmB3_c::getErrors() {
  // Set correct more
  ir_mode_t mode;
  mode = MODE_REPORT_ERRORS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&mode, sizeof( mode));
  Wire.endTransmission();

  // Get data
  ir_errors_t errors;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( errors ));
  Wire.readBytes( (uint8_t*)&errors, sizeof( errors ));

  // Return result
  return errors;
}


ir_crc_t          SwarmB3_c::getCRC() {

  ir_mode_t mode;

  mode = MODE_REPORT_CRC;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&mode, sizeof( mode ));
  Wire.endTransmission();

  ir_crc_t crc;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( crc ));
  Wire.readBytes( (uint8_t*)&crc, sizeof( crc ));
  return crc;
}

ir_sensors_t      SwarmB3_c::getSensors() {
  // Set correct more
  ir_mode_t mode;
  mode = MODE_REPORT_SENSORS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&mode, sizeof( mode));
  Wire.endTransmission();

  // Get data
  ir_sensors_t sensors;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( sensors ));
  Wire.readBytes( (uint8_t*)&sensors, sizeof( sensors  ));

  // Return result
  return sensors;

}
