
/*

   Functions to handle i2c transactions.  Quite extensive
   where the configuration of the board is available.

*/

#include "ircomm_i2c.h"
#include "ir_parser.h"

volatile uint8_t      last_mode;
volatile ir_metrics_t metrics;
volatile ir_config_t  config;

volatile bool i2c_flag_reset_metrics;
volatile bool i2c_flag_full_reset;
volatile bool i2c_flag_set_tx_0;
volatile bool i2c_flag_set_tx_1;
volatile bool i2c_flag_set_tx_2;
volatile bool i2c_flag_set_tx_3;
volatile bool i2c_flag_set_tx_all;
volatile uint8_t i2c_buf[4][MAX_MSG];
volatile uint8_t i2c_tx_len[4];

void i2cInitState() {
  last_mode               = MODE_NOT_SET;
  i2c_flag_reset_metrics  = false;
  i2c_flag_full_reset     = false;
  i2c_flag_set_tx_0       = false;
  i2c_flag_set_tx_1       = false;
  i2c_flag_set_tx_2       = false;
  i2c_flag_set_tx_3       = false;
  i2c_flag_set_tx_all     = false;
  memset( (uint8_t*)i2c_tx_len, 0, sizeof( i2c_tx_len ));
}

void i2cSetMsgStatusBit( int which ) {
  if ( which < 0 || which > 3 ) return;
  config.status |= (1 << which );
}

void i2cClearRxActivityBits() {
  config.status &= 0b00001111;
}
void i2cClearMsgStatusBit( int which ) {
  if ( which < 0 || which > 3 ) return;

  // clear related bit in status byte
  config.status &= ~(1 << which );

}
void i2cSetRxActivityBit( int which ) {
  if ( which < 0 || which > 3 ) return;
  config.status |= (1 << (which + 4) );
}

void i2cClearRxActivityBit( int which ) {
  if ( which < 0 || which > 3 ) return;

  // clear related bit in status byte
  config.status &= ~(1 << (which + 4) );

}
void i2cClearStatusBits() {
  config.status = 0;
  return;
}


void i2c_receive( int len ) {

  if ( len == 1 && last_mode == MODE_NOT_SET ) {

    // Read in new mode
    ir_mode_t new_mode;
    Wire.readBytes( (uint8_t*)&new_mode, len );

    // Act on any direct requests, or record a specific
    // mode change for the next multi-byte receive into
    // last_mode
    switch ( new_mode ) {

      case  MODE_CLEAR_MSG0:
        i2cClearMsgStatusBit(0);
        config.msg_len[0] = 0;
        break;

      case  MODE_CLEAR_MSG1:
        i2cClearMsgStatusBit(1);
        config.msg_len[1] = 0;
        break;

      case  MODE_CLEAR_MSG2:
        i2cClearMsgStatusBit(2);
        config.msg_len[2] = 0;
        break;

      case  MODE_CLEAR_MSG3:
        i2cClearMsgStatusBit(3);
        config.msg_len[3] = 0;
        break;

      // Context changes:
      //      case  MODE_SET_MSG_ALL:
      //      case  MODE_SET_MSG_0:
      //      case  MODE_SET_MSG_1:
      //      case  MODE_SET_MSG_2:
      //      case  MODE_SET_MSG_3:
      //      case  MODE_SET_TX_CONFIG_0:
      //      case  MODE_SET_TX_CONFIG_1:
      //      case  MODE_SET_TX_CONFIG_2:
      //      case  MODE_SET_TX_CONFIG_3:
      //      case  MODE_SET_RX_CONFIG_0:
      //      case  MODE_SET_RX_CONFIG_1:
      //      case  MODE_SET_RX_CONFIG_2:
      //      case  MODE_SET_RX_CONFIG_3:
      //      case  MODE_SET_GEN_CONFIG:
      //      case  MODE_GET_TX_CONFIG_0:
      //      case  MODE_GET_TX_CONFIG_1:
      //      case  MODE_GET_TX_CONFIG_2:
      //      case  MODE_GET_TX_CONFIG_3:
      //      case  MODE_GET_RX_CONFIG_0:
      //      case  MODE_GET_RX_CONFIG_1:
      //      case  MODE_GET_RX_CONFIG_2:
      //      case  MODE_GET_RX_CONFIG_3:
      //      case  MODE_GET_GEN_CONFIG:
      //        last_mode = new_mode;
      //        break;

      case  MODE_FULL_RESET:
        i2c_flag_full_reset = true;
        break;

      case  MODE_RESET_METRICS:
        i2c_flag_reset_metrics = true;
        break;


      // Anything else is a context change for the
      // next operation
      default: last_mode = new_mode;
        break;
    }

    // END OF 1 BYTE MODE RECEIVES


    // START OF MULTI-BYTE RECEIVES
    // check which context we are receiving in
  } else if ( last_mode == MODE_SET_MSG_0 ) {

    Wire.readBytes( (uint8_t*)i2c_buf[0], len );
    i2c_tx_len[0] = len;
    i2c_flag_set_tx_0 = true;
    last_mode = MODE_NOT_SET;

  } else if ( last_mode == MODE_SET_MSG_1 ) {

    Wire.readBytes( (uint8_t*)i2c_buf[1], len );
    i2c_tx_len[1] = len;
    i2c_flag_set_tx_1 = true;
    last_mode = MODE_NOT_SET;

  } else if ( last_mode == MODE_SET_MSG_2 ) {

    Wire.readBytes( (uint8_t*)i2c_buf[2], len );
    i2c_tx_len[2] = len;
    i2c_flag_set_tx_2 = true;
    last_mode = MODE_NOT_SET;

  } else if ( last_mode == MODE_SET_MSG_3 ) {

    Wire.readBytes( (uint8_t*)i2c_buf[3], len );
    i2c_tx_len[3] = len;
    i2c_flag_set_tx_3 = true;
    last_mode = MODE_NOT_SET;

  } else if ( last_mode == MODE_SET_MSG_ALL ) {
    Wire.readBytes( (uint8_t*)i2c_buf[0], len );
    i2c_tx_len[0] = len;
    i2c_flag_set_tx_all = true;
    last_mode = MODE_NOT_SET;

  } else if ( last_mode == MODE_SET_RX_CONFIG_0 ) {
    if ( len == sizeof( ir_rx_params_t ) ) {
      Wire.readBytes( (uint8_t*)&config.rx[0], sizeof( config.rx[0] ));
    } else {
      while (Wire.available()) Wire.read();
    }
    last_mode = MODE_NOT_SET;


  } else if ( last_mode == MODE_SET_RX_CONFIG_1 ) {
    if ( len == sizeof( ir_rx_params_t ) ) {
      Wire.readBytes( (uint8_t*)&config.rx[1], sizeof( config.rx[1] ));

    } else {
      while (Wire.available()) Wire.read();
    }
    last_mode = MODE_NOT_SET;

  } else if ( last_mode == MODE_SET_RX_CONFIG_2 ) {
    if ( len == sizeof( ir_rx_params_t ) ) {
      Wire.readBytes( (uint8_t*)&config.rx[2], sizeof( config.rx[2] ));

    } else {
      while (Wire.available()) Wire.read();
    }

  } else if ( last_mode == MODE_SET_RX_CONFIG_3 ) {
    if ( len == sizeof( ir_rx_params_t ) ) {
      Wire.readBytes( (uint8_t*)&config.rx[3], sizeof( config.rx[3] ));

    } else {
      while (Wire.available()) Wire.read();
    }
    last_mode = MODE_NOT_SET;

  } else if ( last_mode == MODE_SET_TX_CONFIG_0 ) {
    if ( len == sizeof( ir_tx_params_t ) ) {
      Wire.readBytes( (uint8_t*)&config.tx[0], sizeof( config.tx[0] ));

    } else {
      while (Wire.available()) Wire.read();
    }
    last_mode = MODE_NOT_SET;


  } else if ( last_mode == MODE_SET_TX_CONFIG_1 ) {
    if ( len == sizeof( ir_tx_params_t ) ) {
      Wire.readBytes( (uint8_t*)&config.tx[1], sizeof( config.tx[1] ));

    } else {
      while (Wire.available()) Wire.read();
    }
    last_mode = MODE_NOT_SET;

  } else if ( last_mode == MODE_SET_TX_CONFIG_2 ) {
    if ( len == sizeof( ir_tx_params_t ) ) {
      Wire.readBytes( (uint8_t*)&config.tx[2], sizeof( config.tx[2] ));

    } else {
      while (Wire.available()) Wire.read();
    }
    last_mode = MODE_NOT_SET;
  } else if ( last_mode == MODE_SET_TX_CONFIG_3 ) {
    if ( len == sizeof( ir_tx_params_t ) ) {
      Wire.readBytes( (uint8_t*)&config.tx[3], sizeof( config.tx[3] ));

    } else {
      while (Wire.available()) Wire.read();
    }
    last_mode = MODE_NOT_SET;
  } else if ( last_mode == MODE_SET_GEN_CONFIG ) {
    if ( len == sizeof( ir_params_t ) ) {
      Wire.readBytes( (uint8_t*)&config.general, sizeof( config.general ));

    } else {
      while (Wire.available()) Wire.read();
    }
    last_mode = MODE_NOT_SET;
  }

}

void i2c_request() {

  switch ( last_mode ) {
    case  MODE_REPORT_STATUS:
      Wire.write( (uint8_t*)&config.status, sizeof(config.status) );
      break;

    case  MODE_REPORT_SIZE_MSG0:
      Wire.write( (uint8_t*)&config.msg_len[0], sizeof( config.msg_len[0] ));
      break;
    case  MODE_REPORT_SIZE_MSG1:
      Wire.write( (uint8_t*)&config.msg_len[1], sizeof( config.msg_len[1] ));
      break;
    case  MODE_REPORT_SIZE_MSG2:
      Wire.write( (uint8_t*)&config.msg_len[2], sizeof( config.msg_len[2] ));
      break;
    case  MODE_REPORT_SIZE_MSG3:
      Wire.write( (uint8_t*)&config.msg_len[3], sizeof( config.msg_len[3] ));
      break;

    case  MODE_REPORT_MSG0:
      Wire.write( (uint8_t*)&config.msg[0], sizeof( config.msg[0]) );
      i2cClearMsgStatusBit(0);
      config.msg_len[0] = 0;
      break;
    case  MODE_REPORT_MSG1:
      Wire.write( (uint8_t*)&config.msg[1], sizeof( config.msg[1]) );
      i2cClearMsgStatusBit(1);
      config.msg_len[1] = 0;
      break;
    case  MODE_REPORT_MSG2:
      Wire.write( (uint8_t*)&config.msg[2], sizeof( config.msg[2]) );
      i2cClearMsgStatusBit(2);
      config.msg_len[2] = 0;
      break;
    case  MODE_REPORT_MSG3:
      Wire.write( (uint8_t*)&config.msg[3], sizeof( config.msg[3]) );
      i2cClearMsgStatusBit(3);
      config.msg_len[3] = 0;
      break;

    case  MODE_REPORT_ACTIVITY:
      Wire.write( (uint8_t*)&metrics.activity, sizeof( metrics.activity ) );
      break;
    case  MODE_REPORT_VECTORS:
      Wire.write( (uint8_t*)&metrics.vectors, sizeof( metrics.vectors ) );
      break;
    case  MODE_REPORT_BEARING:
      Wire.write( (uint8_t*)&metrics.bearing, sizeof( metrics.bearing ) );
      break;

    case  MODE_REPORT_TX_TIMINGS:
      Wire.write( (uint8_t*)&metrics.tx_timings, sizeof( metrics.tx_timings) );
      break;
    case  MODE_REPORT_MSG_TIMINGS:
      Wire.write( (uint8_t*)&metrics.msg_timings, sizeof( metrics.msg_timings) );
      break;
    case  MODE_REPORT_BYTE_TIMINGS:
      Wire.write( (uint8_t*)&metrics.byte_timings, sizeof( metrics.byte_timings) );
      break;

    case  MODE_REPORT_CRC:
      Wire.write( (uint8_t*)&metrics.crc, sizeof( metrics.crc) );
      break;
    case  MODE_REPORT_FRAME_ERRS:
      Wire.write( (uint8_t*)&metrics.frame_errors, sizeof( metrics.frame_errors) );
      break;
    case  MODE_REPORT_ERRORS:
      Wire.write( (uint8_t*)&metrics.errors, sizeof( metrics.errors) );
      break;
    case  MODE_REPORT_SATURATION:
      Wire.write( (uint8_t*)&metrics.saturation, sizeof( metrics.saturation) );
      break;

    case  MODE_GET_GEN_CONFIG:
      Wire.write( (uint8_t*)&config.general, sizeof( config.general ) );
      break;

    case  MODE_GET_TX_CONFIG_0:
      Wire.write( (uint8_t*)&config.tx[0], sizeof( config.tx[0]) );
      break;
    case  MODE_GET_TX_CONFIG_1:
      Wire.write( (uint8_t*)&config.tx[1], sizeof( config.tx[1]) );
      break;
    case  MODE_GET_TX_CONFIG_2:
      Wire.write( (uint8_t*)&config.tx[2], sizeof( config.tx[2]) );
      break;
    case  MODE_GET_TX_CONFIG_3:
      Wire.write( (uint8_t*)&config.tx[3], sizeof( config.tx[3]) );
      break;

    case  MODE_GET_RX_CONFIG_0:
      Wire.write( (uint8_t*)&config.rx[0], sizeof( config.rx[0]) );
      break;
    case  MODE_GET_RX_CONFIG_1:
      Wire.write( (uint8_t*)&config.rx[1], sizeof( config.rx[1]) );
      break;
    case  MODE_GET_RX_CONFIG_2:
      Wire.write( (uint8_t*)&config.rx[2], sizeof( config.rx[2]) );
      break;
    case  MODE_GET_RX_CONFIG_3:
      Wire.write( (uint8_t*)&config.rx[3], sizeof( config.rx[3]) );
      break;
    case MODE_REPORT_SENSORS:
      // TODO
      break;
  }

  last_mode = MODE_NOT_SET;

}
