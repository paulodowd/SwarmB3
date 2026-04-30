
#ifndef IRCOMM_H
#define IRCOMM_H

#include "ircomm_i2c.h"
#include "sercom_funcs.h"
#include "ir_parser.h"

extern IRParser_c parser[4];

void      fullReset();
void      handleI2cFlags();
void      printStatus();
void      configureFromConfigH();
void      resetMetrics();
void      setBearingTimestamp();
uint32_t  calcBearingDeltaTime();
void      zeroBearingActivity();
void      setAllMsgTimestamps();
void      setMsgTimestamp( int which );
uint32_t  calcMsgDeltaTime( int which );
void      setAllByteTimestamps();
void      setByteTimestamp( int which );
uint32_t  calcByteDeltaTime( int which );
void      calcAllByteDeltaTime();
void      triggerTx( int which );
bool      attemptMsgWriteToSerialBuffer( int which );
bool      updateTx( int which );
void      resetRxBuffers( int which );
void      disableDemodulator( int which, DemodState d_state );
void      enableDemodulator( int which );
bool      triggerDemodDesaturation(int which);
bool      updateDemodDesaturation( int which );
void      updateBearing();
void      handleMsgParsing();
void      handleDemodulatorSaturation();
void      handleBearingEstimation();
void      handleTxBroadcast();
bool      recentByteActivity( int which );
void      handleTx( int which );
uint32_t  getNewTxInterval( int which );
void      handleTransmit();
void      printDemodStatus();

#endif
