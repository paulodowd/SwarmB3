#ifndef SERCOM_FUNCS_H
#define SERCOM_FUNCS_H

#include <Arduino.h>

// extern for linking
extern Uart SerialA4;
extern Uart SerialD12;
extern Uart SerialSPI;
//
//enum class DemodState : uint8_t {
//  OFF = 0,
//  STABILISING = 1,
//  ON = 2
//};


// Struct to group them all up and keep things
// consistent.
enum class TxRxState : uint8_t {
  RX = 0,
  TX = 1
};

struct UartChannel {
  const char * name;
  Sercom *  hw; // raw CMSIS peripheral, register access
  Uart * port;  // Arduino UART/Serial wrapper class
  volatile uint32_t * frame_errors;
  TxRxState tx_rx_state;
};

extern UartChannel uart_channel[4];

void configureSercomInvert(Sercom* hw, bool invertTx, bool invertRx);
void beginSerialA4A1_manual(uint32_t baud);
static inline void countFrameError(Sercom *hw, volatile uint32_t &counter);
void resetAllFrameErrorCounts();
void resetFrameErrorCount( UartChannel& ch );
uint32_t getFrameErrorCount( UartChannel& ch );
void enableSercomTx(Sercom* hw);
void disableSercomTx(Sercom* hw);
bool isUartTxComplete( const UartChannel& ch );
void clearSercomRxBuffer(Sercom* hw);
void disableSercomRx(Sercom* hw);
void enableSercomRx(Sercom* hw);

#endif
