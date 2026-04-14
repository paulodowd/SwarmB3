#ifndef SERCOM_FUNCS_H
#define SERCOM_FUNCS_H

#include <Arduino.h>

// extern for linking
extern Uart port_D1_D0; // alias to Serial1
extern Uart port_D18_D15;
extern Uart port_D25_D24;
extern Uart port_D12_D13;
//
//enum class DemodState : uint8_t {
//  OFF = 0,
//  STABILISING = 1,
//  ON = 2
//};


enum class TxReleasePhase : uint8_t {
  Idle,
  WaitingForComplete
};

struct UartChannel {
  Sercom *  hw; // raw CMSIS peripheral, register access
  Uart * port;  // Arduino UART/Serial wrapper class
  int tx_pin;
  int rx_pin;
  volatile uint32_t * frame_errors;
  volatile TxReleasePhase tx_phase;
  volatile bool tx_release_done;
};


extern UartChannel channel[4];

void setup58kHz();
void stop58kHzClockAndHoldLow();

void configureSercomInvert(Sercom* hw, bool invertTx, bool invertRx);
void beginSerialA4A1_manual(uint32_t baud);
static inline void countFrameError(Sercom *hw, volatile uint32_t &counter);
void resetAllFrameErrorCounts();
void resetFrameErrorCount( UartChannel& ch );
uint32_t getFrameErrorCount( UartChannel& ch );
void enableSercomTx(Sercom* hw);
void disableSercomTx(Sercom* hw);
bool isUartTxComplete( const UartChannel& ch );
bool isUartTxComplete(Sercom* hw);
void waitForTxComplete(Sercom* hw);
void uartSendAndRelease(Uart& uart, Sercom* hw, uint8_t txPin, const uint8_t* data, size_t len, bool inv);
void uartSendAndRelease(UartChannel& ch, const uint8_t* data, size_t len);
bool uartSendAndReleaseAsync(UartChannel& ch, const uint8_t* data, size_t len);
void restoreSercomTx(Sercom* hw, uint8_t txPin);
void clearSercomRxBuffer(Sercom* hw);
void disableSercomRx(Sercom* hw);
void enableSercomRx(Sercom* hw);
void configureSercom8N1(Sercom* hw);

#endif
