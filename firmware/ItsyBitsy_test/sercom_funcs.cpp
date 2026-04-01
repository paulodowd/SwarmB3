

#include "sercom_funcs.h"
//
//#include <Arduino.h>
#include <wiring_private.h>
#include "sam.h" // for SAMD51 registers

// -----------------------------------------------------------------------------
// Frame error counters per UART
// -----------------------------------------------------------------------------
volatile uint32_t Serial1_frame_errors = 0;
volatile uint32_t SerialA4_frame_errors = 0;
volatile uint32_t SerialD12_frame_errors = 0;
volatile uint32_t SerialSPI_frame_errors = 0;

// -----------------------------------------------------------------------------
// UART objects
// TODO: rename all of them to better indicate the direction
//       of the receivers and transmitters with respect to
//       the robot body.
// -----------------------------------------------------------------------------

// To "rename" the built-in Serial1 for readability.
Uart& uart1 = Serial1;

// UART2: A4 TX / A1 RX  -> SERCOM0
Uart SerialA4(&sercom0, PIN_A5, PIN_A4, SERCOM_RX_PAD_2, UART_TX_PAD_0);

// UART3: D12 TX / D13 RX -> SERCOM5
Uart SerialD12(&sercom5, 13, 12, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// UART4: MOSI TX / SCK RX -> SERCOM1
Uart SerialSPI(&sercom1, PIN_SPI_SCK, PIN_SPI_MOSI, SERCOM_RX_PAD_1, UART_TX_PAD_0);

UartChannel uart_channel[4] = {
  {"front", SERCOM3, &Serial1, &Serial1_frame_errors, TxRxState::RX },
  {"front", SERCOM0, &SerialA4, &SerialA4_frame_errors, TxRxState::RX },
  {"front", SERCOM5, &SerialD12, &SerialD12_frame_errors, TxRxState::RX },
  {"front", SERCOM1, &SerialSPI, &SerialSPI_frame_errors, TxRxState::RX }
};




// -----------------------------------------------------------------------------
// Helper: count frame error if present
// We test STATUS.FERR directly on the raw CMSIS SERCOM peripheral.
// -----------------------------------------------------------------------------
static inline void countFrameError(Sercom *hw, volatile uint32_t &counter) {
  if (hw->USART.INTFLAG.bit.ERROR && hw->USART.STATUS.bit.FERR) {
    counter++;
  }
}


// -----------------------------------------------------------------------------
// Reset helpers
// -----------------------------------------------------------------------------
void resetAllFrameErrorCounts() {
  noInterrupts();
  //  uart1_frame_errors = 0;
  uart_channel[0].frame_errors = 0;
  uart_channel[1].frame_errors = 0;
  uart_channel[2].frame_errors = 0;
  uart_channel[3].frame_errors = 0;

  interrupts();
}

void resetFrameErrorCount( UartChannel& ch ) {
  noInterrupts();
  *(ch.frame_errors) = 0;
  interrupts();
}

// Optional read helper
uint32_t getFrameErrorCount( UartChannel& ch ) {
  noInterrupts();
  uint32_t value = *(ch.frame_errors);
  interrupts();
  return value;
}




// -----------------------------------------------------------------------------
// Invert UART logic helpers
// NOTE: SAMD5x errata says TXINV and RXINV are swapped on affected silicon.
// So to invert TX, set RXINV.
// -----------------------------------------------------------------------------
static void configureSercomInvert(Sercom* hw, bool invertTx, bool invertRx) {
  hw->USART.CTRLA.bit.ENABLE = 0;
  while (hw->USART.SYNCBUSY.bit.ENABLE) {
  }

  // SAMD5x/E5x errata: TXINV and RXINV are swapped
  hw->USART.CTRLA.bit.RXINV = invertTx ? 1 : 0;  // actually TX
  hw->USART.CTRLA.bit.TXINV = invertRx ? 1 : 0;  // actually RX

  hw->USART.CTRLA.bit.ENABLE = 1;
  while (hw->USART.SYNCBUSY.bit.ENABLE) {
  }
}

/* I've had a lot of trouble trying to get SERCOM0
   working using A4/A5, or any other pins.  Doing
   low level register config rather than calling
   Serial.begin() seems to work. I am guessing the
   built in function is mucking up the MUX.
*/

void beginSerialA4A5_manual(uint32_t baud) {
  // A4 = TX = PAD0
  // A5 = RX = PAD2
  pinPeripheral(PIN_A4, PIO_SERCOM_ALT);
  pinPeripheral(PIN_A5, PIO_SERCOM_ALT);

  sercom0.initUART(UART_INT_CLOCK, SAMPLE_RATE_x16, baud);
  sercom0.initFrame(UART_CHAR_SIZE_8_BITS,
                    LSB_FIRST,
                    SERCOM_NO_PARITY,
                    SERCOM_STOP_BIT_1);
  sercom0.initPads(UART_TX_PAD_0, SERCOM_RX_PAD_2);
  sercom0.enableUART();
}




void disableSercomRx(Sercom* hw) {
  // Optional: wait for any current receive operation to settle
  //while (hw->USART.SYNCBUSY.bit.CTRLB) {
  //}

  // Disable receiver
  hw->USART.CTRLB.bit.RXEN = 0;

  // Wait for synchronization
  while (hw->USART.SYNCBUSY.bit.CTRLB) {
  }
}

void enableSercomRx(Sercom* hw) {
  // Enable receiver
  hw->USART.CTRLB.bit.RXEN = 1;

  // Wait for synchronization
  while (hw->USART.SYNCBUSY.bit.CTRLB) {
  }


}

void clearSercomRxBuffer(Sercom* hw) {
  // Drain any received data
  while (hw->USART.INTFLAG.bit.RXC) {
    volatile uint8_t dummy = hw->USART.DATA.reg;
    (void)dummy;
  }

  // Clear error flags
  hw->USART.STATUS.reg =
    SERCOM_USART_STATUS_FERR |
    SERCOM_USART_STATUS_BUFOVF |
    SERCOM_USART_STATUS_PERR;

  // Clear error interrupt flag
  hw->USART.INTFLAG.reg = SERCOM_USART_INTFLAG_ERROR;
}

bool isUartTxComplete( const UartChannel& ch ) {
  // Check software buffer empty AND hardware shift register empty
  return (ch.port->availableForWrite() == SERIAL_BUFFER_SIZE) &&
         (ch.hw->USART.INTFLAG.bit.TXC);
}

void disableSercomTx(Sercom* hw) {
  // Optional: wait until any ongoing transmission is complete
  while (!hw->USART.INTFLAG.bit.TXC) {
  }

  // Disable transmitter
  hw->USART.CTRLB.bit.TXEN = 0;

  // Wait for synchronization
  while (hw->USART.SYNCBUSY.bit.CTRLB) {
  }

  // TODO
  // May need to set the pin low here
}

void enableSercomTx(Sercom* hw) {
  // Enable transmitter
  hw->USART.CTRLB.bit.TXEN = 1;

  // Wait for synchronization
  while (hw->USART.SYNCBUSY.bit.CTRLB) {
  }
}



// -----------------------------------------------------------------------------
// SERCOM IRQ handlers. No prototypes in .h
// Invoked directly by hardware.
// Count frame error first, then hand over to the normal Arduino UART handler.
// Note: SERCOM3 (channel 0) is not being tracked
// -----------------------------------------------------------------------------
void SERCOM0_0_Handler() {
  countFrameError( SERCOM0, SerialA4_frame_errors);
  SerialA4.IrqHandler();
}
void SERCOM0_1_Handler() {
  countFrameError( SERCOM0, SerialA4_frame_errors);
  SerialA4.IrqHandler();
}
void SERCOM0_2_Handler() {
  countFrameError( SERCOM0, SerialA4_frame_errors);
  SerialA4.IrqHandler();
}
void SERCOM0_3_Handler() {
  countFrameError( SERCOM0, SerialA4_frame_errors);
  SerialA4.IrqHandler();
}
void SERCOM1_0_Handler() {
  countFrameError( SERCOM1, SerialSPI_frame_errors);
  SerialSPI.IrqHandler();
}
void SERCOM1_1_Handler() {
  countFrameError( SERCOM1, SerialSPI_frame_errors);
  SerialSPI.IrqHandler();
}
void SERCOM1_2_Handler() {
  countFrameError( SERCOM1, SerialSPI_frame_errors);
  SerialSPI.IrqHandler();
}
void SERCOM1_3_Handler() {
  countFrameError( SERCOM1, SerialSPI_frame_errors);
  SerialSPI.IrqHandler();
}
void SERCOM5_0_Handler() {
  countFrameError( SERCOM5, SerialD12_frame_errors);
  SerialD12.IrqHandler();
}
void SERCOM5_1_Handler() {
  countFrameError( SERCOM5, SerialD12_frame_errors);
  SerialD12.IrqHandler();
}
void SERCOM5_2_Handler() {
  countFrameError( SERCOM5, SerialD12_frame_errors);
  SerialD12.IrqHandler();
}
void SERCOM5_3_Handler() {
  countFrameError( SERCOM5, SerialD12_frame_errors);
  SerialD12.IrqHandler();
}
//// Because I'm using the built-in Serial1, I can't override the handler.  To get it to work for
// SERCOM3, I think I would need to edit the variant file.  Getting frame errors on 3 of the
// receivers is probably enough for my research needs (although annoying).
//void SERCOM3_0_Handler() { countFrameError(SERCOM3, uart1_frame_errors); Serial1.IrqHandler(); }
//void SERCOM3_1_Handler() { countFrameError(SERCOM3, uart1_frame_errors); Serial1.IrqHandler(); }
//void SERCOM3_2_Handler() { countFrameError(SERCOM3, uart1_frame_errors); Serial1.IrqHandler(); }
//void SERCOM3_3_Handler() { countFrameError(SERCOM3, uart1_frame_errors); Serial1.IrqHandler(); }
