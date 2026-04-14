

#include "sercom_funcs.h"
//
//#include <Arduino.h>
#include <wiring_private.h>
#include "sam.h" // for SAMD51 registers

// -----------------------------------------------------------------------------
// Frame error counters per UART
// -----------------------------------------------------------------------------
volatile uint32_t port_D1_D0_frame_errors = 0;
volatile uint32_t port_D18_D15_frame_errors = 0;
volatile uint32_t port_D12_D13_frame_errors = 0;
volatile uint32_t port_D25_D24_frame_errors = 0;

// -----------------------------------------------------------------------------
// UART objects
// TODO: rename all of them to better indicate the direction
//       of the receivers and transmitters with respect to
//       the robot body.
// -----------------------------------------------------------------------------

//IRParser_c parser[4] = { port_D12_D13, port_D25_D24, port_D18_D15, port_D1_D0 };
UartChannel channel[4] = {
  {SERCOM5, &port_D12_D13, 12, 13, &port_D12_D13_frame_errors, TxReleasePhase::Idle, false },
  {SERCOM1, &port_D25_D24, 25, 24, &port_D25_D24_frame_errors, TxReleasePhase::Idle, false },
  {SERCOM0, &port_D18_D15, 18, 15, &port_D18_D15_frame_errors, TxReleasePhase::Idle, false },
  {SERCOM3, &port_D1_D0, 1, 0, &port_D1_D0_frame_errors, TxReleasePhase::Idle, false }
};


// To "rename" the built-in Serial1 for readability.

//Uart& port_D1_D0 = Serial1;

Uart port_D1_D0(&sercom3, 0, 1, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// UART2: A4 TX / A1 RX  -> SERCOM0
Uart port_D18_D15(&sercom0, 15, 18, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// UART3: D12 TX / D13 RX -> SERCOM5
Uart port_D12_D13(&sercom5, 13, 12, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// UART4: MOSI TX / SCK RX -> SERCOM1
Uart port_D25_D24(&sercom1, 24, 25, SERCOM_RX_PAD_1, UART_TX_PAD_0);



// -----------------------------------------------------------------------------
// 58 kHz clock on D7 using TCC1
// D7 = PA18 = TCC1/WO[2]
// -----------------------------------------------------------------------------
void setup58kHz() {
  // D7 on ItsyBitsy M4 = PA18
  // PA18 can be muxed to TCC1/WO[2]

  // Configure PA18 for peripheral output
  PORT->Group[0].DIRSET.reg = (1 << 18);
  PORT->Group[0].PMUX[18 >> 1].bit.PMUXE = MUX_PA18F_TCC1_WO2;
  PORT->Group[0].PINCFG[18].bit.PMUXEN = 1;

  // Enable bus clock for TCC1
  MCLK->APBBMASK.reg |= MCLK_APBBMASK_TCC1;

  // Route GCLK0 to TCC1
  // On the ItsyBitsy M4, the board runs at 120 MHz. If GCLK0 is 120 MHz,
  // this gives the frequency calculation below.
  GCLK->PCHCTRL[TCC1_GCLK_ID].reg =
    GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;
  while (!(GCLK->PCHCTRL[TCC1_GCLK_ID].reg & GCLK_PCHCTRL_CHEN));

  // Disable TCC1 before configuration
  TCC1->CTRLA.bit.ENABLE = 0;
  while (TCC1->SYNCBUSY.bit.ENABLE);

  // Prescaler = 1
  TCC1->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1;

  // Normal PWM mode
  TCC1->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
  while (TCC1->SYNCBUSY.bit.WAVE);

  // 120 MHz / (2068 + 1) = 57,999 Hz
  TCC1->PER.reg = 2068;
  while (TCC1->SYNCBUSY.bit.PER);

  // ~50% duty cycle
  TCC1->CC[2].reg = 1034;
  while (TCC1->SYNCBUSY.bit.CC2);

  // Enable TCC1
  TCC1->CTRLA.bit.ENABLE = 1;
  while (TCC1->SYNCBUSY.bit.ENABLE);
}

void stop58kHzClockAndHoldLow() {
  // Disable TCC1 (stop waveform generation)
  TCC1->CTRLA.bit.ENABLE = 0;
  while (TCC1->SYNCBUSY.bit.ENABLE);

  // Optionally also disable the specific waveform output channel
  // (WO[x] depends on your configuration, likely WO[2] for PA18)
  TCC1->DRVCTRL.reg = 0;

  // Detach peripheral and return pin to GPIO
  pinPeripheral(7, PIO_DIGITAL);

  // Drive pin LOW
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);
}

void clearUartTxc(Sercom* hw) {
  // On SAMD SERCOM USART, interrupt flags are cleared by writing 1 to the bit.
  hw->USART.INTFLAG.reg = SERCOM_USART_INTFLAG_TXC;
}

void enableUartTxcInterrupt(Sercom* hw) {
  hw->USART.INTENSET.reg = SERCOM_USART_INTENSET_TXC;
}

void disableUartTxcInterrupt(Sercom* hw) {
  hw->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_TXC;
}

void forcePinLow(uint8_t pin) {
  pinPeripheral(pin, PIO_DIGITAL);  // detach SERCOM
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

bool uartSendAndReleaseAsync(UartChannel& ch, const uint8_t* data, size_t len) {
  if (ch.tx_phase != TxReleasePhase::Idle) {
    return false;
  }

  restoreSercomTx(ch.hw, ch.tx_pin);

  ch.tx_release_done = false;
  ch.tx_phase = TxReleasePhase::WaitingForComplete;

  clearUartTxc(ch.hw);

  char buf[8];
  memset(buf, 0, sizeof( buf ));

  for ( int i = 0; i < 8; i++ ) {
    buf[i] = 0x55;

  }
  ch.port->write(buf, 8);
  ch.port->write(data, len);
  enableUartTxcInterrupt(ch.hw);

  return true;
}

void handleTxCompleteInterrupt(UartChannel& ch) {
  if (ch.hw->USART.INTFLAG.bit.TXC && ch.hw->USART.INTENSET.bit.TXC) {
    disableUartTxcInterrupt(ch.hw);
    clearUartTxc(ch.hw);

    disableSercomTx(ch.hw);
    forcePinLow(ch.tx_pin);

    ch.tx_phase = TxReleasePhase::Idle;
    ch.tx_release_done = true;
  }
}






bool isUartTxComplete(Sercom* hw) {
  return hw->USART.INTFLAG.bit.TXC;
}
void waitForTxComplete(Sercom* hw) {
  while (!isUartTxComplete(hw)) {
    // spin
  }
}

void restoreSercomTx(Sercom* hw, uint8_t txPin) {
  // Reconnect SERCOM to the pin
  pinPeripheral(txPin, PIO_SERCOM_ALT);

  // Re-enable transmitter
  hw->USART.CTRLB.bit.TXEN = 1;
  while (hw->USART.SYNCBUSY.bit.CTRLB) {
  }
}
void uartSendAndRelease(UartChannel& ch, const uint8_t* data, size_t len) {

  setup58kHz();

  //  restoreSercomTx(ch.hw, ch.tx_pin);
  ch.port->write(data, len);

  // Wait for full transmission (shift register empty)
  while (!isUartTxComplete(ch.hw)) {
  }

  // Disable transmitter
  //  disableSercomTx(ch.hw);

  // Force line low
  //  forcePinLow( ch.tx_pin );
  stop58kHzClockAndHoldLow();
}

void uartSendAndRelease(Uart& uart, Sercom* hw, uint8_t txPin, const uint8_t* data, size_t len, bool inv) {

  setup58kHz();
  delayMicroseconds(800);
  //  restoreSercomTx(hw, txPin);

//  beginSerialA4A1_manual(9600);
//  configureSercomInvert(hw, inv, false);
//  delay(1);
  uart.write(data, len);

  // Wait for full transmission (shift register empty)
  while (!isUartTxComplete(hw)) {
  }
  stop58kHzClockAndHoldLow();

  // Disable transmitter
  //  disableSercomTx(hw);

  // Force line low
  //  forcePinLow(txPin);
}


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
  channel[0].frame_errors = 0;
  channel[1].frame_errors = 0;
  channel[2].frame_errors = 0;
  channel[3].frame_errors = 0;

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
void configureSercomInvert(Sercom* hw, bool invertTx, bool invertRx) {
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

void beginSerialA4A1_manual(uint32_t baud) {
  // A4 = TX = PAD0
  // A5 = RX = PAD2
  pinPeripheral(PIN_A4, PIO_SERCOM_ALT);
  pinPeripheral(PIN_A1, PIO_SERCOM_ALT);

  sercom0.initUART(UART_INT_CLOCK, SAMPLE_RATE_x16, baud);
  sercom0.initFrame(UART_CHAR_SIZE_8_BITS,
                    LSB_FIRST,
                    SERCOM_NO_PARITY,
                    SERCOM_STOP_BIT_1);
  sercom0.initPads(UART_TX_PAD_0, SERCOM_RX_PAD_1);
  sercom0.enableUART();
}

void configureSercom8N1(Sercom* hw) {
  // Disable USART before modifying enable-protected registers
  hw->USART.CTRLA.reg &= ~SERCOM_USART_CTRLA_ENABLE;
  while (hw->USART.SYNCBUSY.bit.ENABLE) {}

  // ---- CTRLA: parity + mode ----
  // Clear parity enable (FORM = 0 → no parity)
  hw->USART.CTRLA.reg &= ~SERCOM_USART_CTRLA_FORM_Msk;

  // ---- CTRLB: character size + stop bits ----
  uint32_t ctrlb = hw->USART.CTRLB.reg;

  // 8-bit character size
  ctrlb &= ~SERCOM_USART_CTRLB_CHSIZE_Msk;
  ctrlb |= SERCOM_USART_CTRLB_CHSIZE(0x0);  // 0x0 = 8-bit

  // 1 stop bit
  ctrlb &= ~SERCOM_USART_CTRLB_SBMODE;      // 0 = 1 stop bit

  // Disable parity explicitly (safety)
  ctrlb &= ~SERCOM_USART_CTRLB_PMODE;

  hw->USART.CTRLB.reg = ctrlb;
  while (hw->USART.SYNCBUSY.bit.CTRLB) {}

  // Re-enable USART
  hw->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
  while (hw->USART.SYNCBUSY.bit.ENABLE) {}
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

  // Disable transmitter
  hw->USART.CTRLB.bit.TXEN = 0;

  // Wait for synchronization
  while (hw->USART.SYNCBUSY.bit.CTRLB) {
  }
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


void SERCOM3_0_Handler() {
  //countFrameError( SERCOM0, port_D1_D0_frame_errors);
  port_D1_D0.IrqHandler();
  //handleTxCompleteInterrupt( channel[3] );

}
void SERCOM3_1_Handler() {
  //countFrameError( SERCOM0, port_D1_D0_frame_errors);
  port_D1_D0.IrqHandler();
  //handleTxCompleteInterrupt( channel[3] );
}
void SERCOM3_2_Handler() {
  //countFrameError( SERCOM0, port_D1_D0_frame_errors);
  port_D1_D0.IrqHandler();
  //handleTxCompleteInterrupt( channel[3] );
}
void SERCOM3_3_Handler() {
  //countFrameError( SERCOM0, port_D1_D0_frame_errors);
  port_D1_D0.IrqHandler();
  //handleTxCompleteInterrupt( channel[3] );
}


void SERCOM0_0_Handler() {
  //countFrameError( SERCOM0, port_D18_D15_frame_errors);
  port_D18_D15.IrqHandler();
  //handleTxCompleteInterrupt( channel[2] );
}
void SERCOM0_1_Handler() {
  //countFrameError( SERCOM0, port_D18_D15_frame_errors);
  port_D18_D15.IrqHandler();
  //handleTxCompleteInterrupt( channel[2] );
}
void SERCOM0_2_Handler() {
  //countFrameError( SERCOM0, port_D18_D15_frame_errors);
  port_D18_D15.IrqHandler();
  //handleTxCompleteInterrupt( channel[2] );
}
void SERCOM0_3_Handler() {
  //countFrameError( SERCOM0, port_D18_D15_frame_errors);
  port_D18_D15.IrqHandler();
  //handleTxCompleteInterrupt( channel[2] );
}

void SERCOM1_0_Handler() {
  //countFrameError( SERCOM1, port_D25_D24_frame_errors);
  port_D25_D24.IrqHandler();
  //handleTxCompleteInterrupt( channel[1] );
}
void SERCOM1_1_Handler() {
  //countFrameError( SERCOM1, port_D25_D24_frame_errors);
  port_D25_D24.IrqHandler();
  //handleTxCompleteInterrupt( channel[1] );
}
void SERCOM1_2_Handler() {
  //countFrameError( SERCOM1, port_D25_D24_frame_errors);
  port_D25_D24.IrqHandler();
  //handleTxCompleteInterrupt( channel[1] );
}
void SERCOM1_3_Handler() {
  //countFrameError( SERCOM1, port_D25_D24_frame_errors);
  port_D25_D24.IrqHandler();
  //handleTxCompleteInterrupt( channel[1] );
}

void SERCOM5_0_Handler() {
  //countFrameError( SERCOM5, port_D12_D13_frame_errors);
  port_D12_D13.IrqHandler();
  //handleTxCompleteInterrupt( channel[0] );
}
void SERCOM5_1_Handler() {
  //countFrameError( SERCOM5, port_D12_D13_frame_errors);
  port_D12_D13.IrqHandler();
  //handleTxCompleteInterrupt( channel[0] );
}
void SERCOM5_2_Handler() {
  //countFrameError( SERCOM5, port_D12_D13_frame_errors);
  port_D12_D13.IrqHandler();
  //handleTxCompleteInterrupt( channel[0] );
}
void SERCOM5_3_Handler() {
  //countFrameError( SERCOM5, port_D12_D13_frame_errors);
  port_D12_D13.IrqHandler();
  //handleTxCompleteInterrupt( channel[0] );
}
//// Because I'm using the built-in Serial1, I can't override the handler.  To get it to work for
// SERCOM3, I think I would need to edit the variant file.  Getting frame errors on 3 of the
// receivers is probably enough for my research needs (although annoying).
//void SERCOM3_0_Handler() { //countFrameError(SERCOM3, uart1_frame_errors); Serial1.IrqHandler(); }
//void SERCOM3_1_Handler() { //countFrameError(SERCOM3, uart1_frame_errors); Serial1.IrqHandler(); }
//void SERCOM3_2_Handler() { //countFrameError(SERCOM3, uart1_frame_errors); Serial1.IrqHandler(); }
//void SERCOM3_3_Handler() { //countFrameError(SERCOM3, uart1_frame_errors); Serial1.IrqHandler(); }
