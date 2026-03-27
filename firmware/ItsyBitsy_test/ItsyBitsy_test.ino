#include <Arduino.h>
#include <wiring_private.h>
#include <Wire.h>

//#include "sam.h" // for SAMD51 registers

#include "ir_parser.h"

// GPIO mappings
#define DEMOD1_EN_PIN PIN_PA21
#define DEMOD2_EN_PIN PIN_PA20
#define DEMOD3_EN_PIN PIN_PA19
#define DEMOD4_EN_PIN PIN_PB23
#define IRCLK_58K_PIN PIN_PA18
#define PROXA_IN_PIN  PIN_PA02
#define PROXB_IN_PIN  PIN_PB08
#define LDRA_IN_PIN   PIN_PB09
#define LDRB_IN_PIN   PIN_PA06
#define LDRC_IN_PIN   PIN_PA07


// -----------------------------------------------------------------------------
// UART objects
// TODO: rename all of them to better indicate the direction
//       of the receivers and transmitters with respect to 
//       the robot body.
// -----------------------------------------------------------------------------

// To "rename" the built-in Serial1 for readability.
Uart& uart1 = Serial1;

// UART2: A4 TX / A1 RX  -> SERCOM0
Uart SerialA4(&sercom0, PIN_A1, PIN_A4, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// UART3: D12 TX / D13 RX -> SERCOM5
Uart SerialD12(&sercom5, PIN_PA22, PIN_PA23, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// UART4: MOSI TX / SCK RX -> SERCOM1
Uart SerialSPI(&sercom1, PIN_SPI_SCK, PIN_SPI_MOSI, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// Match up 4 instances of the ir parsrer.
IRParser_c ir_parser[4] = { Serial1, SerialA4, SerialD12, SerialSPI };

// -----------------------------------------------------------------------------
// Frame error counters
// -----------------------------------------------------------------------------
//volatile uint32_t uart1_frame_errors = 0;   // Serial1 / SERCOM3
volatile uint32_t SerialA4_frame_errors = 0;   // SerialA4 / SERCOM0
volatile uint32_t SerialD12_frame_errors = 0;   // SerialD12 / SERCOM5
volatile uint32_t SerialSPI_frame_errors = 0;   // SerialSPI / SERCOM1

// -----------------------------------------------------------------------------
// Helper: count frame error if present
// We test STATUS.FERR directly on the raw CMSIS SERCOM peripheral.
// -----------------------------------------------------------------------------
static inline void countFrameError(Sercom *hw, volatile uint32_t &counter)
{
  if (hw->USART.INTFLAG.bit.ERROR && hw->USART.STATUS.bit.FERR) {
    counter++;
  }
}


// -----------------------------------------------------------------------------
// Reset helpers
// -----------------------------------------------------------------------------
void resetAllFrameErrorCounts()
{
  noInterrupts();
//  uart1_frame_errors = 0;
  SerialA4_frame_errors = 0;
  SerialD12_frame_errors = 0;
  SerialSPI_frame_errors = 0;
  interrupts();
}

void resetFrameErrorCount(volatile uint32_t &counter)
{
  noInterrupts();
  counter = 0;
  interrupts();
}

// Optional read helper
uint32_t getFrameErrorCount(volatile uint32_t &counter)
{
  noInterrupts();
  uint32_t value = counter;
  interrupts();
  return value;
}


// -----------------------------------------------------------------------------
// SERCOM IRQ handlers
// Count frame error first, then hand over to the normal Arduino UART handler.
// -----------------------------------------------------------------------------
void SERCOM0_0_Handler() { countFrameError(SERCOM0, SerialA4_frame_errors); SerialA4.IrqHandler(); }
void SERCOM0_1_Handler() { countFrameError(SERCOM0, SerialA4_frame_errors); SerialA4.IrqHandler(); }
void SERCOM0_2_Handler() { countFrameError(SERCOM0, SerialA4_frame_errors); SerialA4.IrqHandler(); }
void SERCOM0_3_Handler() { countFrameError(SERCOM0, SerialA4_frame_errors); SerialA4.IrqHandler(); }

void SERCOM1_0_Handler() { countFrameError(SERCOM1, SerialSPI_frame_errors); SerialSPI.IrqHandler(); }
void SERCOM1_1_Handler() { countFrameError(SERCOM1, SerialSPI_frame_errors); SerialSPI.IrqHandler(); }
void SERCOM1_2_Handler() { countFrameError(SERCOM1, SerialSPI_frame_errors); SerialSPI.IrqHandler(); }
void SERCOM1_3_Handler() { countFrameError(SERCOM1, SerialSPI_frame_errors); SerialSPI.IrqHandler(); }

void SERCOM5_0_Handler() { countFrameError(SERCOM5, SerialD12_frame_errors); SerialD12.IrqHandler(); }
void SERCOM5_1_Handler() { countFrameError(SERCOM5, SerialD12_frame_errors); SerialD12.IrqHandler(); }
void SERCOM5_2_Handler() { countFrameError(SERCOM5, SerialD12_frame_errors); SerialD12.IrqHandler(); }
void SERCOM5_3_Handler() { countFrameError(SERCOM5, SerialD12_frame_errors); SerialD12.IrqHandler(); }

//// Because I'm using the build in Serial1, I can't override the handler.  To get it to work for
// SERCOM3, I think I would need to edit the variant file.  Getting frame errors on 3 of the 
// receivers is probably enough for my research needs (although annoying).
//void SERCOM3_0_Handler() { countFrameError(SERCOM3, uart1_frame_errors); Serial1.IrqHandler(); }
//void SERCOM3_1_Handler() { countFrameError(SERCOM3, uart1_frame_errors); Serial1.IrqHandler(); }
//void SERCOM3_2_Handler() { countFrameError(SERCOM3, uart1_frame_errors); Serial1.IrqHandler(); }
//void SERCOM3_3_Handler() { countFrameError(SERCOM3, uart1_frame_errors); Serial1.IrqHandler(); }


// -----------------------------------------------------------------------------
// 58 kHz clock on D7 using TCC1
// D7 = PA18 = TCC1/WO[2]
// -----------------------------------------------------------------------------
static void setup58kHz()
{
  // D7 = PA18 = TCC1/WO[2]
  pinPeripheral(IRCLK_58K_PIN, PIO_TIMER_ALT);

  // Enable bus clock for TCC1
  MCLK->APBBMASK.reg |= MCLK_APBBMASK_TCC1;

  // Feed TCC1 from GCLK0 (120 MHz)
  GCLK->PCHCTRL[TCC1_GCLK_ID].reg =
    GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0;

  while ((GCLK->PCHCTRL[TCC1_GCLK_ID].reg & GCLK_PCHCTRL_CHEN) == 0) {
  }

  // Disable before reconfiguring
  TCC1->CTRLA.bit.ENABLE = 0;
  while (TCC1->SYNCBUSY.bit.ENABLE) {
  }

  // Software reset
  TCC1->CTRLA.bit.SWRST = 1;
  while (TCC1->SYNCBUSY.bit.SWRST || TCC1->CTRLA.bit.SWRST) {
  }

  // Normal PWM mode
  TCC1->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
  while (TCC1->SYNCBUSY.bit.WAVE) {
  }

  // 120 MHz / (2067 + 1) = ~58.03 kHz
  TCC1->PER.reg = 2067;
  while (TCC1->SYNCBUSY.bit.PER) {
  }

  // WO[2] uses CC[2]
  TCC1->CC[2].reg = 1034;   // ~50% duty
  while (TCC1->SYNCBUSY.bit.CC2) {
  }

  TCC1->CTRLA.reg =
    TCC_CTRLA_PRESCALER_DIV1 |
    TCC_CTRLA_PRESCSYNC_PRESC |
    TCC_CTRLA_ENABLE;

  while (TCC1->SYNCBUSY.bit.ENABLE) {
  }
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

void setup() {
  Serial.begin(115200);
  while (!Serial) {
  }

  // UART1: built-in Serial1 on D1/D0
  Serial1.begin(9600);

  // UART2: A4 TX / A1 RX -> SERCOM0
  pinPeripheral(PIN_A4, PIO_SERCOM_ALT);
  pinPeripheral(PIN_A1, PIO_SERCOM_ALT);
  SerialA4.begin(9600);

  // UART3: D12 TX / D13 RX -> SERCOM5
  pinPeripheral(PIN_PA23, PIO_SERCOM_ALT);
  pinPeripheral(PIN_PA22, PIO_SERCOM_ALT);
  SerialD12.begin(9600);

  // UART4: MOSI TX / SCK RX -> SERCOM1 (SCK is 24, MOSI is 25)
  pinPeripheral(PIN_SPI_MOSI, PIO_SERCOM_ALT);
  pinPeripheral(PIN_SPI_SCK,  PIO_SERCOM_ALT);
  SerialSPI.begin(9600);

  // Invert TX on all 4 UARTs
  // This will mean idle is low (0v), and so the
  // logical AND with clk will result in IR LEDs
  // being off.  This is important, we don't want
  // to saturate the environment with an idle IR
  // clock signal
  configureSercomInvert(SERCOM3, true, false); // Serial1 on SERCOM3
  configureSercomInvert(SERCOM0, true, false); // SerialA4
  configureSercomInvert(SERCOM5, true, false); // SerialD12
  configureSercomInvert(SERCOM1, true, false); // SerialSPI

  
  resetAllFrameErrorCounts();

  // I2C: default Wire on SDA/SCL -> SERCOM2
  Wire.begin();
  Wire.setClock(400000);

  Wire.onReceive( i2c_receive );
  Wire.onRequest( i2c_request );

  // 58 kHz output on D4
  setup58kHz();

  // Analog inputs
  pinMode( LDRA_IN_PIN, INPUT );
  pinMode( LDRB_IN_PIN, INPUT );
  pinMode( LDRC_IN_PIN, INPUT );
  pinMode( PROXA_IN_PIN, INPUT );
  pinMode( PROXB_IN_PIN, INPUT );
  

  Serial.println("Setup complete");
}

void loop()
{
  if (Serial.available()) {
    int c = Serial.read();
    Serial1.write(c);
    SerialA4.write(c);
    SerialD12.write(c);
    SerialSPI.write(c);
  }

  if (SerialA4.available()) {
    Serial.write(SerialA4.read());
  }
}

void i2c_receive( int len ) {

}

void i2c_request() {

}
