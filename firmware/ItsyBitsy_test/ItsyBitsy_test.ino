#include <Arduino.h>
#include <wiring_private.h>
#include <Wire.h>

//#include "sam.h" // for SAMD51 registers

#include "ir_parser.h"
#include "ircomm_i2c.h"

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
Uart SerialA4(&sercom0, PIN_A5, PIN_A4, SERCOM_RX_PAD_2, UART_TX_PAD_0);

// UART3: D12 TX / D13 RX -> SERCOM5
Uart SerialD12(&sercom5, 13, 12, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// UART4: MOSI TX / SCK RX -> SERCOM1
Uart SerialSPI(&sercom1, PIN_SPI_SCK, PIN_SPI_MOSI, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// Match up 4 instances of the ir parsrer.
IRParser_c parser[4] = { Serial1, SerialA4, SerialD12, SerialSPI };


// -----------------------------------------------------------------------------
// Frame error counters per UART
// -----------------------------------------------------------------------------
volatile uint32_t Serial1_frame_errors = 0;
volatile uint32_t SerialA4_frame_errors = 0;
volatile uint32_t SerialD12_frame_errors = 0;
volatile uint32_t SerialSPI_frame_errors = 0;


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
  uart_channel[0].frame_errors = 0;
  uart_channel[1].frame_errors = 0;
  uart_channel[2].frame_errors = 0;
  uart_channel[3].frame_errors = 0;

  interrupts();
}

void resetFrameErrorCount( UartChannel& ch )
{
  noInterrupts();
  *(ch.frame_errors) = 0;
  interrupts();
}

// Optional read helper
uint32_t getFrameErrorCount( UartChannel& ch )
{
  noInterrupts();
  uint32_t value = *(ch.frame_errors);
  interrupts();
  return value;
}


// -----------------------------------------------------------------------------
// SERCOM IRQ handlers
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

typedef struct {
  ir_crc_t          crc;
  ir_activity_t     activity;
  ir_saturation_t   saturation;
  ir_errors_t       errors;
  ir_frame_errors_t frame_errors;
  ir_msg_timings_t  msg_timings;
  ir_byte_timings_t  byte_timings;
  ir_vectors_t      vectors;
  ir_bearing_t      bearing;
  ir_sensors_t      sensors;
} ircomm_metrics_t;
ircomm_metrics_t metrics;

// A record of byte activity per
// receiver which is periodically
// reset to 0.  Allows for the
// estimation of bearing to other
// transmitting boards/robots.
float bearing_activity[4];
unsigned long tx_ts;     // periodic transmit
unsigned long led_ts;    // LED time stamp


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
typedef struct { // 31 bytes
  ir_tx_params_t  tx[4];          // 11 bytes
  ir_rx_params_t  rx[4];          // 20 bytes
} ircomm_config_t;
ircomm_config_t config;

volatile byte tx_buf[4][MAX_TX_BUF];

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

/* I've had a lot of trouble trying to get SERCOM0
   working using A4/A5, or any other pins.  Doing
   low level register config rather than calling
   Serial.begin() seems to work. I am guessing the
   built in function is mucking up the MUX.
*/

static void beginSerialA4A5_manual(uint32_t baud)
{
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


void setup() {
  Serial.begin(115200);
  //  while (!Serial) {
  //  }

  Serial.println("Reset");


  // UART1: built-in Serial1 on D1/D0
  Serial1.begin(9600);


  // UART4: MOSI TX / SCK RX -> SERCOM1 (SCK is 24, MOSI is 25)
  pinPeripheral(PIN_SPI_MOSI, PIO_SERCOM_ALT);
  pinPeripheral(PIN_SPI_SCK,  PIO_SERCOM_ALT);
  SerialSPI.begin(9600);

  // UART3: D12 TX / D13 RX -> SERCOM5
  pinPeripheral(12, PIO_SERCOM_ALT);
  pinPeripheral(13, PIO_SERCOM_ALT);
  SerialD12.begin(9600);

  beginSerialA4A5_manual(9600);
  //
  //  // Invert TX on all 4 UARTs
  //  // This will mean idle is low (0v), and so the
  //  // logical AND with clk will result in IR LEDs
  //  // being off.  This is important, we don't want
  //  // to saturate the environment with an idle IR
  //  // clock signal
  //  //                              tx    rx
//  configureSercomInvert(SERCOM3, false, false); // Serial1 on SERCOM3
//  configureSercomInvert(SERCOM0, false, false); // SerialA4
//  configureSercomInvert(SERCOM5, false, false); // SerialD12
//  configureSercomInvert(SERCOM1, false, false); // SerialSPI
  //
  //
  //  resetAllFrameErrorCounts();
  //
  //  // I2C: default Wire on SDA/SCL -> SERCOM2
  //  Wire.begin();
  //  Wire.setClock(400000);
  //
  //  Wire.onReceive( i2c_receive );
  //  Wire.onRequest( i2c_request );
  //
  //  // 58 kHz output on D4
  //  setup58kHz();
  //
  //  // Analog inputs
  //  pinMode( LDRA_IN_PIN, INPUT );
  //  pinMode( LDRB_IN_PIN, INPUT );
  //  pinMode( LDRC_IN_PIN, INPUT );
  //  pinMode( PROXA_IN_PIN, INPUT );
  //  pinMode( PROXB_IN_PIN, INPUT );
  //


  Serial.println("Setup complete");
}


void loop() {
  //  while (Serial.available()) {
  //    SerialA4A5.write(Serial.read());
  //  }

  Serial.println("A4: ");
  while (SerialA4.available()) {
    Serial.print((char)SerialA4.read());
  }

  Serial.println("\nSPI: ");
  while ( SerialSPI.available() ) {
    Serial.print( (char)SerialSPI.read() );
  }

  Serial.println("\nD12: ");
  while ( SerialD12.available() ) {
    Serial.print( (char)SerialD12.read() );
  }

  Serial.println("\nS1: ");
  while ( Serial1.available() ) {
    Serial.print( (char)Serial1.read() );
  }

  Serial.println("\n******\n");

  SerialA4.print("A4 ");
  SerialA4.println(millis());


  SerialSPI.print("SPI ");
  SerialSPI.println(millis());


  SerialD12.print("D12 ");
  SerialD12.println(millis());

  Serial1.print("S1 ");
  Serial1.println(millis());


  delay(500);
}


void update() {


  // Check each uart interface
  for ( int i = 0; i < 4; i++ ) {

    // Non-blocking check to see if this channel is
    // still in the process of writing out UART data
    if ( uart_channel[i].tx_rx_state == TxRxState::TX ) {

      // check if we have finished transmitting
      if ( isUartTxComplete( uart_channel[i] ) ) {

        // enable Rx on channel.
        resetUART( i );

        parser[i].reset();
        // Set channel flag so on the next iteration
        // we'll go back to receiving
        uart_channel[i].tx_rx_state = TxRxState::RX;

        // Given that we have just enabled the UART,
        // it is probably fine to skip the parsing
        // below and move to the next channel in
        // the for loop
      }

    } else {

      // Assume that there has been no byte
      // activity and we will not transmit.
      // We'll use these in a drop-through
      // logic block to determine action, I
      // think that's more readable than a
      // fsm.
      bool activity  = false;
      bool transmit  = false;

      int status = parser[i].getNextByte(  );

      if ( i != 0 ) {
        metrics.frame_errors.rx[ i ] += getFrameErrorCount( uart_channel[i] );
        resetFrameErrorCount( uart_channel[i] );
      }

      // Use the value of status to update metrics and/or
      // to transfer a message into the i2c variables.
      // status  < 0 : error type defined in ir_parser.h
      // status == 0 : no bytes were received
      // status == 1 : 1 byte was received, not a full message
      // status  > 1 : a full message was received correctly
      if ( status < 0 ) { // error when receiving

        metrics.crc.fail[i]++;

        // Flip status to index error type
        status *= -1;

        // only status 4 is a no byte error
        if ( status < 4 ) activity = true;

        // Status code is 1:4, error array needs 0:3
        status -= 1;

        metrics.errors.type[ i ][ status ]++;

      } else if ( status == 1 ) { // got a byte, but not a message

        activity = true;


      } else if ( status > 1 ) { // successfully decoded a message

        activity = true;

        updateMsgTimestamp( i );

        // TODO: this is where we would set a flag to
        // transmit on successful receipt of a message
        // if that was set in config?
      }


      // If there was any byte activity, update the
      // byte level timestamps
      if ( activity ) {
        updateByteTimestamp(i);
        bearing_activity[i] += 1;
        metrics.activity.rx[i]++;
      }

      // Update time between receiving messages and
      // receiving bytes (dt)
      updateMsgElapsedTime(i);
      updateByteElapsedTime(i);


      // If there was no activity on this receiver.
      if ( !activity ) {
        // This is an attempt at a fix to a problem I haven't
        // fully diagnosed or solved yet.  It seems like the
        // AGC in the receiver can saturate or lower the gain
        // to 0, causing no bytes to be received.  If we find
        // that there is no activity for a long time, setting
        // desaturate = 1 will mean we attempt to power cycle
        // the receiver.
        //      if ( config.rx.flags.bits.desaturate == 1 ) {
        if ( isRxDesaturate(i) ) { // if we have a saturation timeout

          if ( metrics.byte_timings.dt_us[i] > (uint32_t)config.rx[i].saturation_us ) {

            toggleRxPower( i );

            // Advance this byte time stamp so
            // that we don't immediately trigger
            // again.
            metrics.byte_timings.ts_us[i] = micros();

            // Add to our count of saturation
            // occurences.
            metrics.saturation.rx[i]++;
          }
        }

      }

      // overrun == true: The board is configured to finish receiving
      // once the start byte has been received
      if ( config.rx[i].flags.bits.overrun && parser[i].rx_state != RX_WAIT_START ) {

        // Prevent transmission
        transmit = false;

      } else if ( isTxPeriodic(i) ) {

        if ( millis() - tx_ts > config.tx[i].interval_ms ) {
          transmit = true;
        }
      }


      // defer == true: recent byte activity will mean
      // that the transmission is deferred (cancelled)
      if ( config.tx[i].flags.bits.defer ) {

        // Was the last byte activity within the time
        // expected?
        if ( micros() - metrics.byte_timings.ts_us[ i ] < US_PER_BYTE_58KHZ * config.tx[i].defer_multi ) {
          transmit = false;
        }
      }



      if ( transmit ) {

        // Interupts any rx in process
        // If doTransmit() returns false?? TODO finish comment
        if ( doTransmit(i) ) {


          // Update timestamp for the next
          // transmission occurence
          //        setTxPeriod(); // TODO

          // If we did a transmission, our
          // saturation/inactivty timeout may
          // trigger because of the time
          // spent transmitting. So we
          // update all the byte timestamps
          // here.
          advanceByteTimestamps();
        }

      }

    }

  }

}

boolean doTransmit( int which ) {

  // Don't do anything if there is no message
  // to transmit.
  if ( config.tx[which].len == 0 || config.tx[which].repeat == 0 ) {

    // Schedule next transmission
    // Redundant if set to INTERLEAVED
    //        setTxPeriod(); // TODO
    return false;

  } else {



    // Stop receiving
    disableSercomRx( uart_channel[which].hw );

    // Set channel flag to indicate Tx is now in progress
    uart_channel[which].tx_rx_state = TxRxState::TX;

    // Transmission might be improved with some message
    // pre-amble that allows the receiving UART to
    // determine the clock of the source.
    // Add some preamble bytes. 0x55 = 0b01010101
    for ( uint8_t i = 0; i < config.tx[which].preamble_repeat; i++ ) {
      uart_channel[which].port->write( TX_PREAMBLE_BYTE );
    }

    // Using NeoSerial.print transmits over
    // IR.  NeoSerial TX is modulated with
    // the 38Khz or 58khz carrier in hardware.
    //unsigned long start_t = micros();
    for ( uint32_t i = 0; i < config.tx[which].repeat; i++ ) {

      // Checking HardwareNeoSerial.cpp, .write() is a blocking
      // function.  Therefore we don't need .flush()
      //NeoSerial.availableForWrite();
      for ( int j = 0; j < config.tx[which].len; j++ ) {
        uart_channel[which].port->write( tx_buf[which][j] );
      }

      //NeoSerial.print(tx_buf);
      uart_channel[which].port->flush();  // wait for send to complete
    }

    // Since we used disableRx(), we need to
    // re-enable the UART and so clear
    // the rx flags and rx_buf

    //    resetUART(which);

    // Schedule next transmission
    // Redundant if tx_mode INTERLEAVED
    //        setTxPeriod(); // todo
    return true;

  }

  return false;

}

// TODO: we need to think about whether each UART now
// has independent transmission timings.
bool isTxPeriodic(int which) {
  if ( config.tx[which].interval_ms > 0 ) return true;
  return false;
}

static void disableSercomRx(Sercom* hw) {
  // Optional: wait for any current receive operation to settle
  //while (hw->USART.SYNCBUSY.bit.CTRLB) {
  //}

  // Disable receiver
  hw->USART.CTRLB.bit.RXEN = 0;

  // Wait for synchronization
  while (hw->USART.SYNCBUSY.bit.CTRLB) {
  }
}

static void enableSercomRx(Sercom* hw) {
  // Enable receiver
  hw->USART.CTRLB.bit.RXEN = 1;

  // Wait for synchronization
  while (hw->USART.SYNCBUSY.bit.CTRLB) {
  }


}

static void clearSercomRxBuffer(Sercom* hw) {
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

static inline bool isUartTxComplete( const UartChannel& ch ) {
  // Check software buffer empty AND hardware shift register empty
  return (ch.port->availableForWrite() == SERIAL_BUFFER_SIZE) &&
         (ch.hw->USART.INTFLAG.bit.TXC);
}

static void disableSercomTx(Sercom* hw)
{
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

static void enableSercomTx(Sercom* hw)
{
  // Enable transmitter
  hw->USART.CTRLB.bit.TXEN = 1;

  // Wait for synchronization
  while (hw->USART.SYNCBUSY.bit.CTRLB) {
  }
}


void resetUART(int which) {

  disableSercomRx( uart_channel[which].hw );
  clearSercomRxBuffer( uart_channel[which].hw );
  delayMicroseconds(10);
  enableSercomRx(uart_channel[which].hw);


}

void toggleRxPower(int which) {

  int pin;

  switch ( which ) {
    case 0: pin = DEMOD1_EN_PIN; break;
    case 1: pin = DEMOD2_EN_PIN; break;
    case 2: pin = DEMOD3_EN_PIN; break;
    case 3: pin = DEMOD4_EN_PIN; break;
    default: return;
  }

  // See application note for timing here.
  digitalWrite( pin, LOW );
  delayMicroseconds( 1200 );
  digitalWrite( pin, HIGH );

  //  resetUART();

}

bool isRxDesaturate( int which ) {
  if ( config.rx[which].saturation_us > 0 ) return true;
  return false;
}

// Sometimes a blocking process will have
// prevented our timestamps progressing.
// For messages, we leave this alone as we
// consider blocking processes and their
// effect on the messaging performance a
// matter of interest.
// But for byte activity, we are using this
// to detect receiver saturation.
void advanceByteTimestamps() {
  for ( int i = 0; i < 4; i++ ) metrics.byte_timings.ts_us[i] = micros();
}

// Note: in milliseconds.  At 58khz, it
// takes about 1.2ms to receive 1 byte,
// and the minimum message is 5 bytes
void updateMsgTimestamp( int which ) {
  metrics.msg_timings.ts_ms[ which ] = millis();
}
void updateMsgElapsedTime( int which ) {
  unsigned long dt = millis() - metrics.msg_timings.ts_ms[ which ];
  metrics.msg_timings.dt_ms[ which ] = dt;
}
void advanceMsgTimestamps() {
  for ( int i = 0; i < 4; i++ ) {
    metrics.msg_timings.ts_ms[ i ] = millis();
  }
}

// Note: in microseconds. It is possible
// to receive a byte in 1.2ms, so millis
// isn't quite precise enough.
void updateByteTimestamp( int which ) {
  metrics.byte_timings.ts_us[ which ] = micros();
}

void updateByteElapsedTime( int which ) {
  unsigned long dt = micros() - metrics.byte_timings.ts_us[ which ];
  metrics.byte_timings.dt_us[ which ] = dt;
}


void i2c_receive( int len ) {

}

void i2c_request() {

}
