#include <Arduino.h>
#include <wiring_private.h>

// RX first, TX second
Uart SerialA4A5(&sercom0, PIN_A5, PIN_A4, SERCOM_RX_PAD_2, UART_TX_PAD_0);

// UART4: MOSI TX / SCK RX -> SERCOM1
Uart SerialSPI(&sercom1, PIN_SPI_SCK, PIN_SPI_MOSI, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// UART3: D12 TX / D13 RX -> SERCOM5
Uart SerialD12(&sercom5, PIN_PA22, PIN_PA23, SERCOM_RX_PAD_1, UART_TX_PAD_0);


void SERCOM0_0_Handler() {
  SerialA4A5.IrqHandler();
}
void SERCOM0_1_Handler() {
  SerialA4A5.IrqHandler();
}
void SERCOM0_2_Handler() {
  SerialA4A5.IrqHandler();
}
void SERCOM0_3_Handler() {
  SerialA4A5.IrqHandler();
}


void SERCOM1_0_Handler() {
  //  countFrameError( SERCOM1, SerialSPI_frame_errors);
  SerialSPI.IrqHandler();
}
void SERCOM1_1_Handler() {
  //  countFrameError( SERCOM1, SerialSPI_frame_errors);
  SerialSPI.IrqHandler();
}
void SERCOM1_2_Handler() {
  //  countFrameError( SERCOM1, SerialSPI_frame_errors);
  SerialSPI.IrqHandler();
}
void SERCOM1_3_Handler() {
  //  countFrameError( SERCOM1, SerialSPI_frame_errors);
  SerialSPI.IrqHandler();
}



void SERCOM5_0_Handler() {
  //  countFrameError( SERCOM5, SerialD12_frame_errors);
  SerialD12.IrqHandler();
}
void SERCOM5_1_Handler() {
  //  countFrameError( SERCOM5, SerialD12_frame_errors);
  SerialD12.IrqHandler();
}
void SERCOM5_2_Handler() {
  //  countFrameError( SERCOM5, SerialD12_frame_errors);
  SerialD12.IrqHandler();
}
void SERCOM5_3_Handler() {
  //  countFrameError( SERCOM5, SerialD12_frame_errors);
  SerialD12.IrqHandler();
}

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
  delay(1000);
  Serial.println("Start");

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

  Serial.println("SERCOM0 ready");
  SerialA4A5.println("Hello from SERCOM0 A4/A5");
}

void loop() {
  //  while (Serial.available()) {
  //    SerialA4A5.write(Serial.read());
  //  }

  Serial.println("A4: ");
  while (SerialA4A5.available()) {
    Serial.print((char)SerialA4A5.read());
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

  SerialA4A5.print("A4 ");
  SerialA4A5.println(millis());


  SerialSPI.print("SPI ");
  SerialSPI.println(millis());


  SerialD12.print("D12 ");
  SerialD12.println(millis());

  Serial1.print("S1 ");
  Serial1.println(millis());


  delay(500);
}
