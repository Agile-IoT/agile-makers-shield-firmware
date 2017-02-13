 
/***********************************************************
 *               ATMega Serial-to-I2C                     *
 *                                                        *
 *   Description: Program to interact with two UARTS      *
 *      and store the RX in a buffer. The buffer can      *
 *      be accessed via I2C to open or close the          *
 *      serial port and to read or write.                 *
 *   Author: David Palomares                              *
 *   Version: 0.2                                         *
 *   Date: February 2017                                  *
 **********************************************************/


/* --- INCLUDES ----------------------------------------- */
#include <Wire.h>
/* ------------------------------------------------------ */


/* --- DEFINES ------------------------------------------ */
/*** ATMega ***/
#define SLAVE_ADDRESS 0x14
#define ATMEGA_CHECK_BYTE 0xDA
#define I2C_SIZE 256
#define CHUNK_SIZE 128
#define TX_BUFFER_SIZE 512
#define RX_BUFFER_SIZE 1024
#define GPS_BUFFER_SIZE 128 // (x <= CHUNK_SIZE) && (x <= 255)
#define PIN_ENABLE_UART_0 22
#define PIN_ENABLE_UART_1 23
#define PIN_BUTTON_0 70
#define PIN_INT_0 6
#define PIN_BUTTON_1 71
#define PIN_INT_1 7
#define PIN_ISR 3

/*** Functions ***/
#define MASK_SOCKET(x) ((x >> 4) & 0x0F)
#define MASK_ADDRESS(x) (x & 0x0F)
#define MASK_FULL(x, y) (((x & 0x0F) << 4) | (y & 0x0F))

/*** Sockets ***/
#define SOCKET_0 0
#define SOCKET_1 1
#define SOCKET_GPS 2
#define UART_0 Serial3
#define UART_EVENT_0 serialEvent3
#define UART_1 Serial1
#define UART_EVENT_1 serialEvent1
#define UART_GPS Serial2

/*** I2C Addresses ***/
#define ATMEGA_CHECK 0xFF
#define FIFO_TX 0x00
#define FIFO_RX 0x01
#define FIFO_AVAILABLE_HIGH 0x02 // Available is an uint16_t
#define FIFO_AVAILABLE_LOW 0x03
#define FIFO_TO_READ_HIGH 0x04 // To read is an uint16_t
#define FIFO_TO_READ_LOW 0x05
#define SOCKET_BAUDRATE_3 0x06 // Baudrate is an uint32_t
#define SOCKET_BAUDRATE_2 0x07
#define SOCKET_BAUDRATE_1 0x08
#define SOCKET_BAUDRATE_0 0x09
#define SOCKET_DATABITS 0x0A
#define SOCKET_STOPBITS 0x0B
#define SOCKET_PARITY 0x0C
#define SOCKET_STATUS 0x0D
#define INT_UART 0x0E
#define INT_BUTTON 0x0F
#define GPS_UPDATE 0x00
#define GPS_READ_BUFFER_SIZE 0x01
#define GPS_READ_GGA 0x02
#define GPS_READ_RMC 0x03

/*** Other definitions ***/
// Type of the data sent to the I2C master
#define FAIL 0
#define SUCCESS 1
#define BYTE 2
#define WORD 3
#define DWORD 4
#define ARRAY 5
// UART status
#define MODE_OFF 0
#define MODE_ON 1
// OTHER
#define DELAY_10MS 10

/*** UART config ***/
#define UART_DATABITS_5 0x00
#define UART_DATABITS_6 0x02
#define UART_DATABITS_7 0x04
#define UART_DATABITS_8 0x06
#define UART_STOPBITS_1 0x00
#define UART_STOPBITS_2 0x08
#define UART_PARITY_NONE 0x00
#define UART_PARITY_EVEN 0x20
#define UART_PARITY_ODD 0x30

/*** GPS ***/
#define GPS_HEADER_SIZE 6
#define GPS_TYPE_NONE B00000000
#define GPS_TYPE_GGA B00000001
#define GPS_TYPE_RMC B00000010
/* ------------------------------------------------------ */


/* --- VARIABLES ---------------------------------------- */
/*** I2C ***/
volatile uint8_t i2cMem[I2C_SIZE];
uint8_t i2cPointer = 0x00;
uint8_t returnType = BYTE;

/*** UART Buffers ***/
// Buffer 0
uint8_t txBuffer0[TX_BUFFER_SIZE];
volatile uint8_t rxBuffer0[RX_BUFFER_SIZE];
volatile uint16_t stackPointer0 = 0 ;
volatile uint16_t readPointer0 = 0;
// Buffer 1
uint8_t txBuffer1[TX_BUFFER_SIZE];
volatile uint8_t rxBuffer1[RX_BUFFER_SIZE];
volatile uint16_t stackPointer1 = 0 ;
volatile uint16_t readPointer1 = 0;
// Buffer GPS
uint8_t gpsBufferGGA[GPS_BUFFER_SIZE];
uint8_t gpsBufferRMC[GPS_BUFFER_SIZE];

/*** UART Defaults ***/
uint8_t defBaudrate3 = 0x00; // 0x00002580 = 9600
uint8_t defBaudrate2 = 0x00;
uint8_t defBaudrate1 = 0x25;
uint8_t defBaudrate0 = 0x80;
uint8_t defDatabits = UART_DATABITS_8;
uint8_t defStopbits = UART_STOPBITS_1;
uint8_t defParity = UART_PARITY_NONE;

/*** GPS ***/
boolean gpsRequest = false;
const PROGMEM char GPS_set_baudrate[] = "$PMTK251,115200*1F\r\n"; // Baudrate = 115200
const PROGMEM char GPS_set_NMEA_sentences[] = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"; // GGCA and RMC
const PROGMEM char GPS_set_update_time[] = "$PMTK220,1000*1F\r\n"; // One second between updates
const char GPS_HEADER_GGA[] = "$GPGGA";
const char GPS_HEADER_RMC[] = "$GPRMC";
/* ------------------------------------------------------ */


extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;

uint16_t getFreeSram() {
  uint8_t newVariable;
  // heap is empty, use bss as start memory address
  if ((uint16_t)__brkval == 0)
    return (((uint16_t)&newVariable) - ((uint16_t)&__bss_end));
  // use heap end as the start of the memory address
  else
    return (((uint16_t)&newVariable) - ((uint16_t)__brkval));
};

/* --- FUNCTIONS ---------------------------------------- */

/*
 * Function: setup 
 * Initializes the ATMega
 */
void setup () {

   // Initialize Buffer 0
   memset(txBuffer0, 0x00, TX_BUFFER_SIZE);
   memset(rxBuffer0, 0x00, RX_BUFFER_SIZE);

   // Initialize Buffer 1
   memset(txBuffer1, 0x00, TX_BUFFER_SIZE);
   memset(rxBuffer1, 0x00, RX_BUFFER_SIZE);

   // Initialize GPS Buffers
   memset(gpsBufferGGA, 0x00, GPS_BUFFER_SIZE);
   memset(gpsBufferRMC, 0x00, GPS_BUFFER_SIZE);
   
   // Initialize the I2C memory
   memset(i2cMem, 0x00, I2C_SIZE);
   i2cMem[ATMEGA_CHECK] = ATMEGA_CHECK_BYTE;
   i2cMem[MASK_FULL(SOCKET_0, SOCKET_BAUDRATE_3)] = defBaudrate3;
   i2cMem[MASK_FULL(SOCKET_0, SOCKET_BAUDRATE_2)] = defBaudrate2;
   i2cMem[MASK_FULL(SOCKET_0, SOCKET_BAUDRATE_1)] = defBaudrate1;
   i2cMem[MASK_FULL(SOCKET_0, SOCKET_BAUDRATE_0)] = defBaudrate0;
   i2cMem[MASK_FULL(SOCKET_1, SOCKET_BAUDRATE_3)] = defBaudrate3;
   i2cMem[MASK_FULL(SOCKET_1, SOCKET_BAUDRATE_2)] = defBaudrate2;
   i2cMem[MASK_FULL(SOCKET_1, SOCKET_BAUDRATE_1)] = defBaudrate1;
   i2cMem[MASK_FULL(SOCKET_1, SOCKET_BAUDRATE_0)] = defBaudrate0;
   i2cMem[MASK_FULL(SOCKET_0, SOCKET_DATABITS)] = defDatabits;
   i2cMem[MASK_FULL(SOCKET_1, SOCKET_DATABITS)] = defDatabits;
   i2cMem[MASK_FULL(SOCKET_0, SOCKET_STOPBITS)] = defStopbits;
   i2cMem[MASK_FULL(SOCKET_1, SOCKET_STOPBITS)] = defStopbits;
   i2cMem[MASK_FULL(SOCKET_0, SOCKET_PARITY)] = defParity;
   i2cMem[MASK_FULL(SOCKET_1, SOCKET_PARITY)] = defParity;
   i2cMem[MASK_FULL(SOCKET_0, SOCKET_STATUS)] = MODE_OFF;
   i2cMem[MASK_FULL(SOCKET_1, SOCKET_STATUS)] = MODE_OFF;
   i2cMem[MASK_FULL(SOCKET_GPS, GPS_READ_BUFFER_SIZE)] = GPS_BUFFER_SIZE;
   
   // Initialize I2C as slave
   Wire.begin(SLAVE_ADDRESS);

   // Define callbacks for I2C communication
   Wire.onReceive(receiveData);
   Wire.onRequest(sendData);

   // Initialize pins
   pinMode(PIN_ENABLE_UART_0, OUTPUT);
   pinMode(PIN_ENABLE_UART_0, OUTPUT);
   pinMode(PIN_BUTTON_0, INPUT);
   attachInterrupt(PIN_INT_0, buttonEvent0, RISING);
   pinMode(PIN_BUTTON_1, INPUT);
   attachInterrupt(PIN_INT_1, buttonEvent1, FALLING);
   pinMode(PIN_ISR, OUTPUT);
   digitalWrite(PIN_ISR, LOW);
   
   // Initialize GPS
   UART_GPS.begin(9600);
   delay(DELAY_10MS);
   for (int i = 0; i < strlen_P(GPS_set_baudrate); i++) {
      UART_GPS.write(pgm_read_byte_near(GPS_set_baudrate + i));
   }
   delay(DELAY_10MS);
   UART_GPS.end();
   delay(DELAY_10MS);
   UART_GPS.begin(115200);
   delay(DELAY_10MS);
   for (int i = 0; i < strlen_P(GPS_set_NMEA_sentences); i++) {
      UART_GPS.write(pgm_read_byte_near(GPS_set_NMEA_sentences + i));
   }
   delay(DELAY_10MS);
   for (int i = 0; i < strlen_P(GPS_set_update_time); i++) {
      UART_GPS.write(pgm_read_byte_near(GPS_set_update_time + i));
   }

   //Serial.begin(9600); //TODO: Delete this   
   
}


/*
 * Function: loop
 * Iterates infinitely
 */
void loop () {

   if (gpsRequest == true) {
      updateGPS();
      gpsRequest = false;
   }

}


/*
 * Function: receiveData
 * Callback for received data
 * int byteCount: number of bytes received
 */
void receiveData (int byteCount) {

   uint8_t rxData = 0x00;
   uint8_t rxDataWriteLow = 0x00;
   uint8_t rxDataWriteHigh = 0x00;  
   uint8_t socket = 0x00;

   // Read the first data (the command of the request)
   rxData = Wire.read();
   socket = MASK_SOCKET(rxData);

   if (rxData = ATMEGA_CHECK) {
      //Type of request
      if (byteCount == 1) {
         // Read request
         returnType = BYTE;
         i2cPointer = rxData;
      } else {
         // Write request
         returnType = FAIL;
      }
   } else if (socket == SOCKET_GPS) {
      //Type of request
      if (byteCount == 1) {
         // Read request
         switch (MASK_ADDRESS(rxData)) {
            case GPS_READ_BUFFER_SIZE:
               returnType = BYTE;
               i2cPointer = rxData;
               break;
            case GPS_READ_GGA:
            case GPS_READ_RMC:
               returnType = ARRAY;
               i2cPointer = rxData;
               break;
            default:
               returnType = BYTE;
               i2cPointer = rxData;
               break;  
         }
      } else {
         // Write request
         returnType = FAIL;
         switch (MASK_ADDRESS(rxData)) {
            case GPS_UPDATE:
               gpsRequest = true;
               returnType = SUCCESS;
               break;
            default:
               break;
         }
      }
   } else {
      // Type of request
      if (byteCount == 1) {
         // Read request
         switch (MASK_ADDRESS(rxData)) {
            case FIFO_RX:    
               returnType = ARRAY;
               i2cPointer = rxData;
               break;
            case SOCKET_BAUDRATE_3:
               returnType = DWORD;
               i2cPointer = rxData;
               break;
            case FIFO_AVAILABLE_HIGH:
               // Copy FIFO_AVAILABLE to FIFO_TO_READ in case 
               // it changes while requesting a read
               i2cMem[MASK_FULL(socket, FIFO_TO_READ_HIGH)] = 
                     i2cMem[MASK_FULL(socket, FIFO_AVAILABLE_HIGH)];
               i2cMem[MASK_FULL(socket, FIFO_TO_READ_LOW)] = 
                     i2cMem[MASK_FULL(socket, FIFO_AVAILABLE_LOW)];
               returnType = WORD;
               i2cPointer = MASK_FULL(socket, FIFO_TO_READ_HIGH);
               break;
            case INT_BUTTON:
            case INT_UART:
               if (i2cMem[rxData] == 0x01) {
                  // Clean the interruption
                  i2cMem[rxData] = 0x00;
                  returnType = SUCCESS;
               } else {
                  returnType = FAIL;
               }
               // Clean the pin
               if ((i2cMem[MASK_FULL(SOCKET_0, INT_UART)] == 0x00) &&
                      (i2cMem[MASK_FULL(SOCKET_1, INT_UART)] == 0x00) &&
                      (i2cMem[MASK_FULL(SOCKET_0, INT_BUTTON)] == 0x00) &&
                      (i2cMem[MASK_FULL(SOCKET_1, INT_BUTTON)] == 0x00)) {
                  digitalWrite(PIN_ISR, LOW);
               }
               break;
            default:
               returnType = BYTE;
               i2cPointer = rxData;
               break;
         }   
      } else {
         // Write request
         returnType = FAIL;
         switch (MASK_ADDRESS(rxData)) {
            case FIFO_TX:
               sendToUART(socket);
               returnType = SUCCESS;
               break;
            case SOCKET_STATUS:
               if (Wire.available()) {
                  rxDataWriteLow = Wire.read();
                  updateUART(socket, rxDataWriteLow);
                  returnType = SUCCESS;
               }
               break;
            case SOCKET_BAUDRATE_3:
               if (Wire.available() > 3) {
                  for (int i = 0; i < 4; i++) {
                     rxDataWriteLow = Wire.read();
                     i2cMem[rxData + i] = rxDataWriteLow;
                     returnType = SUCCESS;
                  }
               }     
               break;
            case SOCKET_DATABITS:
               if (Wire.available()) {
                  rxDataWriteLow = Wire.read();
                  if (rxDataWriteLow == UART_DATABITS_5 || 
                      rxDataWriteLow == UART_DATABITS_6 ||
                      rxDataWriteLow == UART_DATABITS_7 ||
                      rxDataWriteLow == UART_DATABITS_8) {
                     i2cMem[rxData] = rxDataWriteLow;   
                     returnType = SUCCESS;
                  }
               }
               break;
            case SOCKET_STOPBITS:
               if (Wire.available()) {
                  rxDataWriteLow = Wire.read();
                  if (rxDataWriteLow == UART_STOPBITS_1 || 
                      rxDataWriteLow == UART_STOPBITS_2) {
                     i2cMem[rxData] = rxDataWriteLow;  
                     returnType = SUCCESS; 
                  }
               }
               break;
            case SOCKET_PARITY:
               if (Wire.available()) {
                  rxDataWriteLow = Wire.read();
                  if (rxDataWriteLow == UART_PARITY_NONE || 
                      rxDataWriteLow == UART_PARITY_EVEN ||
                      rxDataWriteLow == UART_PARITY_ODD) {
                     i2cMem[rxData] = rxDataWriteLow;   
                     returnType = SUCCESS;
                  }
               }
               break;
            default:
               break;
         }
      } 
   }
        
   // Clean any data left
   while (Wire.available()) {
      rxData = Wire.read();
   }

}


/*
 * Function: sendData
 * Callback for sending data
 */
void sendData () {

   uint8_t socket = MASK_SOCKET(i2cPointer);
   uint16_t availData = 0;
   uint16_t dataWritten = 0;

   switch (returnType) {
      case ARRAY:
         // In case of array, return all available data up to CHUNK_SIZE
         if (socket == SOCKET_0) {
            availData = 
                  (i2cMem[MASK_FULL(SOCKET_0, FIFO_TO_READ_HIGH)] << 8) |
                  i2cMem[MASK_FULL(SOCKET_0, FIFO_TO_READ_LOW)];
            for (int i = readPointer0; (i < (readPointer0 + CHUNK_SIZE)) && (i < (readPointer0 + availData)); i++) {
               Wire.write(rxBuffer0[(i % RX_BUFFER_SIZE)]);
               dataWritten++;
            }
            readPointer0 = (readPointer0 + dataWritten) % RX_BUFFER_SIZE;
            // Update the available data
            if (stackPointer0 >= readPointer0) {
               availData = stackPointer0 - readPointer0;   
            } else {
               availData = stackPointer0 + RX_BUFFER_SIZE - readPointer0;
            }
            i2cMem[MASK_FULL(SOCKET_0, FIFO_AVAILABLE_HIGH)] = availData >> 8;
            i2cMem[MASK_FULL(SOCKET_0, FIFO_AVAILABLE_LOW)] = availData & 0xFF;
            i2cMem[MASK_FULL(SOCKET_0, FIFO_TO_READ_HIGH)] = availData >> 8;
            i2cMem[MASK_FULL(SOCKET_0, FIFO_TO_READ_LOW)] = availData & 0xFF;
            if (availData > 0 ) {
               i2cMem[MASK_FULL(SOCKET_0, INT_UART)] = 0x01;
               digitalWrite(PIN_ISR, HIGH);
            }
         }
         if (socket == SOCKET_1) {
            availData = 
                  (i2cMem[MASK_FULL(SOCKET_1, FIFO_TO_READ_HIGH)] << 8) |
                  i2cMem[MASK_FULL(SOCKET_1, FIFO_TO_READ_LOW)];
            for (int i = readPointer1; (i < (readPointer1 + CHUNK_SIZE)) && (i < (readPointer1 + availData)); i++) {
               Wire.write(rxBuffer1[(i % RX_BUFFER_SIZE)]);
               dataWritten++;
            }
            readPointer1 = (readPointer1 + dataWritten) % RX_BUFFER_SIZE;
            // Update the available data
            if (stackPointer1 >= readPointer1) {
               availData = stackPointer1 - readPointer1;   
            } else {
               availData = stackPointer1 + RX_BUFFER_SIZE - readPointer1;
            }
            i2cMem[MASK_FULL(SOCKET_1, FIFO_AVAILABLE_HIGH)] = availData >> 8;
            i2cMem[MASK_FULL(SOCKET_1, FIFO_AVAILABLE_LOW)] = availData & 0xFF;
            i2cMem[MASK_FULL(SOCKET_1, FIFO_TO_READ_HIGH)] = availData >> 8;
            i2cMem[MASK_FULL(SOCKET_1, FIFO_TO_READ_LOW)] = availData & 0xFF;
            if (availData > 0 ) {
               i2cMem[MASK_FULL(SOCKET_1, INT_UART)] = 0x01;
               digitalWrite(PIN_ISR, HIGH);
            }
         }
         if (socket == SOCKET_GPS) {
            if (MASK_ADDRESS(i2cPointer) == GPS_READ_GGA) {
               for (int i = 0; (i < GPS_BUFFER_SIZE) && (i < CHUNK_SIZE); i++) {
                  Wire.write(gpsBufferGGA[i]);
               }
            }
            if (MASK_ADDRESS(i2cPointer) == GPS_READ_RMC) {
               for (int i = 0; (i < GPS_BUFFER_SIZE) && (i < CHUNK_SIZE); i++) {
                  Wire.write(gpsBufferRMC[i]);
               }
            }
         }
         break;
      case DWORD:
         for (int i = 0; i < 4; i++) {
            if ((i2cPointer + i) <= I2C_SIZE) {
               Wire.write(i2cMem[i2cPointer + i]);
            } else {
               Wire.write(0x00);
            }  
         }
         break;
      case WORD:
         Wire.write(i2cMem[i2cPointer]);
         if ((i2cPointer + 1) <= I2C_SIZE) {
            Wire.write(i2cMem[i2cPointer + 1]);
         } else {
            Wire.write(0x00);
         }
         break;
      case BYTE:
         Wire.write(i2cMem[i2cPointer]);
         break;
      case SUCCESS:
         Wire.write(0x01);
         break;
      case FAIL:
      default:
         Wire.write(0x00);
         break;
   }
   
}

/*
 * Function: updateUART()
 * Turns ON or OFF the specified socket
 * uint8_t socket: the socket
 * uint8_t mode: ON or OFF
 */
void updateUART (uint8_t socket, uint8_t mode) {

   uint32_t baud3 = i2cMem[MASK_FULL(socket, SOCKET_BAUDRATE_3)];
   uint32_t baud2 = i2cMem[MASK_FULL(socket, SOCKET_BAUDRATE_2)];
   uint32_t baud1 = i2cMem[MASK_FULL(socket, SOCKET_BAUDRATE_1)];
   uint32_t baud0 = i2cMem[MASK_FULL(socket, SOCKET_BAUDRATE_0)];
   uint32_t baudrate = (baud3 << 32) | (baud2 << 16) | (baud1 << 8) | baud0;

   uint8_t databits = i2cMem[MASK_FULL(socket, SOCKET_DATABITS)];
   uint8_t stopbits = i2cMem[MASK_FULL(socket, SOCKET_STOPBITS)];
   uint8_t parity = i2cMem[MASK_FULL(socket, SOCKET_PARITY)];
   uint8_t config = databits | stopbits | parity; 
  
   if (socket == SOCKET_0) {
       // Close the SOCKET_0
       UART_0.end();
       digitalWrite(PIN_ENABLE_UART_0, LOW);
       // Clean interrupts
       i2cMem[MASK_FULL(socket, INT_UART)] = 0x00;
       if ((i2cMem[MASK_FULL(SOCKET_0, INT_UART)] == 0x00) &&
             (i2cMem[MASK_FULL(SOCKET_1, INT_UART)] == 0x00) &&
             (i2cMem[MASK_FULL(SOCKET_0, INT_BUTTON)] == 0x00) &&
             (i2cMem[MASK_FULL(SOCKET_1, INT_BUTTON)] == 0x00)) {
          digitalWrite(PIN_ISR, LOW);
       }
       // Initialize TX Buffer 0
       memset(txBuffer0, 0x00, TX_BUFFER_SIZE);
       // Initialize RX Buffer 0
       memset(rxBuffer0, 0x00, RX_BUFFER_SIZE);
       stackPointer0 = 0;
       readPointer0 = 0;
       // Reset the available data
       i2cMem[MASK_FULL(socket, FIFO_AVAILABLE_HIGH)] = 0x00;
       i2cMem[MASK_FULL(socket, FIFO_AVAILABLE_LOW)] = 0x00;
       i2cMem[MASK_FULL(socket, FIFO_TO_READ_HIGH)] = 0x00;
       i2cMem[MASK_FULL(socket, FIFO_TO_READ_LOW)] = 0x00;
      if (mode == MODE_ON) {
         // Open the SOCKET_0
         digitalWrite(PIN_ENABLE_UART_0, HIGH);
         UART_0.begin(baudrate, config);
         i2cMem[MASK_FULL(socket, SOCKET_STATUS)] = MODE_ON;
      }
   }
   
   if (socket == SOCKET_1) {
      // Close the SOCKET_1
      UART_1.end();
      digitalWrite(PIN_ENABLE_UART_1, LOW);
      // Clean interrupts
      i2cMem[MASK_FULL(socket, INT_UART)] = 0x00;
      if ((i2cMem[MASK_FULL(SOCKET_0, INT_UART)] == 0x00) &&
            (i2cMem[MASK_FULL(SOCKET_1, INT_UART)] == 0x00) &&
            (i2cMem[MASK_FULL(SOCKET_0, INT_BUTTON)] == 0x00) &&
            (i2cMem[MASK_FULL(SOCKET_1, INT_BUTTON)] == 0x00)) {
         digitalWrite(PIN_ISR, LOW);
      }
      // Initialize TX Buffer 1
      memset(txBuffer1, 0x00, TX_BUFFER_SIZE);
      // Initialize RX Buffer 1
      memset(rxBuffer1, 0x00, RX_BUFFER_SIZE);
      stackPointer1 = 0;
      readPointer1 = 0;
      // Reset the available data
      i2cMem[MASK_FULL(socket, FIFO_AVAILABLE_HIGH)] = 0x00;
      i2cMem[MASK_FULL(socket, FIFO_AVAILABLE_LOW)] = 0x00;
      i2cMem[MASK_FULL(socket, FIFO_TO_READ_HIGH)] = 0x00;
      i2cMem[MASK_FULL(socket, FIFO_TO_READ_LOW)] = 0x00;
      if (mode == MODE_ON) {
         // Open the SOCKET_1
         digitalWrite(PIN_ENABLE_UART_1, HIGH);
         UART_1.begin(baudrate, config);
         i2cMem[MASK_FULL(socket, SOCKET_STATUS)] = MODE_ON;
      }
   }
   
}

/*
 * Function UART_EVENT_0()
 * Callback when UART_0 data is available
 */
void UART_EVENT_0 () {

   uint8_t data;
   uint16_t availData;

   Wire.end(); // Pause I2C interrupts

      while (UART_0.available() > 0) {
         data = UART_0.read();
         // Check if buffer is full
         if (((stackPointer0 + 1 ) % RX_BUFFER_SIZE) != (readPointer0 % RX_BUFFER_SIZE)) {
            // Store the data in the buffer
            rxBuffer0[stackPointer0] = data;
            stackPointer0 = (stackPointer0 + 1) % RX_BUFFER_SIZE;
         }
         // Guard to avoid multiple events for a single transmission
         if (UART_0.available() == 0) {
            delay(DELAY_10MS);
         }
      }
      // Update the available data
      if (stackPointer0 >= readPointer0) {
         availData = stackPointer0 - readPointer0;   
      } else {
         availData = stackPointer0 + RX_BUFFER_SIZE - readPointer0;
      }
      i2cMem[MASK_FULL(SOCKET_0, FIFO_AVAILABLE_HIGH)] = availData >> 8;
      i2cMem[MASK_FULL(SOCKET_0, FIFO_AVAILABLE_LOW)] = availData & 0xFF;
      // Update the interruption flag
      i2cMem[MASK_FULL(SOCKET_0, INT_UART)] = 0x01;
      digitalWrite(PIN_ISR, HIGH); //TODO: Maybe -> write(HIGH); delay(x); write(LOW);
   
   Wire.begin(SLAVE_ADDRESS); // Restart I2C interrupts
    
}

/*
 * Function UART_EVENT_1()
 * Callback when UART_1 data is available
 */
void UART_EVENT_1 () {
  
   uint8_t data;
   uint16_t availData;

   Wire.end(); // Pause I2C interrupts
   
      while (UART_1.available() > 0) {
         data = UART_1.read();
         // Check if buffer is full
         if (((stackPointer1 + 1 ) % RX_BUFFER_SIZE) != (readPointer1 % RX_BUFFER_SIZE)) {
            // Store the data in the buffer
            rxBuffer1[stackPointer1] = data;
            stackPointer1 = (stackPointer1 + 1) % RX_BUFFER_SIZE;
         }
         // Guard to avoid multiple events for a single transmission
         if (UART_1.available() == 0) {
            delay(DELAY_10MS);
         }
      }
      // Update the available data
      if (stackPointer1 >= readPointer1) {
         availData = stackPointer1 - readPointer1;   
      } else {
         availData = stackPointer1 + RX_BUFFER_SIZE - readPointer1;
      }
      i2cMem[MASK_FULL(SOCKET_1, FIFO_AVAILABLE_HIGH)] = availData >> 8;
      i2cMem[MASK_FULL(SOCKET_1, FIFO_AVAILABLE_LOW)] = availData & 0xFF;
      // Update the interruption flag
      i2cMem[MASK_FULL(SOCKET_1, INT_UART)] = 0x01;
      digitalWrite(PIN_ISR, HIGH);
      
   Wire.begin(SLAVE_ADDRESS); // Restart I2C interrupts
    
}

/*
 * Function: sendToUART()
 * Sends the data available in Wire to the specified socket
 */
void sendToUART (uint8_t socket) {

   uint8_t data;
   uint16_t len = 0;
  
   if (socket == SOCKET_0) {
      while (Wire.available()) {
         data = Wire.read();
         if (len < TX_BUFFER_SIZE) {
            txBuffer0[len] = data;
            len++;
         }
      }
      UART_0.write(txBuffer0, len);
      UART_0.flush();
   }

   if (socket == SOCKET_1) {
      while (Wire.available()) {
         data = Wire.read();
         if (len < TX_BUFFER_SIZE) {
            txBuffer1[len] = data;
            len++;
         }
      } 
      UART_1.write(txBuffer1, len);
      UART_1.flush();
   }
   
}

/*
 * Function: buttonEvent0
 * Handles the event of pushing button 0
 */
void buttonEvent0 () {

   i2cMem[MASK_FULL(SOCKET_0, INT_BUTTON)] = 0x01;
   digitalWrite(PIN_ISR, HIGH);
   
}

/*
 * Function: buttonEvent1
 * Handles the event of pushing button 1
 */
void buttonEvent1 () {

   i2cMem[MASK_FULL(SOCKET_1, INT_BUTTON)] = 0x01;
   digitalWrite(PIN_ISR, HIGH);

}

/*
 *  Function: getGPS
 *  Gets the last data from the GPS and stores it in a buffer
 */
void updateGPS() {

   uint8_t data;
   uint16_t pointer = 0;
   char frameStart[GPS_HEADER_SIZE+1];
   uint8_t frameType = GPS_TYPE_NONE;
   uint8_t frameComplete = GPS_TYPE_NONE;

   Wire.end(); // Pause I2C interrupts

   // Clean the GPS buffers
   memset(gpsBufferGGA, 0x00, GPS_BUFFER_SIZE);
   memset(gpsBufferRMC, 0x00, GPS_BUFFER_SIZE);
   memset(frameStart, 0x00, GPS_HEADER_SIZE);

   // Flush all the previous data from the buffer
   while (UART_GPS.available() > 0) {
      UART_GPS.read();
   }

   // Wait till new data
   while (UART_GPS.available() <= 0) {
      delay(DELAY_10MS);
   }

   // Read the data
   while ((UART_GPS.available() > 0)) {// && (pointer < GPS_BUFFER_SIZE) && (frameComplete != (GPS_TYPE_GGA | GPS_TYPE_RMC))) {
      data = UART_GPS.read();
      // If the char is '$', '\r' or '\n', the frame restarts
      if ((data == '$') or (data == '\r') or (data == '\n')) {
         frameComplete = frameComplete | frameType;
         pointer = 0;
         memset(frameStart, 0x00, GPS_HEADER_SIZE);
         frameType = GPS_TYPE_NONE;
      }
      // Decide the action depending on the pointer
      if ((pointer > 0) and (pointer < GPS_HEADER_SIZE)) {
         // Waiting for type of frame
         frameStart[pointer] = data;
         pointer++;
      } else if (pointer == GPS_HEADER_SIZE) {
         frameStart[GPS_HEADER_SIZE] = '\0';
         // Check type of frame
         if (strcmp(frameStart, GPS_HEADER_GGA) == 0) {
            // If we have that type of frame already stored, skip
            if (frameComplete & GPS_TYPE_GGA) {
               pointer = 0;
               memset(frameStart, 0x00, GPS_HEADER_SIZE);
               frameType = GPS_TYPE_NONE;
            } else {
               strcpy(gpsBufferGGA, frameStart);
               frameType = GPS_TYPE_GGA;
               gpsBufferGGA[pointer] = data;
               pointer++;
            }
         } else if (strcmp(frameStart, GPS_HEADER_RMC) == 0) {
            // If we have type of frame already stored, skip
            if (frameComplete & GPS_TYPE_RMC) {
               pointer = 0;
               memset(frameStart, 0x00, GPS_HEADER_SIZE);
               frameType = GPS_TYPE_NONE;
            } else {
               strcpy(gpsBufferRMC, frameStart);
               frameType = GPS_TYPE_RMC;
               gpsBufferRMC[pointer] = data;
               pointer++;
            }
         } else {
            pointer = 0;
            memset(frameStart, 0x00, GPS_HEADER_SIZE);
            frameType = GPS_TYPE_NONE;
         }
      } else if (pointer > GPS_HEADER_SIZE) {
         // Fill the frame
         if (pointer < GPS_BUFFER_SIZE) {
            switch (frameType) {
               case GPS_TYPE_GGA:
                  gpsBufferGGA[pointer] = data;
                  pointer++;
                  break;
               case GPS_TYPE_RMC:
                  gpsBufferRMC[pointer] = data;
                  pointer++;
                  break;
               case GPS_TYPE_NONE:
               default:
                  pointer = 0;
                  memset(frameStart, 0x00, GPS_HEADER_SIZE);
                  frameType = GPS_TYPE_NONE;
                  break;
            }
         }
      } else {
         // Waiting for start of frame
         if (data == '$') {
            frameStart[pointer] = data;
            pointer++;
         }
      }
      // Guard to avoid multiple events for a single transmission
      if (UART_GPS.available() == 0) {
         delay(DELAY_10MS);
      }
   }

   Wire.begin(SLAVE_ADDRESS); // Restart I2C interrupts
   
}
/* ------------------------------------------------------ */


