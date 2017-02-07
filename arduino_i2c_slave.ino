
/***********************************************************
 *               ATMega Serial-to-I2C                     *
 *                                                        *
 *   Description: Program to interact with two UARTS      *
 *      and store the RX in a buffer. The buffer can      *
 *      be accessed via I2C to open or close the          *
 *      serial port and to read or write.                 *
 *   Author: David Palomares                              *
 *   Version: 0.1                                         *
 *   Date: September 2016                                 *
 **********************************************************/


/* --- INCLUDES ----------------------------------------- */
#include <Wire.h>
/* ------------------------------------------------------ */


/* --- DEFINES ------------------------------------------ */
/*** ATMega ***/
#define SLAVE_ADDRESS 0x14
#define I2C_SIZE 256
#define TX_BUFFER_SIZE 512
#define RX_BUFFER_SIZE 1024
#define CHUNK_SIZE 128
#define PIN_BUTTON_0 70
#define PIN_BUTTON_1 71
#define PIN_INT 3

/*** Functions ***/
#define MASK_SOCKET(x) ((x >> 4) & 0x0F)
#define MASK_ADDRESS(x) (x & 0x0F)
#define MASK_FULL(x, y) (((x & 0x0F) << 4) | (y & 0x0F))

/*** Sockets ***/
#define SOCKET_0 0
#define SOCKET_1 1
#define UART_0 Serial
#define UART_EVENT_0 serialEvent
#define UART_1 Serial1
#define UART_EVENT_1 serialEvent1

/*** I2C Addresses ***/
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
/* ------------------------------------------------------ */


/* --- VARIABLES ---------------------------------------- */
/* I2C ***/
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

/*** UART Defaults ***/
uint8_t defBaudrate3 = 0x00; // 0x00002580 = 9600
uint8_t defBaudrate2 = 0x00;
uint8_t defBaudrate1 = 0x25;
uint8_t defBaudrate0 = 0x80;
uint8_t defDatabits = UART_DATABITS_8;
uint8_t defStopbits = UART_STOPBITS_1;
uint8_t defParity = UART_PARITY_NONE;
/* ------------------------------------------------------ */


/* --- FUNCTIONS ---------------------------------------- */

/*
 * Function: setup 
 * Initializes the ATMega
 */
void setup () {

   //TODO: Hardcoded
   pinMode(23, OUTPUT);
   digitalWrite(23, HIGH);
   pinMode(22, OUTPUT);
   digitalWrite(22, HIGH);

   pinMode(72, OUTPUT);
   digitalWrite(72, HIGH);

   // Initialize TX Buffer 0
   for (int i = 0; i < TX_BUFFER_SIZE; i++) {
      txBuffer0[i] = 0x00;
   }

   // Initialize RX Buffer 0
   for (int i = 0; i < RX_BUFFER_SIZE; i++) {
      rxBuffer0[i] = 0x00;
   }

   // Initialize TX Buffer 1
   for (int i = 0; i < TX_BUFFER_SIZE; i++) {
      txBuffer1[i] = 0x00;
   }

   // Initialize RX Buffer 1
   for (int i = 0; i < RX_BUFFER_SIZE; i++) {
      rxBuffer1[i] = 0x00;
   }
   
   // Initialize the I2C memory
   for (int i = 0; i < I2C_SIZE; i++) {
      i2cMem[i] = 0x00; 
   }
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
   
   // Initialize I2C as slave
   Wire.begin(SLAVE_ADDRESS);

   // Define callbacks for I2C communication
   Wire.onReceive(receiveData);
   Wire.onRequest(sendData);

   // Initialize pins
   pinMode(PIN_BUTTON_0, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_0), buttonEvent0, FALLING);
   pinMode(PIN_BUTTON_1, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_1), buttonEvent1, FALLING);
   pinMode(PIN_INT, OUTPUT);
   digitalWrite(PIN_INT, LOW);

   //Serial.begin(9600); //TODO: Delete this
   //Serial2.begin(9600);
   
}


/*
 * Function: loop
 * Iterates infinitely
 */
void loop () {

   // Do nothing
   //if (Serial2.available()) { //TODO: Delete this
   //   Serial.write(Serial2.read());
   //}

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
               digitalWrite(PIN_INT, LOW);
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
      // Clean any data left
      while (Wire.available()) {
         rxData = Wire.read();
      }
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
               digitalWrite(PIN_INT, HIGH);
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
       // Clean interrupts
       i2cMem[MASK_FULL(socket, INT_UART)] = 0x00;
       if ((i2cMem[MASK_FULL(SOCKET_0, INT_UART)] == 0x00) &&
             (i2cMem[MASK_FULL(SOCKET_1, INT_UART)] == 0x00) &&
             (i2cMem[MASK_FULL(SOCKET_0, INT_BUTTON)] == 0x00) &&
             (i2cMem[MASK_FULL(SOCKET_1, INT_BUTTON)] == 0x00)) {
          digitalWrite(PIN_INT, LOW);
       }
       // Initialize TX Buffer 0
       for (int i = 0; i < TX_BUFFER_SIZE; i++) {
          txBuffer0[i] = 0x00;
       }
       // Initialize RX Buffer 0
       for (int i = 0; i < RX_BUFFER_SIZE; i++) {
          rxBuffer0[i] = 0x00;
       }
       stackPointer0 = 0;
       readPointer0 = 0;
       // Reset the available data
       i2cMem[MASK_FULL(socket, FIFO_AVAILABLE_HIGH)] = 0x00;
       i2cMem[MASK_FULL(socket, FIFO_AVAILABLE_LOW)] = 0x00;
       i2cMem[MASK_FULL(socket, FIFO_TO_READ_HIGH)] = 0x00;
       i2cMem[MASK_FULL(socket, FIFO_TO_READ_LOW)] = 0x00;
      if (mode == MODE_ON) {
         // Open the SOCKET_0
         UART_0.begin(baudrate, config);
         i2cMem[MASK_FULL(socket, SOCKET_STATUS)] = MODE_ON;
      }
   }
   
   if (socket == SOCKET_1) {
      // Close the SOCKET_1
      UART_1.end();
      // Clean interrupts
      i2cMem[MASK_FULL(socket, INT_UART)] = 0x00;
      if ((i2cMem[MASK_FULL(SOCKET_0, INT_UART)] == 0x00) &&
            (i2cMem[MASK_FULL(SOCKET_1, INT_UART)] == 0x00) &&
            (i2cMem[MASK_FULL(SOCKET_0, INT_BUTTON)] == 0x00) &&
            (i2cMem[MASK_FULL(SOCKET_1, INT_BUTTON)] == 0x00)) {
         digitalWrite(PIN_INT, LOW);
      }
      // Initialize TX Buffer 1
      for (int i = 0; i < TX_BUFFER_SIZE; i++) {
         txBuffer1[i] = 0x00;
      }
      // Initialize RX Buffer 1
      for (int i = 0; i < RX_BUFFER_SIZE; i++) {
         rxBuffer1[i] = 0x00;
      }
      stackPointer1 = 0;
      readPointer1 = 0;
      // Reset the available data
      i2cMem[MASK_FULL(socket, FIFO_AVAILABLE_HIGH)] = 0x00;
      i2cMem[MASK_FULL(socket, FIFO_AVAILABLE_LOW)] = 0x00;
      i2cMem[MASK_FULL(socket, FIFO_TO_READ_HIGH)] = 0x00;
      i2cMem[MASK_FULL(socket, FIFO_TO_READ_LOW)] = 0x00;
      if (mode == MODE_ON) {
         // Open the SOCKET_1
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
            delay(1);
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
      digitalWrite(PIN_INT, HIGH); //TODO: Maybe -> write(HIGH); delay(x); write(LOW);
   
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
            delay(1);
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
      digitalWrite(PIN_INT, HIGH);
      
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
   digitalWrite(PIN_INT, HIGH);
   
}

/*
 * Function: buttonEvent1
 * Handles the event of pushing button 1
 */
void buttonEvent1 () {

   i2cMem[MASK_FULL(SOCKET_1, INT_BUTTON)] = 0x01;
   digitalWrite(PIN_INT, HIGH);

}
/* ------------------------------------------------------ */


