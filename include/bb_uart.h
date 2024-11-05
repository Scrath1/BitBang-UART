#ifndef BB_UART_H
#define BB_UART_H
#include "ring_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * How to use:
 * 1. Declare a ringbuffer to use by the bitbang uart using the
 *  RING_BUFFER_DEF macro.
 * 2. Declare a function of the format "void func(uint8_t)". This function
 *  should set the GPIO pin used for transmission by this UART high or low
 *  depending on parameter state.
 * 3. Declare a BB_UART_t struct and assign all variables that are not
 *  prefixed with a double underscore. The tx_ringBuf pointer should point
 *  at the ringbuffer from step one. The function declared in step 2
 *  should be assigned to the writePinFunc variable.
 * 4. Start a timer which calls the BB_UART_transmitBit function in the interval
 *  required by the desired baud rate. For 9600 baud this function should be called
 *  every 104.167us.
 * 5. Transmit using the put and putc functions.
 */

/**
 * Timer counter period value calculation
 * (1/(baud*oversampling)) * timer clock freq = counter reload val
 */

/**
 * ToDo:
 * - Add user callback functions
 *      - byte received
 *      - rx buffer full or half full
 *      - tx complete
 *          ...
 * Maybe define weak functions and pass the uart object to them for identification
 * 
 * - Rework ring buffer so that when the rx buffer is full the oldest value is
 *  overwritten rather than dropping the newest value
 */

// UART settings
// ============================================
typedef enum{
    BB_UART_PARITY_NONE = 0,
    BB_UART_PARITY_EVEN = 1,
    BB_UART_PARITY_ODD = 2
} BB_UART_Parity_t;

typedef enum{
    BB_UART_STOPBITS_1 = 1,
    BB_UART_STOPBITS_2 = 2,
} BB_UART_Stopbits_t;

typedef enum{
    BB_UART_WORDLENGTH_7 = 7,
    BB_UART_WORDLENGTH_8 = 8
} BB_UART_Wordlength_t;

typedef enum{
    // UART is only capable of sending data
    BB_UART_TX_ONLY,
    // UART is only capable of receiving data
    BB_UART_RX_ONLY,
    // UART can send and receive data over one wire, but not at the same time.
    // Requires an open-drain pinmode. Functionality of this mode
    // is unverified
    BB_UART_ONE_WIRE,
    // UART has separate RX and TX wires
    // and can send and receive simultaneously
    BB_UART_RX_TX,
} BB_UART_Mode_t;

typedef enum{
    // Waiting for data to transmit
    BB_UART_TX_IDLE,
    // Currently in the process of transmitting a uart frame
    BB_UART_TX_TRANSMITTING_FRAME,
    // All contents of tx buffer have been transmitted
    BB_UART_TX_BUFFER_TRANSMITTED,
    // Tx line is blocked, e.g. because UART is in one-wire mode and is
    // currently receiving data
    BB_UART_TX_BLOCKED
} BB_UART_Tx_Line_State_t;

typedef enum{
    // Waiting for detection of start bit
    BB_UART_RX_IDLE,
    // Currently in the process of receiving a uart frame
    BB_UART_RX_RECEIVING_FRAME,
    // a frame was fully received. UART is preparing for next frame reception
    BB_UART_RX_FRAME_RECEIVED,
    // Rx line is blocked, e.g. because UART is in one-wire mode and is
    // currently transmitting
    BB_UART_RX_BLOCKED
} BB_UART_Rx_Line_State_t;

typedef enum{
    BB_UART_OVERSAMPLE_1 = 1, // No oversampling at all. Only recommended for Tx only mode
    BB_UART_OVERSAMPLE_3 = 3, // 3 bit samples are taken per rx bit
    BB_UART_Oversample_5 = 5, // 5 bit samples are taken per rx bit
} BB_UART_Oversampling_t;

typedef struct{
    BB_UART_Wordlength_t wordLen;
    BB_UART_Parity_t parity;
    BB_UART_Stopbits_t stopBits;
    // Ring buffer for storing data to transmit
    ring_buffer_t* tx_ringBuf;
    // Ring buffer where received data is stored
    ring_buffer_t* rx_ringBuf;
    // Function used by this uart to transmit a bit on the tx line.
    void (*writePinFunc)(uint8_t val);
    // Function used by this uart to sample a bit on the rx line.
    uint8_t (*readPinFunc)(void);
    // Determines the operating mode of the uart
    BB_UART_Mode_t mode;
    // Determines how many samples are taken to determine the state of
    // one received bit. This directly impacts the required timer frequency
    // for the timerCallback. With 3 times oversampling the timer has to run
    // at 3 times the baud rate.
    BB_UART_Oversampling_t oversampling;
    // Contains library internal variables for transmitting data
    struct {
        // Current word being transmit, including start stop and parity bits.
        // The first bit to be transmit is the LSB
        // Bit order:
        // MSB                                  LSB
        // | stop bits | parity | data | start bit|
        uint16_t frame;
        // Remaining bits of current frame before next frame is created
        uint8_t remainingFrameBits;
        // Current state of Tx line
        BB_UART_Tx_Line_State_t state;
    } __tx_internal;
    // Contains library internal variables for receiving data
    struct{
        // Current word being received, including start, stop and parity bits.
        // The LSB is the last bit to be received, e.g. one of the stop bits
        // Bit order:
        // MSB                                   LSB
        // | start bit | data | parity | stop bits |
        uint16_t frame;
        // Contains the bits sampled to determine the next frame bit
        uint16_t bitSamples;
        // Counts how often the the service function has been called.
        // Maximum value is oversampling-1.
        // Whenever this variable is equal to 0, a new rx bit is determined based
        // on the rx bit samples and a tx bit can be transmitted. This variable
        // is incremented in the service function.
        uint8_t overSampleCounter;
        // Current state of Rx line
        BB_UART_Rx_Line_State_t state;
        // Counts the received start, data, parity and stopbits while receiving
        // a single frame. This value is reset when a start bit from a new
        // frame is detected.
        uint8_t receivedBitsCnt;
        // If in one wire mode, this variable represents the number of bit
        // cycles to wait after a full frame reception before reenabling the tx
        // line. If the uart is not in one-wire mode, this variable should always
        // be 0.
        uint8_t cooldownCycles;
    } __rx_internal;
} BB_UART_t;

/**
 * @brief Checks the configuration struct to ensure a valid configuration has been
 *  created and initializes internal variables
 * @param uartPtr [IN] pointer to uart struct
 * @return RC_SUCCESS on success
 * @return RC_ERROR_NULL if uartPtr is null
 * @return RC_ERROR_BAD_PARAM if the buffer or functions required depending on
 *  the mode are not set.
 */
RC_t BB_UART_validateConfig(BB_UART_t* uartPtr);

/**
 * @brief Function which has to be called with a frequency of
 *  the baudrate * oversampling setting for the UART to work.
 * @param uartPtr [IN] pointer to uart struct
 * @return RC_SUCCESS on success
 * @return RC_ERROR_NULL if uartPtr is NULL
 * @return RC_ERROR_INVALID_STATE if the UART mode is not a valid value
 * @return RC_ERROR_BUFFER_FULL if the Rx buffer is full
 */
RC_t BB_UART_service(BB_UART_t* uartPtr);

/**
 * @brief Adds data to tx_buffer and begins transmission.
 * @param uartPtr [IN] pointer to uart struct
 * @param data [IN] pointer to data which should be transmitted
 * @param dataLen [IN] number of bytes to transmit
 * @return RC_SUCCESS on success
 * @return RC_ERROR_NULL if uartPtr is NULL,
 * @return RC_ERROR_BUFFER_FULL if the data does not fit into the available
 *  space in the tx buffer. In that case, no data is added to transmission.
 */
RC_t BB_UART_put(BB_UART_t* uartPtr, const uint8_t* data, uint32_t dataLen);

/**
 * @brief Adds a single char to the transmit buffer
 * @param uartPtr [IN] pointer to uart struct
 * @param c [IN] char to transmit
 * @return RC_SUCCESS on success
 * @return RC_ERROR_NULL if uartPtr is NULL,
 * @return RC_ERROR_BUFFER_FULL if the data does not fit into the available
 *  space in the tx buffer. In that case, no data is added to transmission.
 */
RC_t BB_UART_putc(BB_UART_t* uartPtr, char c);

#ifdef __cplusplus
}
#endif
#endif // BB_UART_H