#include "bb_uart.h"
#include <stddef.h>

#define START_BIT_LEVEL (0)
#define STOP_BIT_LEVEL (1)
// Since the idle state of a UART line is high, the bitsample reset value
// should be all 1s
#define RX_BITSAMPLES_RESET_VALUE (0xFF)

/**
 * @brief Calculates the number of bits in a single UART frame based on
 *  the current configuration. This includes the start bit, data bits,
 *  parity bit and stop bits
 * @param uartPtr [IN] pointer to uart struct
 * @return number of bits in single UART frame or 0 if uartPtr is NULL
 */
uint8_t BB_UART_calculateFrameSize(const BB_UART_t* const uartPtr){
    if(uartPtr == NULL) return 0;
    uint8_t cnt = 1; // start with 1 to include the start bit
    cnt += (uint8_t)(uartPtr->wordLen);
    if(uartPtr->parity != BB_UART_PARITY_NONE) cnt++;
    cnt += (uint8_t)(uartPtr->stopBits);
    return cnt;
}

/**
 * @brief Creates the next word to transmit over serial
 * based on contents of tx_buffer and the wordlength, parity and
 * stopbit configuration of the given UART struct.
 * @return RC_ERROR_NULL if uartPtr or tx_ringBuf is NULL,
 * @return RC_ERROR_BUSY if the current frame is not fully transmitted,
 * @return RC_ERROR_BUFFER_EMPTY if there is no data to transmit
 */
RC_t BB_UART_createNextFrame(BB_UART_t* uartPtr){
    if(uartPtr == NULL) return RC_ERROR_NULL;
    if(uartPtr->__tx_internal.remainingFrameBits > 0) return RC_ERROR_BUSY;
    if(uartPtr->tx_ringBuf == NULL) return RC_ERROR_NULL;
    // Get byte to transmit
    uint8_t byteToTransmit;
    RC_t err = ring_buffer_get(uartPtr->tx_ringBuf, &byteToTransmit);
    if(err != RC_SUCCESS) return err;

    uint16_t* fPtr = &(uartPtr->__tx_internal.frame);
    *fPtr = 0;

    // add stop bits
    for(uint8_t i = 0; i < uartPtr->stopBits; i++){
        *fPtr = ((*fPtr) << 1) | STOP_BIT_LEVEL;
    }

    // add parity bits
    switch(uartPtr->parity){
        // __builtin_parity() returns 1 if number has odd parity and 0 for even
        case BB_UART_PARITY_EVEN:
            *fPtr = ((*fPtr) << 1) | __builtin_parity(byteToTransmit);
            break;
        case BB_UART_PARITY_ODD:
            *fPtr = ((*fPtr) << 1) | (!__builtin_parity(byteToTransmit));
            break;
        default:
            // no parity bit to add
            break;
    };

    // add data bits. LSB last as it will be the first bit to be transmitted
    const uint32_t wordLen = uartPtr->wordLen;
    for(uint32_t i = 0; i < wordLen; i++){
        uint8_t b = ((byteToTransmit >> (wordLen-1-i)) & 0b1);
        *fPtr = ((*fPtr) << 1) | b;
    }

    // add start bit
    *fPtr = *fPtr << 1 | START_BIT_LEVEL;
    // Update remaining frame bits
    uartPtr->__tx_internal.remainingFrameBits = BB_UART_calculateFrameSize(uartPtr);
    return RC_SUCCESS;
}

/**
 * @brief Function to call during callback of timer to transmit the next bit.
 * May be overwritten depending on hardware.
 * @return RC_ERROR_NULL if uartPtr is NULL,
 *  RC_ERROR_BUFFER_EMPTY if transmission of current frame is finished
 */
RC_t BB_UART_transmitBit(BB_UART_t* uartPtr){
    if(uartPtr == NULL) return RC_ERROR_NULL;
    uint8_t* remFrameBitPtr = &(uartPtr->__tx_internal.remainingFrameBits);
    switch(uartPtr->__tx_internal.state){
        case BB_UART_TX_IDLE:
            // returns RC_ERROR_BUFFER_EMPTY if there is no data to transmit
            if(RC_SUCCESS == BB_UART_createNextFrame(uartPtr)){
                uartPtr->__tx_internal.state = BB_UART_TX_TRANSMITTING_FRAME;
                // if in one wire-mode, block rx line while transmitting
                if(uartPtr->mode == BB_UART_ONE_WIRE){
                    uartPtr->__rx_internal.state = BB_UART_RX_BLOCKED;
                    BB_UART_rxBlockedHook(uartPtr);
                }
            }
            break;
        case BB_UART_TX_TRANSMITTING_FRAME:
            if(*remFrameBitPtr == BB_UART_calculateFrameSize(uartPtr)){
                // execute once at beginning of frame transmission
                BB_UART_txFrameStartedHook(uartPtr);
            }
            if((*remFrameBitPtr) > 0){
                // transmit next bit in frame and decrement remainingFrameBits
                uint8_t b = uartPtr->__tx_internal.frame & 0b1; // get next bit
                uartPtr->__tx_internal.frame >>= 1; // shift bit out of queue
                (*remFrameBitPtr)--;
                if(uartPtr->writePinFunc != NULL){
                    uartPtr->writePinFunc(b);
                }
            }
            if((*remFrameBitPtr) == 0){
                uartPtr->__tx_internal.state = BB_UART_TX_FRAME_TRANSMITTED;
            }
            break;
        case BB_UART_TX_FRAME_TRANSMITTED:
            // must be executed here, otherwise the stop bit is not fully
            // included in the hook
            BB_UART_txFrameCompleteHook(uartPtr);
            RC_t ret = BB_UART_createNextFrame(uartPtr);
            if(RC_ERROR_BUFFER_EMPTY == ret){
                // transmission finished, no more data in tx buffer
                uartPtr->__tx_internal.state = BB_UART_TX_BUFFER_TRANSMITTED;
            }
            else if (RC_SUCCESS == ret){
                // There was more data in the tx buffer.
                // Send next frame
                uartPtr->__tx_internal.state = BB_UART_TX_TRANSMITTING_FRAME;
            }
            break;
        case BB_UART_TX_BUFFER_TRANSMITTED:
            if(uartPtr->mode == BB_UART_ONE_WIRE){
                // if in one-wire mode, unblock the rx line
                uartPtr->__rx_internal.state = BB_UART_RX_IDLE;
                BB_UART_rxUnblockedHook(uartPtr);
            }
            // return to idle state
            uartPtr->__tx_internal.state = BB_UART_TX_IDLE;
            BB_UART_txTransmissionCompleteHook(uartPtr);
            break;
        case BB_UART_TX_BLOCKED:
            // do nothing
            break;
        default:
            // should never happen
            return RC_ERROR_INVALID_STATE;
    }
    return RC_SUCCESS;
}

RC_t BB_UART_validateConfig(BB_UART_t* uartPtr){
    if(uartPtr == NULL) return RC_ERROR_NULL;
    BB_UART_Mode_t mode = uartPtr->mode;
    // ensure tx write function and ringbuffer are available if the mode
    // requires it
    if(mode == BB_UART_TX_ONLY || mode == BB_UART_RX_TX){
        if(uartPtr->writePinFunc == NULL) return RC_ERROR_BAD_PARAM;
        if(uartPtr->tx_ringBuf == NULL) return RC_ERROR_BAD_PARAM;
    }
    if(mode == BB_UART_RX_ONLY || mode == BB_UART_RX_TX){
        if(uartPtr->readPinFunc == NULL) return RC_ERROR_BAD_PARAM;
        if(uartPtr->rx_ringBuf == NULL) return RC_ERROR_BAD_PARAM;
    }
    // init ringbuffers
    if(uartPtr->rx_ringBuf != NULL) ring_buffer_init(uartPtr->rx_ringBuf);
    if(uartPtr->tx_ringBuf != NULL) ring_buffer_init(uartPtr->tx_ringBuf);

    // initialize internal data
    uartPtr->__tx_internal.frame = 0;
    uartPtr->__tx_internal.remainingFrameBits = 0;
    uartPtr->__tx_internal.state = BB_UART_TX_IDLE;
    uartPtr->__rx_internal.frame = RX_BITSAMPLES_RESET_VALUE; // idle line is high
    uartPtr->__rx_internal.bitSamples = RX_BITSAMPLES_RESET_VALUE;
    uartPtr->__rx_internal.overSampleCounter = 0;
    uartPtr->__rx_internal.state = BB_UART_RX_IDLE;
    uartPtr->__rx_internal.receivedBitsCnt = 0;
    uartPtr->__rx_internal.cooldownCycles = 0;
    uartPtr->__rx_internal.frame = BB_UART_RX_ERROR_NONE;
    return RC_SUCCESS;
}

/**
 * @brief Counts the number of set bits in the variable,
 *  starting from the lsb. At most bitsToEval are evaluated.
 */
uint32_t numOfSetBits(uint32_t var, uint32_t bitsToEval){
    if(bitsToEval > (sizeof(var)*8)){
        bitsToEval = sizeof(var)*8;
    }
    uint8_t setBits = 0;
    for(uint32_t i = 0; i < bitsToEval; i++){
        uint8_t b = (var >> i) & 0b1;
        if(b == 1) setBits++;
    }
    return setBits;
}

/**
 * @brief Validates a fully received UART frame and extracts
 *  the contained data if it is valid.
 * @param uartPtr [IN] pointer to uart struct
 * @param dataOut [OUT] Data contained in frame, if frame was valid
 * @return RC_SUCCESS if data frame was valid and data was extracted.
 * @return RC_ERROR_NULL if uartPtr or dataOut were nullptrs
 * @return RC_ERROR_INVALID_STATE if received bits don't match expected
 *  frame size. If frame is also invalid, RC_ERROR_INVALID is returned instead.
 * @return RC_ERROR_INVALID if the frame contains errors
 */
RC_t BB_UART_extractData(BB_UART_t* uartPtr, uint16_t* dataOut){
    if(uartPtr == NULL) return RC_ERROR_NULL;
    if(dataOut == NULL) return RC_ERROR_NULL;
    // reset error flag
    uartPtr->__rx_internal.error = BB_UART_RX_ERROR_NONE;
    RC_t ret = RC_SUCCESS;

    // ensure frame was fully received
    const uint32_t expectedFrameSize = BB_UART_calculateFrameSize(uartPtr);
    if(uartPtr->__rx_internal.receivedBitsCnt < expectedFrameSize){
        ret = RC_ERROR_INVALID_STATE;
        uartPtr->__rx_internal.error |= BB_UART_RX_ERROR_BITCOUNT;
    }

    uint16_t frame = uartPtr->__rx_internal.frame;
    // make sure all stop bits are high
    for(uint8_t i = 0; i < uartPtr->stopBits; i++){
        uint8_t sb = frame & 0b1;
        frame = frame >> 1;
        if(sb != STOP_BIT_LEVEL){
            ret = RC_ERROR_INVALID;
            uartPtr->__rx_internal.error |= BB_UART_RX_ERROR_STOPBITS;
        }
    }
    uint8_t pb = 0;
    // Extract parity bit (if available) and save it for later
    if(uartPtr->parity != BB_UART_PARITY_NONE){
        pb = frame & 0b1;
        frame = frame >> 1;
    }
    // extract data bits. Keep in mind they are currently reversed in the frame
    uint16_t data = 0;
    for(uint32_t i = 0; i < uartPtr->wordLen; i++){
        uint8_t b = frame & 0b1;
        frame = frame >> 1;
        // This reverses the bit order again so that it is now correct
        data = (data << 1) | b;
    }
    // verify that a start bit is also present
    if((frame & 0b1) != START_BIT_LEVEL){
        ret = RC_ERROR_INVALID;
        uartPtr->__rx_internal.error |= BB_UART_RX_ERROR_STARTBIT;
    }

    // evaluate parity if enabled
    if(uartPtr->parity != BB_UART_PARITY_NONE){
        // __builtin_parity() returns 1 if number has odd parity and 0 for even
        // temporarily add parity bit back to data for parity check
        uint8_t dataParity = __builtin_parity((data << 1) | pb);
        if(uartPtr->parity == BB_UART_PARITY_EVEN
            && dataParity == 1){
                ret = RC_ERROR_INVALID;
                uartPtr->__rx_internal.error |= BB_UART_RX_ERROR_PARITY;
            }
        else if (uartPtr->parity == BB_UART_PARITY_ODD
            && dataParity == 0){
                ret = RC_ERROR_INVALID;
                uartPtr->__rx_internal.error |= BB_UART_RX_ERROR_PARITY;
            }
    }

    // write back extracted data and return
    *dataOut = data;
    return ret;
}

void BB_UART_switchToRxAligningState(BB_UART_t* uartPtr){
    if(uartPtr == NULL) return;
    
    uartPtr->__rx_internal.overSampleCounter = 0;
    uartPtr->__rx_internal.state = BB_UART_RX_ALIGNING;
    if(uartPtr->mode == BB_UART_ONE_WIRE){
        // in one wire mode, block tx while receiving data
        uartPtr->__tx_internal.state = BB_UART_TX_BLOCKED;
        BB_UART_txBlockedHook(uartPtr);
    }
}

/**
 * @brief Samples UART line to detect transmitted bits
 * @param uartPtr [IN] pointer to uart struct
 * @return RC_SUCCESS on success
 * @return RC_ERROR_NULL if uartPtr is null, readPinFunc is not set
 *  or rx_ringBuf is not set
 * @return RC_ERROR_INVALID_STATE on frame errors
 * @return RC_ERROR_BUFFER_FULL if the rx buffer is full
 */
RC_t BB_UART_receiveBit(BB_UART_t* uartPtr){
    if(uartPtr == NULL) return RC_ERROR_NULL;
    if(uartPtr->rx_ringBuf == NULL) return RC_ERROR_NULL;
    if(uartPtr->readPinFunc == NULL) return RC_ERROR_NULL;

    // Sample Rx line
    // ------------------------
    uint16_t* bitSamples = &(uartPtr->__rx_internal.bitSamples);
    // absolute bit sample
    uint8_t b = uartPtr->readPinFunc();
    // shift bit sample into bitsamples
    *bitSamples = ((*bitSamples) << 1) | b;
    // average of the last couple of sampled bits
    uint8_t averagedBit = 0;
    uint8_t highSamples = numOfSetBits(*bitSamples, (uint32_t)uartPtr->oversampling);
    // more than half of the samples need to be high for the final
    // bit to be high
    const uint8_t thresh = (uartPtr->oversampling/2)+1;
    if(highSamples >= thresh) averagedBit = 1;
    else averagedBit = 0;

    // switch based on state
    uint16_t* framePtr = &(uartPtr->__rx_internal.frame);
    switch(uartPtr->__rx_internal.state){
        case BB_UART_RX_IDLE:
            if(b == START_BIT_LEVEL){
                // possible start of frame detected
                BB_UART_switchToRxAligningState(uartPtr);
            }
            break;
        case BB_UART_RX_ALIGNING:
            // wait until reset of oversamplecounter before transitioning to
            // actually receiving bits
            if(uartPtr->__rx_internal.overSampleCounter == 0){
                if(averagedBit != START_BIT_LEVEL){
                    // False start detected. Revert to idle
                    uartPtr->__rx_internal.state = BB_UART_RX_IDLE;
                    if(uartPtr->mode == BB_UART_ONE_WIRE){
                         uartPtr->__tx_internal.state = BB_UART_TX_IDLE;
                        BB_UART_txUnblockedHook(uartPtr);
                    }
                }
                else{
                    // true start detected. Transition to receiving data
                    uartPtr->__rx_internal.state = BB_UART_RX_RECEIVING_FRAME;
                    // Make sure framebuffer is empty and add startbit
                    *framePtr = averagedBit;
                    uartPtr->__rx_internal.receivedBitsCnt = 1;
                    BB_UART_rxFrameStartDetectedHook(uartPtr);
                }
            }
            break;
        case BB_UART_RX_RECEIVING_FRAME:
            // Only evaluate the sample every few cycles to allow for
            // oversampling
            if(uartPtr->__rx_internal.overSampleCounter != 0) break;
            *framePtr = ((*framePtr) << 1) | averagedBit;
            const uint32_t frameSize = BB_UART_calculateFrameSize(uartPtr);
            uartPtr->__rx_internal.receivedBitsCnt++;
            // if the number of received bits matches the frame size
            // return to idle state and listen for next start bit
            if(uartPtr->__rx_internal.receivedBitsCnt >= frameSize){
                uartPtr->__rx_internal.state = BB_UART_RX_FRAME_RECEIVED;
                // check if frame is valid and extract the data
                uint16_t data = 0;
                RC_t ret = RC_SUCCESS;
                if(RC_SUCCESS != BB_UART_extractData(uartPtr, &data)){
                    // frame was invalid
                    BB_UART_rxFrameErrorHook(uartPtr);
                    ret = RC_ERROR_INVALID;
                }
                else{
                    // frame was valid and data was extracted
                    // moving data to ringbuffer
                    if(RC_SUCCESS != ring_buffer_put(uartPtr->rx_ringBuf, (uint8_t) data)){
                        // shouldn't be possible unless ring_buffer API changed
                        // to have more than RC_SUCCESS as possible return here
                        ret = RC_ERROR_UNKNOWN;
                    }
                }
                if(uartPtr->mode == BB_UART_ONE_WIRE){
                    // in one wire mode, block tx for at least 2 bit cycles
                    // after receiving data to see if there is more being sent
                    uartPtr->__rx_internal.cooldownCycles = (uartPtr->oversampling * 2) + 1;
                }
                BB_UART_rxFrameCompleteHook(uartPtr);
                return ret;
            }
            break;
        case BB_UART_RX_FRAME_RECEIVED:
            if(b == START_BIT_LEVEL){
                // possible start to next frame detected
                BB_UART_switchToRxAligningState(uartPtr);
            }
            else if(uartPtr->__rx_internal.cooldownCycles == 0){
                // execute rx frame complete hook
                // revert to idle state after this one
                uartPtr->__rx_internal.state = BB_UART_RX_IDLE;
                uartPtr->__rx_internal.bitSamples = RX_BITSAMPLES_RESET_VALUE;
                uartPtr->__rx_internal.frame = RX_BITSAMPLES_RESET_VALUE;
                uartPtr->__rx_internal.receivedBitsCnt = 0;
                // in one-wire mode, unblock tx line again
                if(uartPtr->mode == BB_UART_ONE_WIRE){
                    uartPtr->__tx_internal.state = BB_UART_TX_IDLE;
                    BB_UART_txUnblockedHook(uartPtr);
                }
            }
            else{
                uartPtr->__rx_internal.cooldownCycles--;
            }
            break;
        default:
            return RC_ERROR_INVALID_STATE;
    }
    return RC_SUCCESS;
}

RC_t BB_UART_service(BB_UART_t* uartPtr){
    if(uartPtr == NULL) return RC_ERROR_NULL;
    // Shortcut variable
    uint8_t* overSampleCounter = &(uartPtr->__rx_internal.overSampleCounter);

    RC_t err = RC_SUCCESS;
    switch(uartPtr->mode){
        case BB_UART_TX_ONLY:
            // transmit the next bit in the tx frame
            (void)BB_UART_transmitBit(uartPtr);
            break;
        case BB_UART_RX_ONLY:
            // Sample the UART.
            err = BB_UART_receiveBit(uartPtr);
            break;
        case BB_UART_ONE_WIRE:
            // in one wire mode, rx can only occur if tx is idle or blocked
            // and the other way around. Whichever line changes first from
            // idle to transmit will set the other one to blocked.
            if(uartPtr->__tx_internal.state == BB_UART_TX_IDLE
                || uartPtr->__tx_internal.state == BB_UART_TX_BLOCKED){
                err = BB_UART_receiveBit(uartPtr);
            }
            // Determine whether the rx line is idle or blocked
            if(uartPtr->__rx_internal.state == BB_UART_RX_IDLE
                || uartPtr->__rx_internal.state == BB_UART_RX_BLOCKED){
                // determine whether it is time to transmit the next bit
                // depending on oversampling setting
                if(*overSampleCounter == 0){
                    // rx is idle and it is time to transmit a new bit
                    (void)BB_UART_transmitBit(uartPtr);
                }
            }
            break;
        case BB_UART_RX_TX:
            if(*overSampleCounter == 0){
                (void)BB_UART_transmitBit(uartPtr);
            }
            err = BB_UART_receiveBit(uartPtr);
            break;
        default:
            return RC_ERROR_INVALID_STATE;
    }

    // increment oversample counter and if required reset it to 0
    *overSampleCounter = (*overSampleCounter+1) % (uint8_t)(uartPtr->oversampling);
    return err;
}

RC_t BB_UART_put(BB_UART_t* uartPtr, const uint8_t* data, uint32_t wordLen){
    if(uartPtr == NULL) return RC_ERROR_NULL;
    uint32_t remainingSpace = uartPtr->tx_ringBuf->len - ring_buffer_avail(uartPtr->tx_ringBuf);
    if(remainingSpace < wordLen)
        return RC_ERROR_BUFFER_FULL;

    for(uint32_t i = 0; i < wordLen; i++){
        ring_buffer_put(uartPtr->tx_ringBuf, data[i]);
    }
    return RC_SUCCESS;
}

RC_t BB_UART_putc(BB_UART_t* uartPtr, char c){
    if(uartPtr == NULL) return RC_ERROR_NULL;
    if(ring_buffer_is_full(uartPtr->tx_ringBuf)) return RC_ERROR_BUFFER_FULL;

    ring_buffer_put(uartPtr->tx_ringBuf, (uint8_t)c);
    return RC_SUCCESS;
}

int32_t BB_UART_get(BB_UART_t* uartPtr, uint8_t* data, uint16_t len){
    if(uartPtr == NULL) return RC_ERROR_NULL;
    if(data == NULL) return RC_ERROR_NULL;

    uint32_t numBytesRead = 0;
    for(numBytesRead = 0; numBytesRead < len; numBytesRead++){
        uint8_t b = 0;
        if(RC_SUCCESS == ring_buffer_get(uartPtr->rx_ringBuf, &b)){
            data[numBytesRead] = b;
        }
        else{
            break;
        }
    }
    return numBytesRead;
}

int32_t BB_UART_getBlocking(BB_UART_t* uartPtr, uint8_t* data, uint16_t len){
    if(uartPtr == NULL) return RC_ERROR_NULL;
    if(data == NULL) return RC_ERROR_NULL;

    uint32_t numBytesRead = 0;
    while(numBytesRead < len){
        uint8_t b = 0;
        if(RC_SUCCESS == ring_buffer_get(uartPtr->rx_ringBuf, &b)){
            data[numBytesRead] = b;
            numBytesRead++;
        }
    }
    return numBytesRead;
}

__attribute__ ((weak)) void BB_UART_txFrameStartedHook(BB_UART_t* uartPtr){}
__attribute__ ((weak)) void BB_UART_txFrameCompleteHook(BB_UART_t* uartPtr){}
__attribute__ ((weak)) void BB_UART_txTransmissionCompleteHook(BB_UART_t* uartPtr){}
__attribute__ ((weak)) void BB_UART_rxFrameStartDetectedHook(BB_UART_t* uartPtr){}
__attribute__ ((weak)) void BB_UART_rxFrameCompleteHook(BB_UART_t* uartPtr){}
__attribute__ ((weak)) void BB_UART_rxBlockedHook(BB_UART_t* uartPtr){}
__attribute__ ((weak)) void BB_UART_rxUnblockedHook(BB_UART_t* uartPtr){}
__attribute__ ((weak)) void BB_UART_txBlockedHook(BB_UART_t* uartPtr){}
__attribute__ ((weak)) void BB_UART_txUnblockedHook(BB_UART_t* uartPtr){}
__attribute__ ((weak)) void BB_UART_rxFrameErrorHook(BB_UART_t* uartPtr){}