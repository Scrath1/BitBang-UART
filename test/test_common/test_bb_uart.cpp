#include <gtest/gtest.h>
#include <bb_uart.h>
#include <stdio.h>

// bit positions in this macro are 0-indexed
#define SET_BIT(p,n) ((p) |= (1 << (n)))
#define CLR_BIT(p,n) ((p) &= ~((1) << (n)))
#define TGL_BIT(p,n) ((p) ^= (1<<(n)))
#define CHK_BIT(p,n) (((p) &  (1<<(n))) >> n)
// len: length of bit mask/number of set bits
// start: start position of bit msk as 0-based index
#define MSK_BIT(len, start) (((1 << len) - 1) << start)

#define STOP_BIT_LEVEL (1)
#define START_BIT_LEVEL (0)

// Library internal functions
extern "C" {
    uint8_t BB_UART_calculateFrameSize(const BB_UART_t* const uartPtr);
    uint32_t BB_UART_numOfSetBits(uint32_t var, uint32_t bitsToEval);
    RC_t BB_UART_createNextFrame(BB_UART_t* uartPtr);
    RC_t BB_UART_transmitBit(BB_UART_t* uartPtr);
    RC_t BB_UART_validateConfig(BB_UART_t* uartPtr);
    RC_t BB_UART_extractData(BB_UART_t* uartPtr, uint16_t* dataOut);
    RC_t BB_UART_receiveBit(BB_UART_t* uartPtr);
}

struct RxTxWireSim{
    static const uint32_t wireLen = 64;
    // rx wire uses single bits to simulate the oversampling
    uint16_t rxWire[wireLen] = {0};
    // tx wire uses bools since there is no oversampling on tx
    bool txWire[wireLen] = {0};
    // The wirePos represents the current position in the wire array.
    // It is incremented whenever the oversamplingPos resets back to 0.
    uint32_t wirePos = 0;
    // The oversampling position represents the current selected bit
    // from each rxWire array entry.
    uint32_t oversamplingPos = 0;
    // Decides after how many clock cycles the oversamplingPos wraps
    // back to 0.
    // Pointer to uart from which the settings will be used for this sim.
    BB_UART_t* uartPtr;

    // Advances the clock by 1 cycle. It is important to note that this refers
    // to the clock interval as determined by the oversampling setting.
    // e.g. if oversampling is 3, every third clock cycle a bit transmission
    // can happen
    // returns true if successfull,
    // false if the testwire is used up or uartPtr wasn't set
    bool advanceClock(){
        if(uartPtr == NULL) return false;
        oversamplingPos = (oversamplingPos+1) % uartPtr->oversampling;
        if(oversamplingPos == 0){
            wirePos++;
        }
        if(wirePos >= wireLen) return false;
        else return true;
    }

    uint8_t sampleRx(){
        if(wirePos < wireLen)
            return CHK_BIT(rxWire[wirePos], oversamplingPos);
        else
            return 0;
    }

    void writeTx(uint8_t val){
        if(wirePos < wireLen)
            txWire[wirePos] = val;
    }

    void resetWires(){
        oversamplingPos = 0;
        wirePos = 0;
        for(uint32_t i = 0; i < wireLen; i++){
            // idle level of wire is high
            rxWire[i] = 0xFFFF;
            txWire[i] = true;
        }
    }

    // Keep in mind here that the rxFrame must be in order of arrival on the
    // data line and thereby follwing this format:
    // MSB                                   LSB
    // | start bit | data | parity | stop bits |
    bool addRxFrame(uint32_t startPos, uint16_t rxFrame){
        if(uartPtr == NULL) return false;
        const uint32_t frameSize = BB_UART_calculateFrameSize(uartPtr);
        if(startPos + frameSize >= wireLen) return false;

        for(uint32_t i = 0; i < frameSize; i++){
            // add fake uart frame
            if(CHK_BIT(rxFrame, frameSize-1-i) == 0){
                // set rxWire value to 0
                rxWire[startPos + i] = 0;
            }
            else{
                // set rxWire value to 1 bits
                rxWire[startPos + i] = MSK_BIT(uartPtr->oversampling, 0);
            }
        }
        return true;
    }

    uint16_t frameFromTxWire(uint32_t startPos){
        const uint32_t frameSize = BB_UART_calculateFrameSize(uartPtr);
        if(startPos + frameSize >= wireLen) return UINT16_MAX;
        uint16_t out = 0;
        for(uint32_t i = 0; i < frameSize; i++){
            out = (out << 1) | txWire[startPos + i];
        }
        return out;
    }
};

RxTxWireSim simWire;
void writePinFunc(uint8_t val){
    simWire.writeTx(val);
}
uint8_t readPinFunc(){
    return simWire.sampleRx();
}

class BB_UART_Test : public testing::Test
{
    protected:
        

        RING_BUFFER_DEF(rx_ringbuf, 8);
        RING_BUFFER_DEF(tx_ringbuf, 8);
        BB_UART_t testUart = {
            .wordLen = BB_UART_WORDLENGTH_8,
            .parity = BB_UART_PARITY_EVEN,
            .stopBits = BB_UART_STOPBITS_1,
            .tx_ringBuf = &tx_ringbuf,
            .rx_ringBuf = &rx_ringbuf,
            .writePinFunc = writePinFunc,
            .readPinFunc = readPinFunc,
            .mode = BB_UART_RX_TX,
            .oversampling = BB_UART_OVERSAMPLE_3
        };

    void SetUp() override
    {
        // set dummy lines to 1 (idle level)
        simWire.uartPtr = &testUart;
        simWire.resetWires();
        BB_UART_validateConfig(&testUart);
    }
    void TearDown() override
    {
        // Empty test buffer
    }

    // converts a txFrame to an rxFrame and vice-versa
    uint16_t invertFrame(uint16_t frame){
        const uint8_t frameSize = BB_UART_calculateFrameSize(&testUart);
        uint16_t out = 0;
        for(uint8_t i = 0; i < frameSize; i++){
            uint8_t b = (frame >> i) & 0b1;
            out = (out << 1) | b;
        }
        return out;
    }
};

TEST_F(BB_UART_Test, FrameSizeCalculationTest){
    // 8N1 Frame should have 10 bits. 1 Start, 8 data, 1 Stop
    BB_UART_t uart8N1 = {
        .wordLen = BB_UART_WORDLENGTH_7,
        .parity = BB_UART_PARITY_EVEN,
        .stopBits = BB_UART_STOPBITS_1,
    };
    EXPECT_EQ(10, BB_UART_calculateFrameSize(&uart8N1));
    // 7E2 Frame should have 11 bits. 1 Start, 7 data, 1 parity, 2 Stop
    BB_UART_t uart7E2 = {
        .wordLen = BB_UART_WORDLENGTH_7,
        .parity = BB_UART_PARITY_EVEN,
        .stopBits = BB_UART_STOPBITS_2,
    };
    EXPECT_EQ(11, BB_UART_calculateFrameSize(&uart7E2));
}

TEST_F(BB_UART_Test, NumOfSetBitsTest){
    uint32_t testVar = 0;
    EXPECT_EQ(0, BB_UART_numOfSetBits(testVar, sizeof(uint32_t)*8));
    testVar = 0b1100;
    // try to check 0 bits
    EXPECT_EQ(0, BB_UART_numOfSetBits(testVar, 0));
    // only check first 3 bits
    EXPECT_EQ(1, BB_UART_numOfSetBits(testVar, 3));
    // check 4 bits
    EXPECT_EQ(2, BB_UART_numOfSetBits(testVar, 4));

    // Try to check with 16 and 32 set bits
    testVar = 0xFFFF;
    EXPECT_EQ(16, BB_UART_numOfSetBits(testVar, sizeof(uint32_t)*8));
    testVar = 0xFFFFFFFF;
    EXPECT_EQ(32, BB_UART_numOfSetBits(testVar, sizeof(uint32_t)*8));

    // Try to check for more bits than there is size
    EXPECT_EQ(32, BB_UART_numOfSetBits(testVar, sizeof(uint64_t)*8));
}

TEST_F(BB_UART_Test, CreateTxFrameTest){
    // ToDo: Implement
    const uint8_t refData = 42;
    ASSERT_EQ(RC_SUCCESS, BB_UART_putc(&testUart, (char)refData));
    EXPECT_EQ(RC_SUCCESS, BB_UART_createNextFrame(&testUart));
    // start bit should be low
    EXPECT_EQ(START_BIT_LEVEL, CHK_BIT(testUart.__tx_internal.frame, 0));

    // check number of set bits of data
    const uint32_t refDataHighBits = BB_UART_numOfSetBits(refData, 8);
    const uint32_t parityBitPos = testUart.wordLen + 1;
    ASSERT_NE(BB_UART_PARITY_NONE, testUart.parity);
    if(testUart.parity == BB_UART_PARITY_EVEN){
        if(refDataHighBits % 2 != 0){
            // uneven number of set bits in data. Parity bit should be set
            // for even parity
            EXPECT_EQ(1, CHK_BIT(testUart.__tx_internal.frame, parityBitPos));
        }
        else{
            EXPECT_EQ(0, CHK_BIT(testUart.__tx_internal.frame, parityBitPos));
        }
    }
    else if(testUart.parity == BB_UART_PARITY_ODD){
        if(refDataHighBits % 2 != 0){
            // uneven number of set bits in data. Parity bit should not be set
            // for odd parity
            EXPECT_EQ(0, CHK_BIT(testUart.__tx_internal.frame, parityBitPos));
        }
        else{
            EXPECT_EQ(1, CHK_BIT(testUart.__tx_internal.frame, parityBitPos));
        }
    }

    // Check that stop bits are set
    EXPECT_EQ(STOP_BIT_LEVEL, CHK_BIT(testUart.__tx_internal.frame, parityBitPos + 1));
    if(testUart.stopBits == BB_UART_STOPBITS_2){
        EXPECT_EQ(STOP_BIT_LEVEL, CHK_BIT(testUart.__tx_internal.frame, parityBitPos + 2));
    }
}

TEST_F(BB_UART_Test, ExtractDataTest){
    const uint8_t refData = 42;
    ASSERT_EQ(RC_SUCCESS, BB_UART_putc(&testUart, (char)refData));
    ASSERT_EQ(RC_SUCCESS, BB_UART_createNextFrame(&testUart));
    uint16_t txFrame = testUart.__tx_internal.frame;
    uint16_t cleanRxFrame = 0;
    // reverse tx frame to get an rx frame
    const uint8_t frameSize = BB_UART_calculateFrameSize(&testUart);
    for(uint8_t i = 0; i < frameSize; i++){
        uint8_t b = (txFrame >> i) & 0b1;
        cleanRxFrame = (cleanRxFrame << 1) | b;
    }
    testUart.__rx_internal.frame = cleanRxFrame;
    uint16_t data = 0;
    // At this point, data extraction should fail since we didn't set the
    // receivedBit counter variable
    EXPECT_EQ(RC_ERROR_INVALID_STATE, BB_UART_extractData(&testUart, &data));
    EXPECT_TRUE(testUart.__rx_internal.error & BB_UART_RX_ERROR_BITCOUNT);
    testUart.__rx_internal.frame = cleanRxFrame;

    testUart.__rx_internal.receivedBitsCnt = frameSize;
    // Try to extract data and compare it against the original
    ASSERT_EQ(RC_SUCCESS, BB_UART_extractData(&testUart, &data));
    EXPECT_EQ(refData, data);

    // Mess with the rx frame and try again to get the various error codes
    // First set stop bit to low while it should be high
    CLR_BIT(testUart.__rx_internal.frame, 0);
    EXPECT_EQ(RC_ERROR_INVALID, BB_UART_extractData(&testUart, &data));
    // internal error variable should note that the stop bit failed
    EXPECT_TRUE(testUart.__rx_internal.error & BB_UART_RX_ERROR_STOPBITS);
    testUart.__rx_internal.frame = cleanRxFrame;
    
    // next mess up the start bit by flipping it from low to high
    SET_BIT(testUart.__rx_internal.frame, frameSize-1);
    EXPECT_EQ(RC_ERROR_INVALID, BB_UART_extractData(&testUart, &data));
    EXPECT_TRUE(testUart.__rx_internal.error & BB_UART_RX_ERROR_STARTBIT);
    testUart.__rx_internal.frame = cleanRxFrame;

    // Finally invert the parity bit
    // Make sure there is a parity bit first.
    ASSERT_NE(BB_UART_PARITY_NONE, testUart.parity);
    TGL_BIT(testUart.__rx_internal.frame, testUart.stopBits);
    EXPECT_EQ(RC_ERROR_INVALID, BB_UART_extractData(&testUart, &data));
    EXPECT_TRUE(testUart.__rx_internal.error & BB_UART_RX_ERROR_PARITY);
}

TEST_F(BB_UART_Test, SingleByteTxTest){
    // Test purely tx operation mode here. This requires oversampling of 1
    testUart.mode = BB_UART_TX_ONLY;
    testUart.oversampling = BB_UART_OVERSAMPLE_1;

    uint8_t refData = 42;
    ASSERT_EQ(RC_SUCCESS, BB_UART_putc(&testUart, refData));
    ASSERT_EQ(RC_SUCCESS, BB_UART_createNextFrame(&testUart));
    const uint16_t txFrame = testUart.__tx_internal.frame;
    uint16_t expectedRxFrame = invertFrame(txFrame);

    // reset UART again after extracting reference txFrame
    ASSERT_EQ(RC_SUCCESS, BB_UART_validateConfig(&testUart));
    ASSERT_EQ(RC_SUCCESS, BB_UART_putc(&testUart, refData));

    const uint32_t frameSize = BB_UART_calculateFrameSize(&testUart);
    for(uint32_t i = 0; i < frameSize * testUart.oversampling; i++){
        BB_UART_service(&testUart);
        simWire.advanceClock();
    }

    uint16_t outputFrame = simWire.frameFromTxWire(0);
    ASSERT_NE(UINT16_MAX, outputFrame);
    EXPECT_EQ(expectedRxFrame, outputFrame);
}

TEST_F(BB_UART_Test, MultiByteRxTest){
    // Test purely tx operation mode here.
    testUart.mode = BB_UART_RX_ONLY;

    uint8_t refData = 42;
    ASSERT_EQ(RC_SUCCESS, BB_UART_putc(&testUart, refData));
    ASSERT_EQ(RC_SUCCESS, BB_UART_createNextFrame(&testUart));
    const uint16_t txFrame = testUart.__tx_internal.frame;
    uint16_t rxTestFrame = invertFrame(txFrame);

    // reset UART again after extracting reference txFrame
    ASSERT_EQ(RC_SUCCESS, BB_UART_validateConfig(&testUart));
    const uint32_t frameSize = BB_UART_calculateFrameSize(&testUart);
    const uint32_t frameStartPos = 10;
    ASSERT_TRUE(simWire.addRxFrame(frameStartPos, rxTestFrame));
    ASSERT_TRUE(simWire.addRxFrame(frameStartPos + frameSize + 1, rxTestFrame));

    // simulate reception process
    do{
        BB_UART_service(&testUart);
    } while(simWire.advanceClock());

    uint8_t data[4];
    uint32_t dataRead = 0;
    dataRead = BB_UART_get(&testUart, data, sizeof(data));
    // 2 bytes should have been on the rx line
    EXPECT_EQ(2, dataRead);
    EXPECT_EQ(data[0], refData);
    EXPECT_EQ(data[1], refData);
}

TEST_F(BB_UART_Test, OneWireModeTest){
    testUart.mode = BB_UART_ONE_WIRE;
    uint8_t refData = 42;
    ASSERT_EQ(RC_SUCCESS, BB_UART_putc(&testUart, refData));
    ASSERT_EQ(RC_SUCCESS, BB_UART_createNextFrame(&testUart));
    const uint16_t txFrame = testUart.__tx_internal.frame;
    uint16_t rxTestFrame = invertFrame(txFrame);

    // reset UART again after extracting reference txFrame
    ASSERT_EQ(RC_SUCCESS, BB_UART_validateConfig(&testUart));
    const uint32_t frameSize = BB_UART_calculateFrameSize(&testUart);
    ASSERT_TRUE(simWire.addRxFrame(0, rxTestFrame));
    ASSERT_TRUE(simWire.addRxFrame(frameSize * 3, rxTestFrame));

    ASSERT_EQ(RC_SUCCESS, BB_UART_putc(&testUart, refData));
    do{
        EXPECT_EQ(RC_SUCCESS, BB_UART_service(&testUart));
        if(testUart.__tx_internal.state != BB_UART_TX_IDLE){
            // EXPECT_EQ(BB_UART_RX_BLOCKED, testUart.__rx_internal.state);
        }
        if(testUart.__rx_internal.state != BB_UART_RX_IDLE){
            // EXPECT_EQ(BB_UART_TX_BLOCKED, testUart.__tx_internal.state);
        }
    } while(simWire.advanceClock());

    // after running the sim, there should be one byte in rx buffer
    uint8_t data[4];
    uint32_t dataRead = 0;
    dataRead = BB_UART_get(&testUart, data, sizeof(data));
    EXPECT_EQ(2, dataRead);
    EXPECT_EQ(data[0], refData);
    EXPECT_EQ(data[1], refData);
    // and one byte should have been sent
    // calculate expected startposition of tx frame
    // After one frame was received, UART has to wait for 2*oversampling+1
    // cycles before resuming tx operation. Since transmission only occur
    // when oversampleCounter == 0 this rounds up to 3 oversampling cycles
    const uint32_t txFrameStartPos = frameSize + 3;
    uint16_t transmittedFrame = simWire.frameFromTxWire(txFrameStartPos);
    testUart.__rx_internal.frame = transmittedFrame;
    testUart.__rx_internal.receivedBitsCnt = frameSize;
    uint16_t b = 0;
    EXPECT_EQ(RC_SUCCESS, BB_UART_extractData(&testUart, &b));
    EXPECT_EQ(refData, b);
}