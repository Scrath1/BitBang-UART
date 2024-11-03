#include <gtest/gtest.h>
extern "C"{
    #include "bb_uart.h"
}
#include <stdio.h>

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0') 

extern "C" {
    uint8_t BB_UART_calculateFrameSize(const BB_UART_t* const uartPtr);
    RC_t BB_UART_createNextFrame(BB_UART_t* uartPtr);
    RC_t BB_UART_extractData(const BB_UART_t* const uartPtr, uint16_t* dataOut);
}

void testPinWriter(uint8_t val){
    if(val == 0) printf("_");
    else printf("-");
}

class BB_UART_Test : public testing::Test
{
    protected:
        RING_BUFFER_DEF(test_buffer, 8);
        BB_UART_t testUart = {
            .wordLen = BB_UART_WORDLENGTH_8,
            .parity = BB_UART_PARITY_EVEN,
            .stopBits = BB_UART_STOPBITS_1,
            .tx_ringBuf = &test_buffer,
            .writePinFunc = testPinWriter
        };
    
    void SetUp() override
    {

    }
    void TearDown() override
    {
        // Empty test buffer
        uint8_t b;
        while(RC_ERROR_BUFFER_EMPTY != ring_buffer_get(&test_buffer, &b));
    }
};



TEST_F(BB_UART_Test, FrameCreationTest){
    char c = 0b00010101;
    RC_t err = BB_UART_putc(&testUart, c);
    ASSERT_EQ(err, RC_SUCCESS);
    ASSERT_EQ(testUart.wordLen, BB_UART_WORDLENGTH_8);
    ASSERT_EQ(testUart.parity, BB_UART_PARITY_EVEN);
    ASSERT_EQ(testUart.stopBits, BB_UART_STOPBITS_1);
    err = BB_UART_createNextFrame(&testUart);
    ASSERT_EQ(err, RC_SUCCESS);
    // order in refFrame is |stopbits|parity|data|start|
    uint16_t refFrame = 0b11000101010;
    uint16_t frame = testUart.__tx_internal.frame;
    EXPECT_EQ(frame, refFrame);
    printf("ref: %0b\n", refFrame);
    printf("fra: %0b\n", frame);
}

TEST_F(BB_UART_Test, SingleByteTransmissionTest){
    BB_UART_putc(&testUart, 'c');
    RC_t err = RC_SUCCESS;
    // Simulate timer triggers
    uint32_t transmittedBits = 0;
    while(true){
        err = BB_UART_transmitBit(&testUart);
        if(err == RC_SUCCESS) transmittedBits++;
        else break;
    }
    EXPECT_EQ(transmittedBits, 11);
}

TEST_F(BB_UART_Test, MultiByteTransmissionTest){
    uint32_t frameSize = testUart.stopBits + testUart.wordLen + 1;
    if(testUart.parity != BB_UART_PARITY_NONE) frameSize++;
    char str[] = "abc";
    uint32_t len = strlen(str);
    RC_t err = BB_UART_put(&testUart, (uint8_t*)str, len);
    ASSERT_EQ(RC_SUCCESS, err);
    // Simulate timer triggers
    uint32_t transmittedBits = 0;
    while(true){
        err = BB_UART_transmitBit(&testUart);
        if(err == RC_SUCCESS) transmittedBits++;
        else break;
    }
    EXPECT_EQ(transmittedBits, frameSize * len);
}

uint32_t reverseBits(uint32_t num)
{
    uint32_t count = sizeof(num) * 8 - 1;
    uint32_t reverse_num = num;

    num >>= 1;
    while (num) {
        reverse_num <<= 1;
        reverse_num |= num & 1;
        num >>= 1;
        count--;
    }
    reverse_num <<= count;
    return reverse_num;
}

TEST_F(BB_UART_Test, FrameDataExtractionTest){
    char testChar = 'a';
    RC_t err = BB_UART_putc(&testUart, testChar);
    ASSERT_EQ(err, RC_SUCCESS);
    err = BB_UART_createNextFrame(&testUart);
    ASSERT_EQ(err, RC_SUCCESS);
    const uint32_t frameSize = BB_UART_calculateFrameSize(&testUart);
    for(uint32_t i = 0; i < frameSize; i++){
        uint8_t b = (testUart.__tx_internal.frame >> i) & 0b1;
        testUart.__rx_internal.frame = (testUart.__rx_internal.frame << 1) | b;
    }
    // bits are reversed when receiving a frame as compared to sending one
    // uint16_t refFrame = reverseBits(testUart.__tx_internal.frame);
    // testUart.__rx_internal.frame = refFrame;
    uint16_t rxData = 0;
    err = BB_UART_extractData(&testUart, &rxData);
    // Expect an error since receivedBitsCnt was not set
    EXPECT_EQ(err, RC_ERROR_INVALID_STATE);
    testUart.__rx_internal.receivedBitsCnt = frameSize;
    err = BB_UART_extractData(&testUart, &rxData);
    // now it should succeed
    EXPECT_EQ(err, RC_SUCCESS);
    EXPECT_EQ(rxData, (uint16_t)testChar);
}