#include <gtest/gtest.h>
#include <stdio.h>
#include <ring_buffer.h>

class Ring_Buffer_Test : public testing::Test
{
    protected:
        RING_BUFFER_DEF(test_buffer, 4);
    
    void SetUp() override
    {
        ring_buffer_init(&test_buffer);
    }
    void TearDown() override
    {
        // Empty test buffer
        uint8_t b;
        while(RC_ERROR_BUFFER_EMPTY != ring_buffer_get(&test_buffer, &b));
    }
};

void dumpRingBuffer(ring_buffer *const rb){
    for(uint32_t i = 0; i < rb->len-1; i++){
        printf("[%lu]: %u, ", i, rb->buffer[i]);
    }
    printf("[%lu]: %u\n", rb->len-1, rb->buffer[rb->len-1]);
    printf("head: %lu, tail: %lu", rb->head, rb->tail);
}

TEST_F(Ring_Buffer_Test, IsEmptyFunctionTest){
    // Assert that the ringbuffer is truly empty before beginning test
    ASSERT_EQ(test_buffer.head, test_buffer.tail);
    // check is_empty detection
    EXPECT_TRUE(ring_buffer_is_empty(&test_buffer));
    // add one item to buffer
    uint8_t testVal = 1;
    ASSERT_EQ(RC_SUCCESS, ring_buffer_put(&test_buffer, testVal));
    // check that is_empty returns false with one item in buffer
    EXPECT_FALSE(ring_buffer_is_empty(&test_buffer));
    // retrieve item, thereby deleting it from the buffer
    uint8_t b = 0;
    EXPECT_EQ(RC_SUCCESS, ring_buffer_get(&test_buffer, &b));
    // check correct item was retrieved
    EXPECT_EQ(b, testVal);
    // check that is_empty returns to true after the last item was removed
    // from the buffer
    EXPECT_TRUE(ring_buffer_is_empty(&test_buffer));
}

TEST_F(Ring_Buffer_Test, AvailabilityFunctionTest){
    // With every item put into the buffer, the return of avail should
    // rise by one until the buffer is full
    uint32_t expectedVal = 0;
    while(!ring_buffer_is_full(&test_buffer)){
        EXPECT_EQ(expectedVal, ring_buffer_avail(&test_buffer));
        ASSERT_EQ(RC_SUCCESS, ring_buffer_put(&test_buffer, 0));
        expectedVal++;
    }
    // Check that ring buffer is truly full now
    ASSERT_TRUE(ring_buffer_is_full(&test_buffer));
    // expected return value should not change since the oldest
    // item in the buffer is now overwritten when adding a new one
    ASSERT_EQ(RC_SUCCESS, ring_buffer_put(&test_buffer, 0));
    EXPECT_EQ(expectedVal, ring_buffer_avail(&test_buffer));
}

TEST_F(Ring_Buffer_Test, StateDetectionTest){
    const uint32_t len = test_buffer.len;
    EXPECT_TRUE(ring_buffer_is_empty(&test_buffer));
    // fill ringbuffer except for one byte. Keep in mind
    // that actual capacity of the buffer is len - 1.
    for(uint32_t i = 1; i < len-1; i++){
        RC_t ret = ring_buffer_put(&test_buffer, (uint8_t)i);
        ASSERT_EQ(ret, RC_SUCCESS);
        bool isFull = ring_buffer_is_full(&test_buffer);
        EXPECT_FALSE(isFull);
        if(isFull){
            dumpRingBuffer(&test_buffer);
        }
        EXPECT_EQ(i, ring_buffer_avail(&test_buffer));
    }
    // put in last byte and check that the buffer detects as full
    RC_t ret = ring_buffer_put(&test_buffer, (uint8_t)len);
    ASSERT_EQ(ret, RC_SUCCESS);
    EXPECT_TRUE(ring_buffer_is_full(&test_buffer));
    EXPECT_FALSE(ring_buffer_is_empty(&test_buffer));
}

TEST_F(Ring_Buffer_Test, CircularOverrideTest){
    ASSERT_TRUE(ring_buffer_is_empty(&test_buffer));
    // fill buffer completely but don't try to override anything yet
    for(uint32_t i = 0; i < (test_buffer.len-1); i++){
        ring_buffer_put(&test_buffer, i);
        if(i != test_buffer.len - 2){
            EXPECT_FALSE(ring_buffer_is_full(&test_buffer));
        }
    }
    EXPECT_TRUE(ring_buffer_is_full(&test_buffer));
    // oldest item in buffer should now be a 0 from the first loop iteration
    // add an item with the value of len which should override the oldest value,
    // thereby making the next get call return a 1.
    ASSERT_EQ(RC_SUCCESS, ring_buffer_put(&test_buffer, test_buffer.len));
    uint8_t b = 255;
    ASSERT_EQ(RC_SUCCESS, ring_buffer_get(&test_buffer, &b));
    EXPECT_EQ(b, 1);
    ASSERT_EQ(RC_SUCCESS, ring_buffer_get(&test_buffer, &b));
    EXPECT_EQ(b, 2);
}