/**
 * Example created on 2024-11-04
 * using a Seeed Studio XIAO ESP32C3.
 */
#include <Arduino.h>
#include <bb_uart.h>

/**
 * This example uses the TimerOne library for easy timer creation,
 * but any timer that is fast enough and generates interrupts can be used.
 * Library source: https://github.com/PaulStoffregen/TimerOne
 */
#include <TimerOne.h>

/**
 * For the UART to function a timer interrupt is necessary,
 * the frequency of which is calculated by multiplying the desired
 * baudrate with the oversampling setting.
 * In this example, that would be 9600 baud * 3.
 */
#define TIMER_INTERVAL_US (1000000/9600/3)

#define TX_PIN D5
#define RX_PIN D4

/**
 * These functions tell the bitbang UART how to interact with this hardware.
 * The pins are hardcoded per function.
 * If you need multiple bitbang UARTs you have to define these functions
 * for each one with different pins.
 */
void bbuart_pinWriteFunc(uint8_t val){
    digitalWrite(TX_PIN, val);
}
uint8_t bbuart_pinReadFunc(void){
    return digitalRead(RX_PIN);
}
// Create required ringbuffers
RING_BUFFER_DEF(bbuart_tx_buf, 128);
RING_BUFFER_DEF(bbuart_rx_buf, 128);
// create BitBang UART struct and configure it
BB_UART_t bbuart = {
    // baud rate can not be controlled from the UART library but is instead
    // determined by the frequency of the timer interrupt
    .wordLen = BB_UART_WORDLENGTH_8,
    .parity = BB_UART_PARITY_NONE,
    .stopBits = BB_UART_STOPBITS_1,
    .tx_ringBuf = &bbuart_tx_buf,
    .rx_ringBuf = &bbuart_rx_buf,
    .writePinFunc = bbuart_pinWriteFunc,
    .readPinFunc = bbuart_pinReadFunc,
    .mode = BB_UART_RX_TX,
    .oversampling = BB_UART_OVERSAMPLE_3
};

/**
 * The timer interrupt has to call the BB_UART_service function
 * which will handle transmission and reception of bits.
 */ 
void timer1ISR(){
    BB_UART_service(&bbuart);
}

void setup()
{
    // put your setup code here, to run once:

    // Initializes the GPIO pins used for reception and transmission
    pinMode(TX_PIN, OUTPUT);
    pinMode(RX_PIN, INPUT);
    // Validate configuration and initialize internal variables
    if(RC_SUCCESS != BB_UART_validateConfig(&bbuart)){
        while(1);
    }
    // Create timer interrupt for bitbang uart and register
    // the interrupt service routine
    Timer1.initialize(TIMER_INTERVAL_US);
    Timer1.attachInterrupt(timer1ISR);
    Timer1.start();
}

void loop()
{
    // put your main code here, to run repeatedly:
    // Keep printing dots to showcase activity
    BB_UART_putc(&bbuart, '.');
    delay(250);
    // Also echo any data received back on the tx line
    while(ring_buffer_avail(bbuart.rx_ringBuf)){
        char c;
        ring_buffer_get(bbuart.rx_ringBuf, (uint8_t*)(&c));
        BB_UART_putc(&bbuart, c);
    }
}