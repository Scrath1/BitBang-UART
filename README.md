[![Library CI](https://github.com/Scrath1/BitBang-UART/actions/workflows/githubci.yml/badge.svg)](https://github.com/Scrath1/BitBang-UART/actions/workflows/githubci.yml)
# BitBang-UART
Timer-interrupt-based hardware independent bitbang UART driver library with
support for receiving and transmitting data.

## Supported operating modes
- Tx only
- Rx only
- Rx and Tx (full-duplex)
- One-Wire (half-duplex)

## Installation
This library does not have any external dependencies. Just include
the source and header files in your project and include the `bb_uart.h` file
where you want to define your Bitbang-UART.

## Usage
For specific examples, look to the `examples` folder. Examples prefixed with
`arduino` where written for the Arduino framework. Examples prefixed with
`esp-idf` for the ESP-IDF framework, etc.

1. Configure the pin(s) you are going to use for Rx and Tx operation.
   If you are using one-wire mode, make sure to configure the pin as
   open-drain.
   ```C
    pinMode(TX_PIN, OUTPUT);
    pinMode(RX_PIN, INPUT);
   ```
2. Define functions to tell your UART object how to read and write from the
   hardware. These are specific to each UART object you create since they will
   typically hardcode the pins used during reading and writing.
   ```C
    void bbuart_pinWriteFunc(uint8_t val){
        digitalWrite(TX_PIN, val);
    }
    uint8_t bbuart_pinReadFunc(void){
        return digitalRead(RX_PIN);
    }
   ```
3. Allocate the Rx and Tx buffers and configure the UART struct. If only Rx
   or Tx are used, the other buffer can be omitted.
   ```C
    RING_BUFFER_DEF(bbuart_tx_buf, 128);
    RING_BUFFER_DEF(bbuart_rx_buf, 128);
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
        .mode = BB_UART_ONE_WIRE,
        .oversampling = BB_UART_OVERSAMPLE_3
    };
   ```
4. Create a timer interrupt that triggers in an interval determined
   by the desired baudrate and oversampling setting. For a baudrate of 9600,
   a single bit should be transmitted every 104.167us. If 3 times oversampling
   is configured the timer interrupt has to be triggered every
   (104.167/3) = 34.72us. Try to make this timer as accurate as possible to prevent
   possible sampling errors. If you still run into errors during bit reception,
   try increasing the oversampling rate.
   This timer is responsible for calling the `BB_UART_service()` function
   which handles sampling of the rx line and transmission of single bits.
   ```C
    void timer1ISR(){
        BB_UART_service(&bbuart);
    }
   ```
5. Validate the UART configuration using `BB_UART_validateConfig()` during your
   initialization and start the timer interrupt you configured in the previous
   step.

## Hook functions
This library has a number of hook functions you can overwrite which are triggered
automatically based on events during sending or receiving data.
While the original purpose of these functions is to aid in debugging the library,
they can also be used to handle UART events. Keep in mind that these functions
are still running in the context of a timer interrupt, so keep whatever you do
in them to a minimum in regards to execution time.

Examples:
- `BB_UART_rxFrameCompleteHook()` is called whenever reception of an Rx frame
  has been completed. At the point where it is called, the received byte has been
  added to the internal ringbuffer so this function can be used as an `onReceive`
  callback. It is called irrespectively of any errors during the reception process
  so the presence of data is not guaranteed.
- `BB_UART_rxFrameErrorHook()` is called when an error with the received Rx frame
  was detected, e.g. the start or stop bits don't line up or the parity doesn't
  match the configuration. During this function you can take a snapshot of the
  `__rx_internal.error` register which saves the specific errors that occurred
  using single bits as flags. These can be compared against the errors
  defined in `BB_UART_Rx_Frame_Error_t` to troubleshoot.

For a full list of available hook functions, refer to the `bb_uart.h` file.

## USART?
This is not something I have actually tested but theoretically you should be
able to use this library for USART as well by doing the following:
1. Set oversampling to `BB_UART_OVERSAMPLE_1`.
2. Instead of a timer, use a GPIO interrupt to trigger the `BB_UART_service()`
   function.
3. Apply an external clock to the GPIO pin you configured for the interrupt.

## Running unit tests
This library is built using PlatformIO and uses its integration for unit testing.
To run the tests, use the command `pio test -e native` which will run the tests
using the `native` environment specied in `platformio.ini`.

## Found a Bug?
Feel free to open an issue.

## License
See [LICENSE](LICENSE).

## Credits
Thank you [Lorgres](https://github.com/Lorgres) for helping me debug the
Rx frame alignment.