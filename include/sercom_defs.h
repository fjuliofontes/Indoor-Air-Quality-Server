#ifndef SERCOM_DEFS_H
#define SERCOM_DEFS_H

// Serial2 pin and pad definitions (in Arduino files Variant.h & Variant.cpp)
#define PIN_SERIAL2_RX       (1ul)                // Pin description number for PIO_SERCOM on D1
#define PIN_SERIAL2_TX       (0ul)                // Pin description number for PIO_SERCOM on D0
#define PAD_SERIAL2_TX       (UART_TX_PAD_0)      // SERCOM pad 0 TX
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_1)    // SERCOM pad 1 RX

// Serial3 pin and pad definitions (in Arduino files Variant.h & Variant.cpp)
#define PIN_SERIAL3_RX       (3ul)                // Pin description number for PIO_SERCOM on D3
#define PIN_SERIAL3_TX       (2ul)                // Pin description number for PIO_SERCOM on D2
#define PAD_SERIAL3_TX       (UART_TX_PAD_2)      // SERCOM pad 2 TX
#define PAD_SERIAL3_RX       (SERCOM_RX_PAD_3)    // SERCOM pad 3 RX

#endif