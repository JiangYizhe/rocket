# RX Interrupt Build Note

This project copy changes the HC-05 UART receive path from polling to an
RXNE interrupt with a 64-byte ring buffer. UART ORE, FE, NE, and PE errors
request parser resynchronization in the main loop.

The unused legacy `Hardware/Serial.c` source is excluded from the Keil
project because it owned the old `USART1_IRQHandler` symbol.

The files already present under `Objects/` and `Listings/` were copied from
the original project and do not contain this change. Rebuild the Keil target
before programming the controller.

