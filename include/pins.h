#pragma once

#include <Arduino.h>

// Per-board pin assignments. Fixed by hardware, not configurable at runtime.
// Selected by which BOARD_* macro platformio.ini's build_flags define for
// the active environment (see that file's env:node_atoms3lite/
// env:node_atomlite vs. env:gateway/env:node comments).
namespace Pins {

#if defined(BOARD_ATOMLITE)

// Classic M5Stack Atom Lite (ESP32, see src/main_atom_node.cpp) - no
// display, touch, SD, or RTC, so only the pins actually used by this
// board's build are defined here. Paired with an M5Stack ATOM Tail485
// (clips onto this board's bottom pogo-pin "tail" interface) - chosen over
// pairing that same module with the AtomS3 Lite below because M5Stack's own
// docs/example code only list ATOM Tail485 as compatible with Atom Lite/
// Matrix, not AtomS3 Lite (a separate "ATOMIC RS485 Base" product covers
// that instead, not used here).
//
// RS485_RX/RS485_TX = GPIO32/GPIO26 come directly from M5Stack's own
// Tail485 example sketch (github.com/m5stack/M5-ProductExampleCodes,
// AtomBase/Tail485/Tail485.ino: RX_PIN=32, TX_PIN=26) rather than being
// derived secondhand the way the AtomS3 Lite's Grove pins below were - this
// is the one RS-485 pairing in this project confirmed directly against a
// manufacturer example, not inferred from connector wire colors.
constexpr gpio_num_t RS485_RX = GPIO_NUM_32;
constexpr gpio_num_t RS485_TX = GPIO_NUM_26;

// Onboard WS2812 RGB LED and single button - used for status feedback in
// place of the Tough's display/touch (see main_atom_node.cpp). GPIO39 is
// one of classic ESP32's input-only pins (34-39) - see setup()'s pinMode
// comment for why that means INPUT, not INPUT_PULLUP, here.
constexpr gpio_num_t RGB_LED = GPIO_NUM_27;
constexpr gpio_num_t BUTTON = GPIO_NUM_39;

#elif defined(BOARD_ATOMS3_POE)

// M5Stack AtomS3 (ESP32-S3) paired with the Atomic PoE Base (W5500
// Ethernet-over-SPI, powered via 802.3af PoE) instead of an RS-485
// tail/base module - see DeviceSettings.h's kEthernetModels and
// NetworkManager.cpp's BOARD_ATOMS3_POE branch. No RS485_RX/RS485_TX here
// at all: this board polls no SPI-CCP equipment (the PoE Base occupies
// the bottom pogo-pin interface an RS485 module would otherwise use), so
// there's nothing to wire.
//
// SCK/MISO/MOSI/CS confirmed against two independent real-hardware
// sources for this exact AtomS3 + Atomic PoE Base pairing (OpenELAB's
// worked example, and a GitHub issue discussing this same board's W5500
// wiring) - RST/INT are simply not wired on this board at all (both -1),
// which is why NetworkManager uses the polling M5_Ethernet library
// instead of the ESP32 core's native ETH.h (that one's beginSPI() rejects
// boards without real RST/INT pins).
constexpr gpio_num_t ETH_SCK = GPIO_NUM_5;
constexpr gpio_num_t ETH_MISO = GPIO_NUM_7;
constexpr gpio_num_t ETH_MOSI = GPIO_NUM_8;
constexpr gpio_num_t ETH_CS = GPIO_NUM_6;

// RS485Bus.cpp references Pins::RS485_RX/TX unconditionally (it's shared,
// generic infra compiled into every board), but this one genuinely has no
// RS-485 transceiver wired to anything - main_atom_node.cpp's setup()
// skips the SpiIm.begin() call for BOARD_ATOMS3_POE entirely, so these
// never actually configure a UART; GPIO_NUM_NC just satisfies the compile-
// time reference safely (not a real pin, can't collide with the SPI/LED/
// button pins above even if something changes that assumption later).
constexpr gpio_num_t RS485_RX = GPIO_NUM_NC;
constexpr gpio_num_t RS485_TX = GPIO_NUM_NC;

// Onboard WS2812 RGB LED and single button - same physical AtomS3 module/
// pins as BOARD_ATOMS3LITE below (the PoE Base only changes what's
// attached to the bottom pogo-pin interface, not the module's own
// GPIO35/41 LED/button).
constexpr gpio_num_t RGB_LED = GPIO_NUM_35;
constexpr gpio_num_t BUTTON = GPIO_NUM_41;

#elif defined(BOARD_ATOMS3LITE)

// M5Stack AtomS3 Lite (ESP32-S3, see src/main_atom_node.cpp) - no display,
// touch, SD, or RTC, so only the pins actually used by this board's build
// are defined here.
//
// RS485_RX/RS485_TX target the AtomS3 Lite's Grove port (G1=GPIO1=white
// wire, G2=GPIO2=yellow wire), wired to an M5Stack Grove RS485 Unit. Grove
// cables are straight-through (color-to-color, no crossover) and that
// unit's own connector is documented as black=GND/red=5V/yellow=UART_RX/
// white=UART_TX - so its white (UART_TX, i.e. the unit's output) wire lands
// on this board's GPIO1, making GPIO1 this MCU's RX pin, and its yellow
// (UART_RX, the unit's input) wire lands on GPIO2, making GPIO2 this MCU's
// TX pin. That Grove connector exposes no separate DE/RE direction-control
// pin, so whatever transceiver chip is on that unit must handle TX/RX
// switching on its own board - RS485Bus deliberately doesn't toggle
// anything for it, same as it doesn't for the Tough's built-in transceiver.
// Unlike the Atom Lite/Tail485 pairing above, this pairing is inferred from
// connector documentation on both sides, not a manufacturer example
// targeting this exact board - worth a real-hardware sanity check first.
// Tried swapping RX/TX as a test (2026-09-12): no change, still zero reply
// at any baud rate either way - see /memories/repo/spi-ccp-rs485-debug.md.
// Reverted to this derived assignment since the swap didn't help.
constexpr gpio_num_t RS485_RX = GPIO_NUM_1;
constexpr gpio_num_t RS485_TX = GPIO_NUM_2;

// Onboard WS2812 RGB LED and single button - used for status feedback in
// place of the Tough's display/touch (see main_atom_node.cpp). Matches
// RTSNow-Gateway's node-firmware, which uses the same board.
constexpr gpio_num_t RGB_LED = GPIO_NUM_35;
constexpr gpio_num_t BUTTON = GPIO_NUM_41;

#else

// M5Stack Tough.

// microSD (TF card) - shares the display's VSPI bus (SCK=18, MOSI=23,
// MISO=38) with its own chip-select line.
constexpr gpio_num_t SD_CS = GPIO_NUM_4;

// RS485 - built into the Tough Ext Board (green port). Serial2, no
// direction/DE pin: the onboard transceiver handles it automatically.
// Source: https://docs.m5stack.com/en/arduino/m5tough/rs485
constexpr gpio_num_t RS485_RX = GPIO_NUM_27;
constexpr gpio_num_t RS485_TX = GPIO_NUM_19;

// Touch controller INT line (active low on new touch data). The PMIC's hard
// power-off cuts the ESP32 and touch IC's supply entirely, so nothing can
// react to a tap - deep sleep instead, with this pin as an ext0 wakeup
// source, keeps the touch IC powered so a tap can wake the chip back up.
constexpr gpio_num_t TOUCH_INT = GPIO_NUM_39;

#endif

}  // namespace Pins
