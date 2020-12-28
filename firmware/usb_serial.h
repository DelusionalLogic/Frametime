#pragma once
#include <stdint.h>

// Setup
void usb_init();
uint8_t usb_configured();

// Serial stuff

#define USB_SERIAL_DTR 0x01
#define USB_SERIAL_RTS 0x02
#define USB_SERIAL_1_STOP 0
#define USB_SERIAL_1_5_STOP 1
#define USB_SERIAL_2_STOP 2
#define USB_SERIAL_PARITY_NONE 0
#define USB_SERIAL_PARITY_ODD 1
#define USB_SERIAL_PARITY_EVEN 2
#define USB_SERIAL_PARITY_MARK 3
#define USB_SERIAL_PARITY_SPACE 4
#define USB_SERIAL_DCD 0x01
#define USB_SERIAL_DSR 0x02
#define USB_SERIAL_BREAK 0x04
#define USB_SERIAL_RI 0x08
#define USB_SERIAL_FRAME_ERR 0x10
#define USB_SERIAL_PARITY_ERR 0x20
#define USB_SERIAL_OVERRUN_ERR 0x40

// RX
int16_t usb_serial_getchar();
uint8_t usb_serial_available();
void usb_serial_flush_input();

// TX
int8_t usb_serial_putchar(uint8_t c);
int8_t usb_serial_putchar_nowait(uint8_t c);
void usb_serial_flush_output();

uint8_t usb_serial_get_control();

void serial_fast_select();
void serial_fast_write(uint8_t c);
void serial_fast_write16(uint16_t n);
void serial_fast_flush();

// Keyboard stuff
extern uint8_t keyboard_keys[6];

// Keyboard keys
#define KEY_A             4
#define KEY_B             5
#define KEY_C             6
#define KEY_D             7
#define KEY_E             8
#define KEY_F             9
#define KEY_G             10
#define KEY_H             11
#define KEY_I             12
#define KEY_J             13
#define KEY_K             14
#define KEY_L             15
#define KEY_M             16
#define KEY_N             17
#define KEY_O             18
#define KEY_P             19
#define KEY_Q             20
#define KEY_R             21
#define KEY_S             22
#define KEY_T             23
#define KEY_U             24
#define KEY_V             25
#define KEY_W             26
#define KEY_X             27
#define KEY_Y             28
#define KEY_Z             29
#define KEY_1             30
#define KEY_2             31
#define KEY_3             32
#define KEY_4             33
#define KEY_5             34
#define KEY_6             35
#define KEY_7             36
#define KEY_8             37
#define KEY_9             38
#define KEY_0             39
#define KEY_ENTER         40
#define KEY_ESC           41
#define KEY_BACKSPACE     42
#define KEY_TAB           43
#define KEY_SPACE         44
#define KEY_MINUS         45
#define KEY_EQUAL         46
#define KEY_LEFT_BRACE    47
#define KEY_RIGHT_BRACE   48
#define KEY_BACKSLASH     49
#define KEY_NUMBER        50
#define KEY_SEMICOLON     51
#define KEY_QUOTE         52
#define KEY_TILDE         53
#define KEY_COMMA         54
#define KEY_PERIOD        55
#define KEY_SLASH         56
#define KEY_CAPS_LOCK     57
#define KEY_F1            58
#define KEY_F2            59
#define KEY_F3            60
#define KEY_F4            61
#define KEY_F5            62
#define KEY_F6            63
#define KEY_F7            64
#define KEY_F8            65
#define KEY_F9            66
#define KEY_F10           67
#define KEY_F11           68
#define KEY_F12           69
#define KEY_PRINTSCREEN   70
#define KEY_SCROLL_LOCK   71
#define KEY_PAUSE         72
#define KEY_INSERT        73
#define KEY_HOME          74
#define KEY_PAGE_UP       75
#define KEY_DELETE        76
#define KEY_END           77
#define KEY_PAGE_DOWN     78
#define KEY_RIGHT         79
#define KEY_LEFT          80
#define KEY_DOWN          81
#define KEY_UP            82
#define KEY_NUM_LOCK      83
#define KEYPAD_SLASH      84
#define KEYPAD_ASTERIX    85
#define KEYPAD_MINUS      86
#define KEYPAD_PLUS       87
#define KEYPAD_ENTER      88
#define KEYPAD_1          89
#define KEYPAD_2          90
#define KEYPAD_3          91
#define KEYPAD_4          92
#define KEYPAD_5          93
#define KEYPAD_6          94
#define KEYPAD_7          95
#define KEYPAD_8          96
#define KEYPAD_9          97
#define KEYPAD_0          98
#define KEYPAD_PERIOD     99

int8_t usb_keyboard_send();
int8_t usb_keyboard_press(uint8_t key);

int8_t usb_keyboard_send();
int8_t usb_keyboard_send_sync();
