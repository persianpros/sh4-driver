#ifndef _PT6958_H_
#define _PT6958_H_

#define ADB_BOX_BUTTON_UP    0x01
#define ADB_BOX_BUTTON_DOWN  0x02
#define ADB_BOX_BUTTON_LEFT  0x04
#define ADB_BOX_BUTTON_RIGHT 0x08
#define ADB_BOX_BUTTON_POWER 0x10
#define ADB_BOX_BUTTON_MENU  0x20
#define ADB_BOX_BUTTON_EXIT  0x40

/*
 * variables and definitions taken from uboot
 */

//------------------------------------------------------------------------------
#define LOW                     0
#define HIGH                    1
//------------------------------------------------------------------------------
#define FP_CLK_SETUP            SET_PIO_PIN(PIO_PORT(4),0, STPIO_OUT); //BIDIR?
#define FP_DATA_SETUP           SET_PIO_PIN(PIO_PORT(4),1, STPIO_BIDIR);
#define FP_STB_SETUP            SET_PIO_PIN(PIO_PORT(1),6, STPIO_OUT);
#define FP_CSB_SETUP            SET_PIO_PIN(PIO_PORT(1),2, STPIO_OUT);
#define FP_KEY_SETUP            SET_PIO_PIN(PIO_PORT(2),2, STPIO_IN);
//------------------------------------------------------------------------------
#define FP_CLK_BIT(x)           STPIO_SET_PIN(PIO_PORT(4),0, x)
#define FP_DATA_BIT(x)          STPIO_SET_PIN(PIO_PORT(4),1, x)
#define FP_STB_BIT(x)           STPIO_SET_PIN(PIO_PORT(1),6, x)
#define FP_CSB_BIT(x)           STPIO_SET_PIN(PIO_PORT(1),2, x)
//------------------------------------------------------------------------------
#define FP_DATA                 STPIO_GET_PIN(PIO_PORT(4),1)
//------------------------------------------------------------------------------

#define FP_TIMING_PWCLK         1  // clock pulse width is 1us 
#define FP_TIMING_STBCLK        1  // clock strobe time 2us
#define FP_TIMING_CSBCLK        FP_TIMING_STBCLK  // same as STBCLK
#define FP_TIMING_CLKSTB        1  // strobe clock time 2us
#define FP_TIMING_PWSTB         1  // strobe pulse width 2us
#define FP_TIMING_TWAIT         1  // waiting time 2us between ReadCMD & ReadDATA (KEY)
#define FP_TIMING_TDOFF         1  // 300ns delay between data bytes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
typedef struct
{
	char znak;  //original mark
	unsigned char kod;  //at the sign
} pt6958_char_table_t;

//------------------------------------------------------------------------------
#define PT6958_TABLE_LEN 		(sizeof(pt6958_char_table_data) / sizeof(pt6958_char_table_t))
#define PT6958_MAX_CHARS		4

#define LED_LEAVE              0xFF
#define LED_OFF                0
#define LED_ON                 1
#define LED_RED                1
#define LED_GREEN              2
#define LED_ORANGE             3
//------------------------------------------------------------------------------
#define FP_DISP1               PT6958_CMD_ADDR_DISP1
#define FP_DISP2               PT6958_CMD_ADDR_DISP2
#define FP_DISP3               PT6958_CMD_ADDR_DISP3
#define FP_DISP4               PT6958_CMD_ADDR_DISP4
//------------------------------------------------------------------------------
#define FP_LED_POWER           PT6958_CMD_ADDR_LED1
#define FP_LED_CLOCK           PT6958_CMD_ADDR_LED2
#define FP_LED_MAIL            PT6958_CMD_ADDR_LED3
#define FP_LED_WARN            PT6958_CMD_ADDR_LED4

/*********************************************************************
 * Definitions of LED segments
 *
 *      MSB  LSB
 *  bit 76543210
 *
 *      ----- 0
 *     |     |
 *  5  |     |  1
 *  5  |     |
 *   6  -----
 *     |     |
 *  4  |     |  2
 *     |     |
 *      ----- 3
 *
 **********************************************************************/
#define PT6958_LED_A          (0x77|0x00)  //A
#define PT6958_LED_B          (0x7C|0x00)  //b
#define PT6958_LED_C          (0x58|0x00)  //c
#define PT6958_LED_D          (0x5E|0x00)  //d
#define PT6958_LED_E          (0x79|0x00)  //E
#define PT6958_LED_F          (0x71|0x00)  //F
#define PT6958_LED_G          (0x6F|0x00)  //g
#define PT6958_LED_H          (0x74|0x00)  //h
#define PT6958_LED_I          (0x30|0x00)  //i
#define PT6958_LED_J          (0x0e|0x00)  //J
#define PT6958_LED_K          (0x76|0x00)  //K
#define PT6958_LED_L          (0x38|0x00)  //L
#define PT6958_LED_M          (0x37|0x00)  //M
#define PT6958_LED_N          (0x54|0x00)  //n
#define PT6958_LED_O          (0x5C|0x00)  //o
#define PT6958_LED_P          (0x73|0x00)  //P
#define PT6958_LED_Q          (0x67|0x00)  //q
#define PT6958_LED_R          (0x50|0x00)  //r
#define PT6958_LED_S          (0x6D|0x00)  //S
#define PT6958_LED_T          (0x78|0x00)  //t
#define PT6958_LED_U          (0x1C|0x00)  //u
#define PT6958_LED_V          (0x3C|0x00)  //V 
#define PT6958_LED_W          (0x3E|0x00)  //W
#define PT6958_LED_X          (0x76|0x80)  //X 
#define PT6958_LED_Y          (0x66|0x00)  //Y
#define PT6958_LED_Z          (0x5B|0x00)  //Z

/*
 * 0 1 2 3 4 5 6 6 7 8 9
 */
#define PT6958_LED_0_dot      (0x3F|0x80)
#define PT6958_LED_1_dot      (0x06|0x80)
#define PT6958_LED_2_dot      (0x5B|0x80)
#define PT6958_LED_3_dot      (0x4F|0x80)
#define PT6958_LED_4_dot      (0x66|0x80)
#define PT6958_LED_5_dot      (0x6D|0x80)
#define PT6958_LED_6_dot      (0x7D|0x80)
#define PT6958_LED_7_dot      (0x27|0x80)
#define PT6958_LED_8_dot      (0x7F|0x80)
#define PT6958_LED_9_dot      (0x6F|0x80)

#define PT6958_LED_0          (0x3F|0x00)
#define PT6958_LED_1          (0x06|0x00)
#define PT6958_LED_2          (0x5B|0x00)
#define PT6958_LED_3          (0x4F|0x00)
#define PT6958_LED_4          (0x66|0x00)
#define PT6958_LED_5          (0x6D|0x00)
#define PT6958_LED_6          (0x7D|0x00)
#define PT6958_LED_7          (0x27|0x00)
#define PT6958_LED_8          (0x7F|0x00)
#define PT6958_LED_9          (0x6F|0x00)

/*
 * A B C E F H L I O P S T U
 */
#define PT6958_LED_LARGE_A    (0x77|0x80)
#define PT6958_LED_LARGE_B    (0x7C|0x80)
#define PT6958_LED_LARGE_C    (0x39|0x80)
#define PT6958_LED_LARGE_E    (0x79|0x80)
#define PT6958_LED_LARGE_F    (0x71|0x80)
#define PT6958_LED_LARGE_H    (0x76|0x80)
#define PT6958_LED_LARGE_I    (0x30|0x80)
#define PT6958_LED_LARGE_L    (0x38|0x80)
#define PT6958_LED_LARGE_O    PT6958_LED_0
#define PT6958_LED_LARGE_P    (0x73|0x80)
#define PT6958_LED_LARGE_S    (0x6D|0x80)
#define PT6958_LED_LARGE_T    (0x31|0x80)
#define PT6958_LED_LARGE_U    (0x3E|0x80)
/*
 * b c d i g h n o r t
 */
#define PT6958_LED_SMALL_B    (0x7C|0x80)
#define PT6958_LED_SMALL_C    (0x58|0x80)
#define PT6958_LED_SMALL_D    (0x5E|0x80)
#define PT6958_LED_SMALL_I    (0x10|0x80)
#define PT6958_LED_SMALL_G    (0x6F|0x80)
#define PT6958_LED_SMALL_H    (0x74|0x80)
#define PT6958_LED_SMALL_N    (0x54|0x80)
#define PT6958_LED_SMALL_O    (0x5C|0x80)
#define PT6958_LED_SMALL_R    (0x50|0x80)
#define PT6958_LED_SMALL_T    (0x78|0x80)
/*
 * - _ ~ *
 */
#define PT6958_LED_DASH       (0x40|0x00)
#define PT6958_LED_UNDERSCORE (0x08|0x00)
#define PT6958_LED_UPPERSCORE (0x01|0x00)
#define PT6958_LED_EMPTY      (0x00|0x00)
#define PT6958_LED_ASTERISK   (0x63|0x00)
/*
 * .
 */
#define PT6958_LED_DOT        (0x00|0x80)

/*
 * conversion table CHAR -> CODE
 */
static const pt6958_char_table_t pt6958_char_table_data[] =
{
	// special characters
	{ ' ',  PT6958_LED_EMPTY, },
	{ '-',  PT6958_LED_DASH, },
	{ '_',  PT6958_LED_UNDERSCORE, },
	{ '~',  PT6958_LED_UPPERSCORE, },
	{ '*',  PT6958_LED_ASTERISK, },
	// digits
	{ '0',  PT6958_LED_0, },
	{ '1',  PT6958_LED_1, },
	{ '2',  PT6958_LED_2, },
	{ '3',  PT6958_LED_3, },
	{ '4',  PT6958_LED_4, },
	{ '5',  PT6958_LED_5, },
	{ '6',  PT6958_LED_6, },
	{ '7',  PT6958_LED_7, },
	{ '8',  PT6958_LED_8, },
	{ '9',  PT6958_LED_9, },
	// lower case letters
	{ 'a',  PT6958_LED_A, },
	{ 'b',  PT6958_LED_B, },
	{ 'c',  PT6958_LED_C, },
	{ 'd',  PT6958_LED_D, },
	{ 'e',  PT6958_LED_E, },
	{ 'f',  PT6958_LED_F, },
	{ 'g',  PT6958_LED_G, },
	{ 'h',  PT6958_LED_H, },
	{ 'i',  PT6958_LED_I, },
	{ 'j',  PT6958_LED_J, },
	{ 'k',  PT6958_LED_K, },
	{ 'l',  PT6958_LED_L, },
	{ 'm',  PT6958_LED_M, },
	{ 'n',  PT6958_LED_N, },
	{ 'o',  PT6958_LED_O, },
	{ 'p',  PT6958_LED_P, },
	{ 'q',  PT6958_LED_Q, },
	{ 'r',  PT6958_LED_R, },
	{ 's',  PT6958_LED_S, },
	{ 't',  PT6958_LED_T, },
	{ 'u',  PT6958_LED_U, },
	{ 'v',  PT6958_LED_V, },
	{ 'w',  PT6958_LED_W, },
	{ 'x',  PT6958_LED_X, },
	{ 'y',  PT6958_LED_Y, },
	{ 'z',  PT6958_LED_Z, },
	// upper case letters (same as lower case)
	{ 'A',  PT6958_LED_A, },
	{ 'B',  PT6958_LED_B, },
	{ 'C',  PT6958_LED_C, },
	{ 'D',  PT6958_LED_D, },
	{ 'E',  PT6958_LED_E, },
	{ 'F',  PT6958_LED_F, },
	{ 'G',  PT6958_LED_G, },
	{ 'H',  PT6958_LED_H, },
	{ 'I',  PT6958_LED_I, },
	{ 'J',  PT6958_LED_J, },
	{ 'K',  PT6958_LED_K, },
	{ 'L',  PT6958_LED_L, },
	{ 'M',  PT6958_LED_M, },
	{ 'N',  PT6958_LED_N, },
	{ 'O',  PT6958_LED_O, },
	{ 'P',  PT6958_LED_P, },
	{ 'Q',  PT6958_LED_Q, },
	{ 'R',  PT6958_LED_R, },
	{ 'S',  PT6958_LED_S, },
	{ 'T',  PT6958_LED_T, },
	{ 'U',  PT6958_LED_U, },
	{ 'V',  PT6958_LED_V, },
	{ 'W',  PT6958_LED_W, },
	{ 'X',  PT6958_LED_X, },
	{ 'Y',  PT6958_LED_Y, },
	{ 'Z',  PT6958_LED_Z, },
	// other characters
	{ '!',  PT6958_LED_EMPTY, },
	{ '"',  PT6958_LED_EMPTY, },
	{ '$',  PT6958_LED_EMPTY, },
	{ '%',  PT6958_LED_EMPTY, },
	{ '&',  PT6958_LED_EMPTY, },
	{ 0x27, PT6958_LED_EMPTY, },
	{ '(',  PT6958_LED_EMPTY, },
	{ ')',  PT6958_LED_EMPTY, },
	{ '+',  PT6958_LED_EMPTY, },
	{ ',',  PT6958_LED_DOT, },
	{ '.',  PT6958_LED_DOT, },
	{ '/',  PT6958_LED_EMPTY, },
	{ ':',  PT6958_LED_DOT, },
	{ ';',  PT6958_LED_EMPTY, },
	{ '<',  PT6958_LED_EMPTY, },
	{ '=',  PT6958_LED_EMPTY, },
	{ '>',  PT6958_LED_EMPTY, },
	{ '?',  PT6958_LED_EMPTY, },
	{ '[',  PT6958_LED_EMPTY, },
	{ ']',  PT6958_LED_EMPTY, },
	{ '^',  PT6958_LED_EMPTY, },
	{ '{',  PT6958_LED_EMPTY, },
	{ '|',  PT6958_LED_EMPTY, },
	{ '}',  PT6958_LED_EMPTY, },

};
#endif
// vim:ts=4
