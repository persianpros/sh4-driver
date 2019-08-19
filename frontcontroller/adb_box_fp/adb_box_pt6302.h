#if 0
#define DCRAM_COMMAND           (0x20 & 0xf0)
#define CGRAM_COMMAND           (0x40 & 0xf0)
#define ADRAM_COMMAND           (0x60 & 0xf0)
#define NUM_DIGIT_COMMAND       (0xe0 & 0xf0)
#define LIGHT_ON_COMMAND        (0xe8 & 0xfc)
#define DIMMING_COMMAND         (0xe4 & 0xfc)
#define GRAY_LEVEL_DATA_COMMAND (0xa0 & 0xf8)
#define GRAY_LEVEL_ON_COMMAND   (0xc0 & 0xe0)


#define VFDDCRAMWRITE           0xc0425a00
#define VFDBRIGHTNESS           0xc0425a03
#define VFDDISPLAYWRITEONOFF    0xc0425a05
#define VFDDRIVERINIT           0xc0425a08
#define VFDICONDISPLAYONOFF     0xc0425a0a
#endif
/***************************************
 *
 * Define commands to PT6958
 *
 */
#define PT6302_COMMAND_DCRAM_WRITE 1
#define PT6302_COMMAND_CGRAM_WRITE 2
#define PT6302_COMMAND_ADRAM_WRITE 3
#define PT6302_COMMAND_SET_PORTS   4
#define PT6302_COMMAND_SET_DUTY    5
#define PT6302_COMMAND_SET_DIGITS  6
#define PT6302_COMMAND_SET_LIGHTS  7
#define PT6302_COMMAND_TESTMODE    8

// brightness control
#define PT6302_DUTY_MIN  2
#define PT6302_DUTY_NORMAL  7
#define PT6302_DUTY_MAX  7

// setting display width values
#define PT6302_DIGITS_MIN    9
#define PT6302_DIGITS_MAX    16
#define PT6302_DIGITS_OFFSET 8

typedef union
{
	struct
	{
		uint8_t addr: 4, cmd: 4;
	} dcram;
	struct
	{
		uint8_t addr: 3, reserved: 1, cmd: 4;
	} cgram;
	struct
	{
		uint8_t addr: 4, cmd: 4;
	} adram;
	struct
	{
		uint8_t port1: 1, port2: 1, reserved: 2, cmd: 4;
	} port;
	struct
	{
		uint8_t duty: 3, reserved: 1, cmd: 4;
	} duty;
	struct
	{
		uint8_t digits: 3, reserved: 1, cmd: 4;
	} digits;
	struct
	{
		uint8_t onoff: 2, reserved: 2, cmd: 4;
	} lights;
	uint8_t all;
} pt6302_command_t;

static int len_vfd = 16;
// vim:ts=4
