/*
 */
#define TAGDEBUG "[adb_fp] "

#define dprintk(level, x...) do \
	 { \
		if (((paramDebug) && (paramDebug >= level)) || level == 0) \
		{ \
			printk(TAGDEBUG x); \
		} \
	} while (0)

//------------------------------------------------
#define VFD_MAJOR          147

#define SCP_TXD_BIT        6
#define SCP_SCK_BIT        8
#define SCP_ENABLE_BIT     5
#define VFD_PORT           0
#define SCP_DATA           3
#define SCP_CLK            4
#define SCP_CS             5

#define PIO_PORT_SIZE      0x1000
#define PIO_BASE           0xb8020000
#define STPIO_SET_OFFSET   0x4
#define STPIO_CLEAR_OFFSET 0x8
#define STPIO_POUT_OFFSET  0x00
#define STPIO_SET_PIN(PIO_ADDR, PIN, V) \
	writel(1 << PIN, PIO_ADDR + STPIO_POUT_OFFSET + ((V) ? STPIO_SET_OFFSET : STPIO_CLEAR_OFFSET))
#define PIO_PORT(n)        (((n)*PIO_PORT_SIZE) + PIO_BASE)
//------------------------------------------------

/*
  PIO assignments
*/
// PT6302 (VFD display driver)
#define VFD_PIO_PORT_DIN 4
#define VFD_PIO_PIN_DIN  1

#define VFD_PIO_PORT_CLK 4
#define VFD_PIO_PIN_CLK  0

#define VFD_PIO_PORT_CS  1
#define VFD_PIO_PIN_CS   2

// PT6958 (LED display driver & button device)
#define BUTTON_PIO_PORT_DOUT  2
#define BUTTON_PIO_PIN_DOUT   2

#define BUTTON_PIO_PORT_STB   1
#define BUTTON_PIO_PIN_STB    6

#define BUTTON_PIO_PORT_RESET 3
#define BUTTON_PIO_PIN_RESET  2

//PT6958 definitions
/**********************************************************************
 * PT6958 Internal RAM Size: 14 bytes
 *
 * Byte
 *   1 - Data Setting Command
 *   1 - Address Setting Command
 *  10 - Display Data
 *   1 - Display Control Command
 **********************************************************************/
#define PT6958_RAM_SIZE (1 + 1 + 10 + 1)

/**********************************************************************
 * CMD 40: Data Setting Command
 * bits:   0100 dcba
 *
 * ba - Data Write & Read Mode Settings:
 * 00: Write Data to Display Mode
 * 10: Read Key Data
 *
 *  c - Address Incremental Mode Settings (Display Mode):
 *  0: Increment Address after Data has been Written
 *  1: Fixed Address
 *
 *  d - Mode Settings
 *  0: Normal Operation Mode
 *  1: Test Mode
 **********************************************************************/
#define PT6958_CMD_WRITE_INC   0x40
#define PT6958_CMD_READ_KEY    0x42
#define PT6958_CMD_WRITE_FIXED 0x44
#define PT6958_CMD_MODE_NORMAL 0x40
#define PT6958_CMD_MODE_TEST   0x48

/**********************************************************************
 * CMD 8X: Display Control Command
 * bitswise: 1000 dcba
 *
 * cba - Dimming settings:
 * 000: Pulse width = 1/16  (dim)
 * 001: Pulse width = 2/16
 * 010: Pulse width = 4/16
 * 011: Pulse width = 10/16
 * 100: Pulse width = 11/16
 * 101: Pulse width = 12/16
 * 110: Pulse width = 13/16
 * 111: Pulse width = 14/16 (brightest)
 *
 *   d - Display on/off setting
 *   0: Display off (Key Scan Continues)
 *   1: Display on
 **********************************************************************/
#define PT6958_CMD_DISPLAY_OFF        0x80
#define PT6958_CMD_DISPLAY_OFF_DIM(x) (0x80 + x)  // 0x80-0x87
#define PT6958_CMD_DISPLAY_ON_BIT     0b00001000
#define PT6958_CMD_DISPLAY_ON         0x80 + PT6958_CMD_DISPLAY_ON_BIT + 3
#define PT6958_CMD_DISPLAY_ON_DIM(x)  (0x88 + x)  // 0x88-0x8F

/**********************************************************************
 * CMD Cx: Address Setting Command
 * bits:   1100 dcba
 *
 * 0, 2, 4, 6 = digit 1, 2, 3, 4
 * 1, 3, 5, 7 = power, timer, at, alert LEDs
 * 8, 9       = not connected
 *
 * dcba - Address 0x00 to 0x09
 **********************************************************************/
#define PT6958_CMD_ADDR_SET    0xC0 // 0xC0 to 0xC9
//------------------------------------------------------------------------------
#define PT6958_CMD_ADDR_DISP1  0xC0  // leftmost digit  8.:.. 
#define PT6958_CMD_ADDR_DISP2  0xC2  // left digit      .8:..  
#define PT6958_CMD_ADDR_DISP3  0xC4  // right digit     ..:8. 
#define PT6958_CMD_ADDR_DISP4  0xC6  // rightmost digit ..:.8

#define PT6958_CMD_ADDR_LED1   0xC1  // power LED
#define PT6958_CMD_ADDR_LED2   0xC3  // rec or timer LED
#define PT6958_CMD_ADDR_LED3   0xC5  // at LED
#define PT6958_CMD_ADDR_LED4   0xC7  // alert LED

/*
PIO 1.6 [fp_nload  ] [OUT (push-pull)    ] []	- STB
//PIO 2.2 [fp_key    ] [IN  (Hi-Z)         ] []	- DOUT
//PIO 3.3 [fp_ir_in  ] [IN  (Hi-Z)         ] [] - IRDA
PIO 4.0 [fp_clk    ] [Alt-BI (open-drain)] [] 	- CLK
PIO 4.1 [fp_data   ] [Alt-BI (open-drain)] [] 	- DIN
*/
#define FP_GETKEY STPIO_GET_PIN(PIO_PORT(2),2) //4 ?

//------------------------------------------------

#define BUTTON_POLL_MSLEEP 1
#define BUTTON_POLL_DELAY  10

static char                    *button_driver_name = "PT6958 front panel buttons driver";
static struct workqueue_struct *button_wq;
static struct input_dev        *button_dev;

// pin names
static struct stpio_pin *button_dout = NULL;
static struct stpio_pin *button_stb = NULL;
static struct stpio_pin *button_reset = NULL;  // rest function (RES + DOWN)

// PT6302 stuff
#define VFD_MAX_CHARS 16

// display on/off values
#define PT6302_LIGHT_NORMAL 0  // normal display of text
#define PT6302_LIGHT_OFF    1  // all segments off
#define PT6302_LIGHT_ON     3  // all segments on

// pin names
static const char stpio_vfd_cs[] = "pt6302_cs";
static const char stpio_vfd_clk[] = "pt6302_clk";
static const char stpio_vfd_din[] = "pt6302_din";

struct pt6302_pin_driver
{
	struct stpio_pin *pt6302_cs;   // chip select (CS) pin
	struct stpio_pin *pt6302_clk;  // clock (CLK) pin
	struct stpio_pin *pt6302_din;  // data in (DIN) pin
};

struct pt6302_driver
{
	struct pt6302_pin_driver *pt6302_pin_drv;
};

struct vfd_driver
{
	struct pt6302_pin_driver *pt6302_pin_drv;
	struct pt6302_driver     *pt6302_drv;
	struct semaphore         sem;
	int                      opencount;
};


// IOCTL------------------------------------------------

#define VFDIOC_DCRAMWRITE        0xc0425a00
#define VFDIOC_BRIGHTNESS        0xc0425a03
#define VFDIOC_DISPLAYWRITEONOFF 0xc0425a05
#define VFDIOC_DRIVERINIT        0xc0425a08
#define VFDIOC_ICONDISPLAYONOFF  0xc0425a0a

#define VFDSETFAN                0xc0425af6
#define VFDGETVERSION            0xc0425af7
#define VFDLEDBRIGHTNESS         0xc0425af8
#define VFDGETWAKEUPMODE         0xc0425af9
#define VFDGETTIME               0xc0425afa
#define VFDSETTIME               0xc0425afb
#define VFDSTANDBY               0xc0425afc
#define VFDREBOOT                0xc0425afd

#define VFDSETLED                0xc0425afe
#define VFDSETMODE               0xc0425aff

#define VFD_LED_IR	             0x00000023
#define VFD_LED_POW	             0x00000024
#define VFD_LED_REC	             0x0000001e
#define VFD_LED_HD	             0x00000011
#define VFD_LED_LOCK	         0x00000013
#define VFD_LED_PAUSE            0x00000015  // to be corrected

struct vfd_ioctl_data
{
	unsigned char address;
	unsigned char data[64];
	unsigned char length;
};

#if 0
struct __vfd_vfd_pin
{
	__u8 tr_rp_ctrl;
	__u8 rp_data;
	__u8 tr_data;
	__u8 start_tr;
	__u8 status;
	__u8 reserved;
	__u8 clk_div;
};

#define SCP_TXD_CTRL  (vfd_scp_ctrl->tr_rp_ctrl)
#define SCP_TXD_DATA  (vfd_scp_ctrl->tr_data)
#define SCP_TXD_START (vfd_scp_ctrl->start_tr)
#define SCP_STATUS    (vfd_scp_ctrl->status)
#endif

//Global variables
extern int paramDebug;
extern int delay;

// from adb_box_pt6302.c
extern struct pt6302_driver *pt6302_init(struct pt6302_pin_driver *scp);
extern void pt6302_free(struct pt6302_driver *ptd);
extern void pt6302_setup(struct pt6302_driver *pfd);
extern int pt6302_write_dcram(struct pt6302_driver *ptd, unsigned char addr, unsigned char *data, unsigned char len);
extern void pt6302_set_brightness(struct pt6302_driver *ptd, int level);
extern void pt6302_set_light(struct pt6302_driver *ptd, int onoff);
// from adb_box_pt6958.c
extern int pt6958_write(unsigned char cmd, unsigned char *data, int data_len);
extern unsigned char pt6958_read_key(void);
extern int pt6958_led_control(unsigned char led, unsigned char set);
extern void pt6958_display(char *str);
// vim:ts=4
