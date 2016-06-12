/***********************************************************************************
 * Description: hal_odroidxu_gpio.c
 *              Driver for the ODROID XU4 GPIO pins
 * 
 * Author sanjit paonam
 * 
 * Initial version : 
 * Date : 22/03/2016
 *
 ***********************************************************************************/
#define TARGET_PLATFORM_ODROID_XU
#define BUILD_SYS_USER_DSO

#if !defined(BUILD_SYS_USER_DSO) 
#error "This driver is for usermode threads only"
#endif
#if !defined(TARGET_PLATFORM_ODROID_XU)
#error "This driver is for the Odroid XU platform only"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>

#include "rtapi.h"
#include "rtapi_bitops.h"
#include "rtapi_app.h"
#include "hal_gpio_config.h"

#include "hal.h"

MODULE_AUTHOR("Sanjit Paonam");
MODULE_DESCRIPTION("Driver for Odroid XU4 GPIO pins");
MODULE_LICENSE("GPL");

#define MODULE_NAME  "hal_odroidxu_gpio"
#define INPUT  0
#define OUTPUT 1

static const char *modname = MODULE_NAME;
static volatile unsigned int *gpio, *gpio1;
static int comp_id;             /* component ID */
//static int *pinToGpio;
static int  mem_fd;

#define PINS_PER_HEADER  40

typedef struct {
        hal_bit_t* led_pins[4];
        hal_bit_t* input_pins[PINS_PER_HEADER];  // array of pointers to bits
        hal_bit_t* output_pins[PINS_PER_HEADER]; // array of pointers to bits
        hal_bit_t  *led_inv[4];
        hal_bit_t  *input_inv[PINS_PER_HEADER];
        hal_bit_t  *output_inv[PINS_PER_HEADER];
} port_data_t;
        
static port_data_t *port_data;

static char *input_pins;
RTAPI_MP_STRING(input_pins, "input pins, comma separated. 1-30");

static char *output_pins;
RTAPI_MP_STRING(output_pins, "output pins, comma separated. 1-30");

static void write_port(void *arg, long period);
static void read_port(void *arg, long period);
static int  gpioToShiftReg (int pin);
static int  gpioToGPFSELReg (int pin);
static int  gpioToGPLEVReg(int pin);
static int  setup_gpio_access();
void        set_pin_mode(int pin, int mode);
void        initialize_pins_mode();

#if 0
//
// pinToGpio:
//      Take a Wiring pin (0 through X) and re-map it to the ODROIDXU_GPIO pin
//
static int pinToGpioOdroidXU [64] = {
   174, 173,    //  0 |  1 : GPA0.3(UART_0.CTSN), GPA0.2(UART_0.RTSN)
    21,  22,    //  2 |  3 : GPX1.5, GPX1.6
    19,  23,    //  4 |  5 : GPX1.3, GPX1.7
    24,  18,    //  6 |  7 : GPX2.0, GPX1.2

   209, 210,    //  8 |  9 : GPB3.2(I2C_1.SDA), GPB3.3(I2C_1.SCL)
   190,  25,    // 10 | 11 : GPA2.5(SPI_1.CSN), GPX2.1
   192, 191,    // 12 | 13 : GPA2.7(SPI_1.MOSI), GPA2.6(SPI_1.MISO)
   189, 172,    // 14 | 15 : GPA2.4(SPI_1.SCLK), GPA0.1(UART_0.TXD)
   171,  -1,    // 16 | 17 : GPA0.0(UART_0.RXD),
    -1,  -1,    // 18 | 19
    -1,  28,    // 20 | 21 :  , GPX2.4
    30,  31,    // 22 | 23 : GPX2.6, GPX2.7
    -1,  -1,    // 24 | 25   PWR_ON(INPUT), ADC_0.AIN0
    29,  33,    // 26 | 27 : GPX2.5, GPX3.1
    -1,  -1,    // 28 | 29 : REF1.8V OUT, ADC_0.AIN3
   187, 188,    // 30 | 31 : GPA2.2(I2C_5.SDA), GPA2.3(I2C_5.SCL)

    // Padding:
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,     // 32...47
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,     // 48...63
};
#endif

static int pinToGpio [PINS_PER_HEADER] = {
    -1,
    -1, -1,      // 1 | 2 : 5v, GND
    -1, 173,     // 3 | 4 : ADC_0.AIN0,          GPA0.2(UART_0.RTSN)
    174, 171,    // 5 | 6 : GPA0.3(UART_0.CTSN), GPA0.0(UART_0.RXD)
    192, 172,    // 7 | 8 : GPA2.7(SPI_1.MOISI), GPA0.1(UART_0.TXD)
    191, 189,    // 9 | 10: GPA2.6(SPI_1.MISO),  GPA2.4(SPI_1.CLK)
    190, -1,     // 11| 12: GPA2.5(SPI_1.CSN),   PWRON
    21,  210,    // 13| 14: GPX1.5(XE.INT13),    GPB3.3(I2C_1.SCL)
    18,  209,    // 15| 16: GPX1.2(XE.INT10),    GPB3.2(I2_C1.SDA)
    22,  19,     // 17| 18: GPX1.6(XE.INT14),    GPX1.3(XE.INT11)
    30,  28,     // 19| 20: GPX2.6(XE.INT22),    GPX2.4(XE.INT20)
    29,  31,     // 21| 22: GPX2.5(XE.INT21),    GPX2.7(XE.INT23)
    -1,  25,     // 23| 24: XADC0AIN_1(ADC_0.AIN1), GPX2.1(XE.INT17)
    23,  24,     // 25| 26: GPX1.7 (Soft I2C SCL)(XE.INT15), GPX2.0(XE.INT16)
    33,  -1,     // 27| 28: GPX3.1 (Soft I2C SDA)(XE.INT25), GND
    -1,  -1,     // 29| 30: VDD_IO(1.8V),        GND 

    -1, -1, -1, -1, -1, -1, -1, -1, -1,
};
#if 0

//
// physToGpio:
//      Take a physical pin (1 through 40) and re-map it to the ODROIDXU_GPIO pin
//
static int physToGpioOdroidXU [64] =
{
    -1,         //  0
    -1,  -1,    //  1 |  2 : 3.3V, 5.0V
   209,  -1,    //  3 |  4 : GPB3.2(I2C_1.SDA), 5.0V
   210,  -1,    //  5 |  6 : GPB3.3(I2C_1.SCL), GND
    18, 172,    //  7 |  8 : GPX1.2, GPA0.1(UART_0.TXD)
    -1, 171,    //  9 | 10 : GND, GPA0.0(UART_0.RXD)
   174, 173,    // 11 | 12 : GPA0.3(UART_0.CTSN), GPA0.2(UART_0.RTSN)
    21,  -1,    // 13 | 14 : GPX1.5, GND
    22,  19,    // 15 | 16 : GPX1.6, GPX1.3
    -1,  23,    // 17 | 18 : 3.3V, GPX1.7
   192,  -1,    // 19 | 20 : GPA2.7(SPI_1.MOSI), GND
   191,  24,    // 21 | 22 : GPA2.6(SPI_1.MISO), GPX2.0
   189, 190,    // 23 | 24 : GPA2.4(SPI_1.SCLK), GPA2.5(SPI_1.CSN)
    -1,  25,    // 25 | 26 : GND, GPX2.1
   187, 188,    // 27 | 28 : GPA2.2(I2C_5.SDA), GPA2.4(I2C_5.SCL)
    28,  -1,    // 29 | 30 : GPX2.4, GND
    30,  29,    // 31 | 32 : GPX2.6, GPX2.5
    31,  -1,    // 33 | 34 : GPX2.7, GND
    -1,  33,    // 35 | 36 : PWR_ON(INPUT), GPX3.1
    -1,  -1,    // 37 | 38 : ADC_0.AIN0, 1.8V REF OUT
    -1,  -1,    // 39 | 40 : GND, AADC_0.AIN3

    // Not used
    -1, -1, -1, -1, -1, -1, -1, -1, // 41...48
    -1, -1, -1, -1, -1, -1, -1, -1, // 49...56
    -1, -1, -1, -1, -1, -1, -1      // 57...63
};
// gpioToGPFSEL:
// Map a BCM_GPIO pin to it's Function Selection
// control port. (GPFSEL 0-5)
// Groups of 10 - 3 bits per Function - 30 bits per port

static unsigned char gpioToGPFSEL [] =
{
  0,0,0,0,0,0,0,0,0,0,
  1,1,1,1,1,1,1,1,1,1,
  2,2,2,2,2,2,2,2,2,2,
  3,3,3,3,3,3,3,3,3,3,
  4,4,4,4,4,4,4,4,4,4,
  5,5,5,5,5,5,5,5,5,5,
};

// gpioToShift
// Define the shift up for the 3 bits per pin in each GPFSEL port

static unsigned char gpioToShift [] =
{
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
};
#endif

static unsigned char pin_mode [] = 
{
    1, 1, // 0, 1
    1, 1, // 2, 3
    1, 1, // 4, 5
    1, 1, // 6, 7
    1, 1, // 8, 9
    1, 1, // 10, 11
};
//
// offset to the GPIO bit
//
static int gpioToShiftReg(int pin)
{
    switch(pin) {
	case    GPIO_X1_START...GPIO_X1_END:
	    return  (pin - GPIO_X1_START);
	case    GPIO_X2_START...GPIO_X2_END:
	    return  (pin - GPIO_X2_START);
	case    GPIO_X3_START...GPIO_X3_END:
	    return  (pin - GPIO_X3_START);
	case    GPIO_A0_START...GPIO_A0_END:
	    return  (pin - GPIO_A0_START);
	case    GPIO_A2_START...GPIO_A2_END:
	    return  (pin - GPIO_A2_START);
	case    GPIO_B3_START...GPIO_B3_END:
	    return  (pin - GPIO_B3_START);
	default:
	    break;
    }
    return -1;
}

//
// offset to the GPIO Function register
//
static int gpioToGPFSELReg(int pin)
{
    switch(pin) {
	case    GPIO_X1_START...GPIO_X1_END:
	    return  (GPIO_X1_CON_OFFSET >> 2);
	case    GPIO_X2_START...GPIO_X2_END:
	    return  (GPIO_X2_CON_OFFSET >> 2);
	case    GPIO_X3_START...GPIO_X3_END:
	    return  (GPIO_X3_CON_OFFSET >> 2);
	case    GPIO_A0_START...GPIO_A0_END:
	    return  (GPIO_A0_CON_OFFSET >> 2);
	case    GPIO_A2_START...GPIO_A2_END:
	    return  (GPIO_A2_CON_OFFSET >> 2);
	case    GPIO_B3_START...GPIO_B3_END:
	    return  (GPIO_B3_CON_OFFSET >> 2);
	default:
	    break;
    }
    return -1;
}

//
// offset to the GPIO Input regsiter
//
static int gpioToGPLEVReg(int pin)
{
    switch(pin) {
	case    GPIO_X1_START...GPIO_X1_END:
	    return  (GPIO_X1_DAT_OFFSET >> 2);
	case    GPIO_X2_START...GPIO_X2_END:
	    return  (GPIO_X2_DAT_OFFSET >> 2);
	case    GPIO_X3_START...GPIO_X3_END:
	    return  (GPIO_X3_DAT_OFFSET >> 2);
	case    GPIO_A0_START...GPIO_A0_END:
	    return  (GPIO_A0_DAT_OFFSET >> 2);
	case    GPIO_A2_START...GPIO_A2_END:
	    return  (GPIO_A2_DAT_OFFSET >> 2);
	case    GPIO_B3_START...GPIO_B3_END:
	    return  (GPIO_B3_DAT_OFFSET >> 2);
	default:
	    break;
    }
    return -1;
}


static int setup_gpio_access()
{
    rtapi_print_msg(RTAPI_MSG_INFO,"HAL_GPIO: setup_gpio_access\n");
    // open /dev/mem 
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC) ) < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,"HAL_GPIO: setup_gpio_access can't open /dev/mem \n");
	return -EPERM;
    }     
    // mmap GPIO range
    //#define ODROIDXU_GPX_BASE   0x13400000  // GPX0,1,2,3
    gpio = mmap(NULL, BLOCK_SIZE,
	    PROT_READ|PROT_WRITE, MAP_SHARED,
	    mem_fd, ODROIDXU_GPX_BASE);

    if (gpio == MAP_FAILED) {
	rtapi_print_msg(RTAPI_MSG_ERR,
		"HAL_GPIO: setup_gpio_access mmap failed for ODROIDXU_GPX_BASE :%d - %s\n", 
		errno, strerror(errno));
	return -ENOMEM;;                   
    }       

    gpio1 = mmap(NULL, BLOCK_SIZE,
	    PROT_READ|PROT_WRITE, MAP_SHARED,
	    mem_fd, ODROIDXU_GPA_BASE);

    if (gpio1 == MAP_FAILED) {
	rtapi_print_msg(RTAPI_MSG_ERR,
		"HAL_GPIO: setup_gpio_access mmap failed for ODROIDXU_GPA_BASE :%d - %s\n", 
		errno, strerror(errno));
	return -ENOMEM;;                   
    }       

    //pinToGpio = pinToGpioOdroidXU;
    close(mem_fd);
    
    return 0;
}       
  
void set_pin_mode(int pin, int mode)
{
    unsigned int gpio_mask;
    int shift;

    gpio_mask = ODROIDXU_GPIO_MASK;
    rtapi_print_msg(RTAPI_MSG_ERR, "set_pin_mode pin %d mode %d\n", pin, mode);
    //if ((pin & gpio_mask) == 0){          // On-board pin
    if (1){          // On-board pin
	//pin   = pinToGpio [pin];
	shift = (gpioToShiftReg(pin) * 4);
        rtapi_print_msg(RTAPI_MSG_ERR, "set_pin_mode pin %d mode %d shift %d\n", pin, mode, shift);

	if (mode == INPUT){
	    if(pin < 100)
		*(gpio  + gpioToGPFSELReg(pin)) &= ~(0xF << shift);
	    else
		*(gpio1 + gpioToGPFSELReg(pin)) &= ~(0xF << shift);
	}
	else if (mode == OUTPUT){
	    if(pin < 100){
		*(gpio  + gpioToGPFSELReg(pin)) &= ~(0xF << shift);
		*(gpio  + gpioToGPFSELReg(pin)) |=  (0x1 << shift);
	    }
	    else{
		*(gpio1 + gpioToGPFSELReg(pin)) &= ~(0xF << shift);
		*(gpio1 + gpioToGPFSELReg(pin)) |=  (0x1 << shift);
	    }
	}
	else{
	    rtapi_print_msg(RTAPI_MSG_ERR, "set_pin_mode invalid pin mode\n");
	}
    }
    else{
	rtapi_print_msg(RTAPI_MSG_ERR, "set_pin_mode pin out of mask\n");
    }
}

void initialize_pins_mode()
{
    int size, pin_number;

    rtapi_print_msg(RTAPI_MSG_INFO, "initialize_pin_mode \n");

    size = sizeof(pin_mode);
    for(pin_number = 0; pin_number < size; ++pin_number){
	set_pin_mode(pin_number, pin_mode[pin_number]);
    }
}


static int toggle = 0;

static void write_port(void *arg, long period)
{
    int i;
    int gpioPin;
    static volatile unsigned int *gpioPtr;
    port_data_t *port = (port_data_t *)arg;

    //rtapi_print_msg(RTAPI_MSG_INFO, "write_port called for odroid GPIO pin\n");

    if (port == NULL){
	rtapi_print_msg(RTAPI_MSG_INFO, "write_port port is NULL\n");
	return;
    }

    // set output states
    for(i=1; i<= PINS_PER_HEADER; i++){

	if(port->output_pins[i] == NULL) {
            //rtapi_print_msg(RTAPI_MSG_INFO, "write_port called for pin %d\n", i);
	    continue; // short circuit if hal hasn't malloc'd a bit at this location
	}

	gpioPin = pinToGpio[i];

	if(gpioPin  < 100)
	    gpioPtr = gpio;
	else
	    gpioPtr = gpio1;

	if(toggle == 0){
            rtapi_print_msg(RTAPI_MSG_INFO, "write_port gpioPin toggle 0 on %d pin %d\n", gpioPin, i);
	    *(gpioPtr  + gpioToGPLEVReg(gpioPin)) &= ~(1 << gpioToShiftReg(gpioPin));
	}
	else{
            rtapi_print_msg(RTAPI_MSG_INFO, "write_port gpioPin toggle 1 on %d pin %d\n", gpioPin, i);
	    *(gpioPtr  + gpioToGPLEVReg(gpioPin)) |=  (1 << gpioToShiftReg(gpioPin));
	}
#if 0
	if((*(port->output_pins[i]) ^ *(port->output_inv[i])) == 0){
	    *(gpioPtr  + gpioToGPLEVReg(gpioPin)) &= ~(1 << gpioToShiftReg(gpioPin));
	}
	else{
	    *(gpioPtr  + gpioToGPLEVReg(gpioPin)) |=  (1 << gpioToShiftReg(gpioPin));
	}
#endif 
    }
    if (toggle == 0)
	toggle = 1;
    else
	toggle = 0;
    sleep(4);
}

static void read_port(void *arg, long period)
{
    int i;
    int gpioPin;
    static volatile unsigned int *gpioPtr;
    port_data_t *port = (port_data_t *)arg;

    rtapi_print_msg(RTAPI_MSG_INFO, "read_port called\n");
    if (port == NULL){
	rtapi_print_msg(RTAPI_MSG_INFO, "read_port port is NULL\n");
	return;
    }

    // set output states
    for(i=1; i<= PINS_PER_HEADER; i++){

	if(port->input_pins[i] == NULL) 
	    continue; // short circuit if hal hasn't malloc'd a bit at this location

	gpioPin = pinToGpio[i];

	if(gpioPin  < 100)
	    gpioPtr = gpio;
	else
	    gpioPtr = gpio1;

	*(port->input_pins[i]) = (*(gpioPtr  + gpioToGPLEVReg(gpioPin)) & (1 << gpioToShiftReg(gpioPin)) >> gpioToShiftReg(gpioPin)) ^ *(port->input_inv[i]);
    }
}

int rtapi_app_main(void)
{
    int retval;
    char *data, *token;

    gpio = NULL;   
    gpio1= NULL;

    rtapi_print_msg(RTAPI_MSG_INFO, "Odroid XU4 GPIO main\n");
    
    port_data = hal_malloc(sizeof(port_data_t));
    memset(port_data, 0, sizeof(port_data_t));

    if(setup_gpio_access()){
	rtapi_print_msg(RTAPI_MSG_ERR, "Odroid XU4 GPIO failed in setup_gpio_access\n");
	return -1;
    }

    comp_id = hal_init(modname);
    if (comp_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
		"HAL_GPIO: ERROR: hal_init() failed\n");
	return -1;
    }

    //initialize_pins_mode();
    // configure input pin
    if(input_pins != NULL){

	data = input_pins;
	while((token = strtok(data, ",")) != NULL){
	    int pin = strtol(token, NULL, 10);
	    int gpioPin = pinToGpio[pin];

	    data = NULL;
	    if (gpioPin == -1){
		rtapi_print_msg(RTAPI_MSG_ERR, "Odroid XU4 invalid input pin :%d\n", pin);
		continue;
	    }

	    retval = hal_pin_bit_newf(HAL_OUT, &(port_data->input_pins[pin]), comp_id, "odroidxu_gpio.p%d.in-%02d", 0, pin);
	    if(retval < 0){
		rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: pin p%d.%02d could not export input pin, err: %d\n", modname, 0, pin, retval);
		hal_exit(comp_id);
		return -1;
	    }

	    retval = hal_pin_bit_newf(HAL_IN, &(port_data->input_inv[pin]), comp_id, "odroidxu_gpio.p%d.in-%02d.invert", 0, pin);
	    if(retval < 0){
		rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: pin p%d.%02d could not export input invert pin, err: %d\n", modname, 0, pin, retval);
		hal_exit(comp_id);
		return -1;
	    }

	    rtapi_print_msg(RTAPI_MSG_ERR, "%s: done with hal_pin_bit\n", modname);
	    set_pin_mode(gpioPin, INPUT);
	    rtapi_print_msg(RTAPI_MSG_ERR, "%s: after set_pin_mode\n", modname);
	    *(port_data->input_inv[pin]) = 0;
	    rtapi_print_msg(RTAPI_MSG_ERR, "%s: done with assignment\n", modname);
	}
    }

    // configure output pin
    if(output_pins != NULL){

	data = output_pins;
	while((token = strtok(data, ",")) != NULL){
	    int pin = strtol(token, NULL, 10);
	    int gpioPin = pinToGpio[pin];

	    data = NULL;
	    if (gpioPin == -1){
		rtapi_print_msg(RTAPI_MSG_ERR, "Odroid XU4 invalid output pin :%d\n", pin);
		continue;
	    }

	    retval = hal_pin_bit_newf(HAL_IN, &(port_data->output_pins[pin]), comp_id, "odroidxu_gpio.p%d.out-%02d", 0, pin);
	    if(retval < 0){
		rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: pin p%d.%02d could not export output pin, err: %d\n", modname, 0, pin, retval);
		hal_exit(comp_id);
		return -1;
	    }

	    retval = hal_pin_bit_newf(HAL_IN, &(port_data->output_inv[pin]), comp_id, "odroidxu_gpio.p%d.out-%02d.invert", 0, pin);
	    if(retval < 0){
		rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: pin p%d.%02d could not export output invert pin, err: %d\n", modname, 0, pin, retval);
		hal_exit(comp_id);
		return -1;
	    }

	    set_pin_mode(gpioPin, OUTPUT);
	    *(port_data->output_inv[pin]) = 0;
	}
    }

    retval = hal_export_funct("hal_odroidxu_gpio.write", write_port, port_data, 0, 0, comp_id);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
		"HAL_GPIO: ERROR: write funct export failed\n");
	hal_exit(comp_id);
	return -1;
    }

    retval = hal_export_funct("hal_odroidxu_gpio.read", read_port, port_data, 0, 0, comp_id);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
		"HAL_GPIO: ERROR: read funct export failed\n");
	hal_exit(comp_id);
	return -1;
    }

    rtapi_print_msg(RTAPI_MSG_INFO,
	    "HAL_GPIO: installed driver\n");
    hal_ready(comp_id);

    return 0;
}

void rtapi_app_exit(void)
{
    if (gpio && gpio1){
	munmap((void *)gpio, BLOCK_SIZE);
	munmap((void *)gpio1, BLOCK_SIZE);
	hal_exit(comp_id);
    }
}































