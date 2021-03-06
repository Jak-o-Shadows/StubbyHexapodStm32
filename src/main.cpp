
extern "C"
{
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>

#include <libopencm3/cm3/nvic.h>

#include <libopencm3/stm32/f1/i2c.h>

#include "i2c/i2cMaster.h"

#include "pca9685/pca9685.h"

#include "servo4017/servo4017.h"

#include "serialServo/serialServo.h"

}

#include <stdint.h>

#include "main.hpp"

#include "types/Point.hpp"
#include "Leg.hpp"
#include "Stubby.h"
#include "movementController.hpp"

// Define Functions
#define IsHigh(BIT, PORT) ((PORT & (1 << BIT)) != 0)
#define IsLow(BIT, PORT) ((PORT & (1 << BIT)) == 0)
#define SetBit(BIT, PORT) PORT |= (1 << BIT)
#define ClearBit(BIT, PORT) PORT &= ~(1 << BIT)

// Specific to the comms protocol
#define PackBits(V, P) (((V << 5) & 0xFF00) | (V & 0x07) | 0x8000 | (P << 3))

// Comms
typedef enum servoFlags_e
{
	ENGCACHE = 0,
	SERVOON,
	SERVOOFF,
	CMD2CUR
} servoFlags_t;

// Config
#define NUMSERVOS 19
#define IDOFF 0

const uint8_t SERVOADDRESSES[] = {0 + IDOFF,
								  1 + IDOFF,
								  2 + IDOFF,
								  3 + IDOFF,
								  4 + IDOFF,
								  5 + IDOFF,
								  6 + IDOFF,
								  7 + IDOFF,
								  8 + IDOFF,
								  9 + IDOFF,
								  10 + IDOFF,
								  11 + IDOFF,
								  12 + IDOFF,
								  13 + IDOFF,
								  14 + IDOFF,
								  15 + IDOFF,
								  16 + IDOFF,
								  17 + IDOFF,
								  18 + IDOFF};

#define Version 5

#define ReplyPos 0
#define ReplyCur 1
#define ReplyVer 2

// Variables

static uint16_t cachepos[NUMSERVOS];
static uint16_t cmdpos[NUMSERVOS];
static uint16_t _cmdpos[NUMSERVOS];
static uint8_t listen[NUMSERVOS];

static uint16_t updatePeriod_ms = 1000;
static uint16_t updateCounter_ms = 0;

//comms side stuff
unsigned int dobyte(char data);
unsigned int servoCmd(unsigned int command, unsigned int argument);

// ----------HEXAPOD THINGS---------------
static Leg legs[LEG_COUNT] = {
	Leg(FRONT_LEFT, 0, 1, 2, 2 * LEG_MOUNTING_ANGLE, Point(-60, 104, 0), cmdpos),
	Leg(MIDDLE_LEFT, 3, 4, 5, 3 * LEG_MOUNTING_ANGLE, Point(-120, 0, 0), cmdpos),
	Leg(REAR_LEFT, 6, 7, 8, 4 * LEG_MOUNTING_ANGLE, Point(-60, -104, 0), cmdpos),
	Leg(REAR_RIGHT, 9, 10, 11, 5 * LEG_MOUNTING_ANGLE, Point(60, -104, 0), cmdpos),
	Leg(MIDDLE_RIGHT, 12, 13, 14, 0 * LEG_MOUNTING_ANGLE, Point(120, 0, 0), cmdpos),
	Leg(FRONT_RIGHT, 15, 17, 18, 1 * LEG_MOUNTING_ANGLE, Point(60, 104, 0), cmdpos)};

void doResetLegs()
{
	for (uint8_t l = 0; l < LEG_COUNT; l += 2)
	{
		legs[l].setOffset(Point(0, 0, 30));
	}
	//pwm_apply_batch();
	//delay_ms(200);
	for (int i = 0; i < 100000; i++)
	{
		__asm__("nop");
	}

	for (uint8_t l = 0; l < LEG_COUNT; l += 2)
	{
		legs[l].setOffset(Point(0, 0, 0));
	}
	//pwm_apply_batch();
	//delay_ms(200);
	for (int i = 0; i < 100000; i++)
	{
		__asm__("nop");
	}

	for (uint8_t l = 1; l < LEG_COUNT; l += 2)
	{
		legs[l].setOffset(Point(0, 0, 30));
	}
	//pwm_apply_batch();
	//delay_ms(200);
	for (int i = 0; i < 100000; i++)
	{
		__asm__("nop");
	}

	for (uint8_t l = 1; l < LEG_COUNT; l += 2)
	{
		legs[l].setOffset(Point(0, 0, 0));
	}
	//pwm_apply_batch();
	//delay_ms(200);
	for (int i = 0; i < 100000; i++)
	{
		__asm__("nop");
	}
}

// 4017 servo driver things
static uint8_t servoIndexStart_4017 = 400; // The first servo that the 4017's control
static servo4017_t servo4017Device = {
	41.6666,
	60,
	0,
	{0, 0, 0, 0, 0, 0, 0, 0, 0},
};

// Serial servo driver things
static const uint8_t servoIndexStart_serial = 16;



// ----------Microcontroller things--------
static void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	// Enable GPIOA clock (for LED GPIOs).
	rcc_periph_clock_enable(RCC_GPIOC);

	rcc_periph_clock_enable(RCC_GPIOA);

	// Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2.
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_AFIO);

	rcc_periph_clock_enable(RCC_GPIOB);
	//I2C
	rcc_periph_clock_enable(RCC_I2C2);

	// Enable clocks for USART1 on PA9/PA10
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_AFIO);
}

static void usart_setup(void)
{

	// USART 2 on PA2, PA3
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO2); //USART 2 TX is A2
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO3);					  //USART 2 RX is A3

	usart_set_baudrate(USART2, 9600);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
	//enable interrupt rx
	USART_CR1(USART2) |= USART_CR1_RXNEIE;

	usart_enable(USART2);

	// USART 1 on PA9, PA10
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9); //USART 1 TX is A9
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO10);					  //USART 1 RX is A10

	usart_set_baudrate(USART1, 9600);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	//enable interrupt rx
	//USART_CR1(USART1) |= USART_CR1_RXNEIE;

	usart_enable(USART1);




}

static void nvic_setup(void)
{
	// Without this the RTC interrupt routine will never be called.
	nvic_enable_irq(NVIC_USART2_IRQ);
	nvic_set_priority(NVIC_USART2_IRQ, 2);

	nvic_set_priority(NVIC_TIM3_IRQ, 0);
}

static void gpio_setup(void)
{
	gpio_set(GPIOC, GPIO13);

	// Setup GPIO for LED use.
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
				  GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

	//setup i2c pins
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
				  GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
				  GPIO10 | GPIO11); //B10 =SCL, B11=SDA
									//		  GPIO6 | GPIO7);
}

static void timer_setup(void)
{

	rcc_periph_clock_enable(RCC_TIM2);

	nvic_enable_irq(NVIC_TIM2_IRQ);

	rcc_periph_reset_pulse(RST_TIM2);

	//Timer global mode:
	//	no divider
	//	alignment edge
	//	direction up
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	// Note that TIM2 on APB1 is running at double frequency according to
	//	https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/f1/stm32-h103/timer/timer.c
	timer_set_prescaler(TIM2, ((rcc_apb1_frequency * 2) / 10000));

	// Disable preload
	timer_disable_preload(TIM2);
	timer_continuous_mode(TIM2);

	// Count the full range, as the compare value is used to set the value
	timer_set_period(TIM2, 65535);

	timer_set_oc_value(TIM2, TIM_OC1, 10); //was 10000

	timer_enable_counter(TIM2);

	timer_enable_irq(TIM2, TIM_DIER_CC1IE);
}

/////////////////////////////////////////////////////
////////////// Main Loop   //////////////////////////
/////////////////////////////////////////////////////

int main(void)
{

	clock_setup();
	gpio_setup();
	timer_setup();
	usart_setup();
	//i2cMaster_setup(I2C2);
	nvic_setup();

	//pca9685_setup(I2C2, 0x80);

	servo4017_setup(&servo4017Device);

	serialServo_setup(USART1);

	// Manually centre the legs
	for (int i = 0; i < NUMSERVOS; i++)
	{
		// Set the hidden and commanded values to different,
		//	so that it automatically gets set.
		cmdpos[i] = 336; //0x7F;
		_cmdpos[i] = 0;
	}
	// wait for it to apply
	for (int i = 0; i < 100000; i++)
	{
		__asm__("nop");
	}

	// Hexapod reset legs
	doResetLegs();

	while (1)
	{
		for (int i = 0; i < 2000000; i++)
		{
			__asm__("nop");
		}
		walk(legs);
		//translate(legs);
	}
}

// Functions

void debugToggle(void)
{
	//pass
}

/////////////////////////////////////////////////////
////////////// Comms Stuff //////////////////////////
/////////////////////////////////////////////////////

unsigned int dobyte(char data)
{

	static unsigned char state = 0;
	static unsigned int command;
	static unsigned int argument;

	if (state == 0)
	{
		if (IsLow(7, data))
		{
			state = 1;
			command = (data >> 3);
			argument = (argument & 0xFFF8) | (data & 0x07); // glue in its 0 through 2
		}
	}
	else
	{
		state = 0;
		if (IsHigh(7, data))
		{
			argument = (argument & 0x0007) | ((data & 0x7F) << 3); //glue in bits 3 through 9
			return servoCmd(command, argument);
		}
	}

	return 0;
}

// 0  listen (servo number) 256 = all                      {always obey command} // sticks through listen once
// 1  ignore (servo number) 256 = all                      {always obey command} // overrides listen once
// 2  One Time listen (servo number)                       {always obey command}
// 3  set flags (flags) (+toggle debug)                    { bitwise obey }
//    0 enguage cached position                              {always obey command}
//    1 turn servo on                                        {obey if listening}
//    2 turn servo off                                       {obey if listening}
//    3 set cmdpos to curpos                                 {obey if listening}
// 4  set servo position (position)                        {obey if listening}
// 5  set cached position (position)                       {obey if listening}
// 6 get servo current  (servo number)                    {servo number}
// 7 get servo position (servo number)                    {servo number}
// 8 send device model  (servo number)                    {servo number}

unsigned int servoCmd(unsigned int command, unsigned int argument)
{

	unsigned int reply;
	static unsigned int chainAddress = 1023;

	reply = 0;

	switch (command)
	{

	case 0: // listen(id)
		chainAddress = 1023;
		if (argument == 256)
		{
			// Listen all
			for (uint8_t servoIdx = 0; servoIdx < NUMSERVOS; servoIdx++)
			{
				listen[servoIdx] |= 2;
			}
		}
		else
		{
			for (uint8_t servoIdx = 0; servoIdx < NUMSERVOS; servoIdx++)
			{
				if (argument == SERVOADDRESSES[servoIdx])
				{
					listen[servoIdx] |= 2;
					break;
				}
			}
		}
		break;

	case 1: // ignore(id)
		chainAddress = 1023;
		if (argument == 256)
		{
			// Listen all
			for (uint8_t servoIdx = 0; servoIdx < NUMSERVOS; servoIdx++)
			{
				listen[servoIdx] = 0;
			}
		}
		else
		{
			// Listen Specific
			for (uint8_t servoIdx = 0; servoIdx < NUMSERVOS; servoIdx++)
			{
				if (argument == SERVOADDRESSES[servoIdx])
				{
					listen[servoIdx] = 2;
					break;
				}
			}
		}
		break;

	case 2: // listen to only the next command
		chainAddress = 1023;
		if (argument == 256)
		{
			// Listen all
			for (uint8_t servoIdx = 0; servoIdx < NUMSERVOS; servoIdx++)
			{
				listen[servoIdx] |= 1;
			}
		}
		else if (argument >= 512)
		{
			//Update chainAddress
			chainAddress = argument - 512;
			// Then set listen based on chainAddress
			for (uint8_t servoIdx = 0; servoIdx < NUMSERVOS; servoIdx++)
			{
				if (chainAddress == SERVOADDRESSES[servoIdx])
				{
					listen[servoIdx] |= 1;
					break;
				}
			}
		}
		else
		{
			for (uint8_t servoIdx = 0; servoIdx < NUMSERVOS; servoIdx++)
			{
				if (argument == SERVOADDRESSES[servoIdx])
				{
					listen[servoIdx] |= 1;
					break;
				}
			}
		}
		break;

		//       0 enguage cached position                              {always obey command}
		//       1 turn servo on                                        {obey if listening}
		//       2 turn servo off                                       {obey if listening}
		//       3 set cmdpos to curpos                                 {obey if listening}

	case 3: // set flags
		debugToggle();

		if (IsHigh(ENGCACHE, argument))
		{
			for (uint8_t servoIdx = 0; servoIdx < NUMSERVOS; servoIdx++)
			{
				cmdpos[servoIdx] = cachepos[servoIdx];
			}
		}

		if (IsHigh(CMD2CUR, argument))
		{
			for (uint8_t servoIdx = 0; servoIdx < NUMSERVOS; servoIdx++)
			{
				if (listen[servoIdx])
				{
					cmdpos[servoIdx] = 0xFF; // Rue originally read the ADC values
				}
			}
		}

		if (IsHigh(SERVOON, argument))
		{
			for (uint8_t servoIdx = 0; servoIdx < NUMSERVOS; servoIdx++)
			{
				if (listen[servoIdx])
				{
					// Turn on
				}
			}
		}
		else if (IsHigh(SERVOOFF, argument))
		{
			for (uint8_t servoIdx = 0; servoIdx < NUMSERVOS; servoIdx++)
			{
				if (listen[servoIdx])
				{
					// Turn Off
				}
			}
		}
		break;

	case 4: // set servo position
		for (uint8_t servoIdx = 0; servoIdx < NUMSERVOS; servoIdx++)
		{
			if (listen[servoIdx])
			{
				cmdpos[servoIdx] = argument;
			}
		}
		break;

	case 5: // set cached position
		for (uint8_t servoIdx = 0; servoIdx < NUMSERVOS; servoIdx++)
		{
			if (listen[servoIdx])
			{
				cachepos[servoIdx] = argument;
			}
		}
		break;

	case 6: // get servo current
		for (uint8_t servoIdx = 0; servoIdx < NUMSERVOS; servoIdx++)
		{
			if (argument == SERVOADDRESSES[servoIdx])
			{
				reply = PackBits(0, ReplyCur);
				break;
			}
		}
		break;

	case 7: // get servo position
		for (uint8_t servoIdx = 0; servoIdx < NUMSERVOS; servoIdx++)
		{
			if (argument == SERVOADDRESSES[servoIdx])
			{
				reply = PackBits(1, ReplyCur);
				break;
			}
		}
		break;

	case 8: // get model
		for (uint8_t servoIdx = 0; servoIdx < NUMSERVOS; servoIdx++)
		{
			if (argument == SERVOADDRESSES[servoIdx])
			{
				reply = PackBits(Version, ReplyCur);
				break;
			}
		}
		break;
	}

	switch (command)
	{ // clear one time flags
	case 3:
	case 4:
	case 5:
		for (uint8_t servoIdx = 0; servoIdx < NUMSERVOS; servoIdx++)
		{
			listen[servoIdx] &= 2;
		}
		if (chainAddress != 1023)
		{
			chainAddress++;
			for (uint8_t servoIdx = 0; servoIdx < NUMSERVOS; servoIdx++)
			{
				if (chainAddress == SERVOADDRESSES[servoIdx])
				{
					listen[servoIdx] = 1;
					break;
				}
			}
		}
		break;
	}

	return reply;
}

// Interrupt Functions

void usart2_isr(void)
{
	static uint8_t data = 'A';
	static uint8_t reply = 0;

	// Check if we were called because of RXNE.
	if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
		((USART_SR(USART2) & USART_SR_RXNE) != 0))
	{

		// Receieve the data, using the MiniSSC protocol
		//	This protocol has a header byte (0xFF), followed
		//	by a number (0->254) followed by a number (0-254)
		data = usart_recv(USART2);
		reply = dobyte(data);

		// Enable transmit interrupt so it sends back the data.
		USART_CR1(USART2) |= USART_CR1_TXEIE;
	}

	// Check if we were called because of TXE.
	if (((USART_CR1(USART2) & USART_CR1_TXEIE) != 0) &&
		((USART_SR(USART2) & USART_SR_TXE) != 0))
	{

		// Indicate that we are sending out data.
		// gpio_toggle(GPIOA, GPIO7);

		// Put data into the transmit register.
		usart_send(USART2, reply);

		// Disable the TXE interrupt as we don't need it anymore.
		USART_CR1(USART2) &= ~USART_CR1_TXEIE;
	}
}

void tim2_isr(void)
{
	// This timer is used to update the servo position from the in-memory position.

	// This timer ticks every 1ms
	if (timer_get_flag(TIM2, TIM_SR_CC1IF))
	{
		timer_clear_flag(TIM2, TIM_SR_CC1IF);

		// Setup next compare time
		uint16_t compare_time = timer_get_counter(TIM2);
		timer_set_oc_value(TIM2, TIM_OC1, 10 + compare_time);

		// Only update at a specific rate
		updateCounter_ms++;
		if (updateCounter_ms >= updatePeriod_ms)
		{
			// Reset counter
			updateCounter_ms = 0;

			// Do work
			gpio_toggle(GPIOC, GPIO13);

			// Set servo position if different
			for (uint8_t servoIdx = 0; servoIdx < NUMSERVOS; servoIdx++)
			{

				if (1)//(_cmdpos[servoIdx] != cmdpos[servoIdx])
				{
					_cmdpos[servoIdx] = cmdpos[servoIdx];

					if( servoIdx >= servoIndexStart_serial){
						if(servoIdx >= servoIndexStart_4017){
							//timerCompareValue_4017[servoIdx - servoIndexStart_4017] = cmdpos[servoIdx];
						} else {
							serialServo_setServoPos(USART1, servoIdx-servoIndexStart_serial, cmdpos[servoIdx]);
						}
					} else {
						//pca9685_setServoPos(I2C2, 0x80, servoIdx, _cmdpos[servoIdx]);
					}
				}
			}
		}
	}
}

void tim3_isr(void)
{
	static uint8_t idxA; // how far through the 8 servos the 4017 TIM3_CH1 controls is

	if (timer_get_flag(TIM3, TIM_SR_CC1IF))
	{														 // from compare1?
		timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_FORCE_LOW); //     as per OC1M page 329
		timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_ACTIVE);	 //     as per OC1M page 329

		//TIM3_CCR1 += 10000; //timerCompareValue_4017[0 * 8 + idxA];
		uint16_t compare_time = timer_get_counter(TIM3);
		timer_set_oc_value(TIM3, TIM_OC1, servo4017Device.pulseLengths_ticks[0 * 8 + idxA] + compare_time);

		//gpio_toggle(GPIOA, GPIO5);

		if (idxA == 8)
		{ // do 4017 reset
			gpio_set(GPIOA, GPIO5);
			__asm__("nop");
			__asm__("nop");
			gpio_clear(GPIOA, GPIO5);
		}
		if (++idxA == 9)
		{
			idxA = 0;
		}

		timer_clear_flag(TIM3, TIM_SR_CC1IF);
	}
}
