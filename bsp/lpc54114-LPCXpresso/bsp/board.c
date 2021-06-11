#include "board.h"
#include <rtthread.h>
#include <chip.h>

/* Pin muxing table, only items that need changing from their default pin
   state are in this table. Not every pin is mapped. */
STATIC const PINMUX_GRP_T pinmuxing[] = {
	{0, 29, (IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN)},		/* LED-0 */
	{1, 9,  (IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN)},		/* LED-1 */
	{1, 10, (IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN)},		/* LED-2 */

	{0, 31,	(IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN)},		/* ISP_0 / button_0 */
	{0, 4,	(IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN)},		/* ISP_1 / button_1 */
	{0, 24,	(IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN)},		/* WAKE  / button_2 */

	{1, 6,  (IOCON_FUNC7 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},		/* USB_VBUS */

	{0, 0,	(IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF)},	/* UART RX */
	{0, 1,	(IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF)},	/* UART TX */

	{0, 6,  (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},		/* test 0 */
	{0, 7,  (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},		/* test 1 */

	/* Debugger signals */
	{0, 15, (IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)},		/* SWO */
};

static const uint32_t pinmuxing_ct = sizeof(pinmuxing) / sizeof(PINMUX_GRP_T);

typedef struct {
	uint8_t port;
	uint8_t pin;
} PORT_PIN_T;
static const PORT_PIN_T ledBits[] = {{0, 29}, {1, 10}, {1, 9}};
static const uint32_t ledBits_ct = sizeof(ledBits) / sizeof(PORT_PIN_T);

#define BRIDGE_SSEL_PORT 0
#define BRIDGE_SSEL_PIN 14
static void ConfigureBridgeSSEL(void)
{
	PINMUX_GRP_T pinMuxBridgeSSEL[] = {
		{BRIDGE_SSEL_PORT, BRIDGE_SSEL_PIN, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGITAL_EN)}
	};
	/* Default bits to Link processor powered down. */
	uint32_t functionBits = (IOCON_FUNC1 | IOCON_MODE_PULLDOWN | IOCON_DIGITAL_EN);

	/* Set the bridge SSEL pin to GPIO pull down so we can read the state */
	Chip_IOCON_SetPinMuxing(LPC_IOCON, pinMuxBridgeSSEL, sizeof(pinMuxBridgeSSEL) / sizeof(PINMUX_GRP_T));

	/* Drive the bridge SSEL pin low and then read it back */
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, BRIDGE_SSEL_PORT, BRIDGE_SSEL_PIN);
	Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_SSEL_PORT, BRIDGE_SSEL_PIN, false);

	/* Set direction back to input and if the pin reads high, we know the link processor is powered */
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, BRIDGE_SSEL_PORT, BRIDGE_SSEL_PIN);
	if (Chip_GPIO_GetPinState(LPC_GPIO, BRIDGE_SSEL_PORT, BRIDGE_SSEL_PIN)) {

		/* Set function bits when Link processor present */
		functionBits = (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGITAL_EN);
	}

	pinMuxBridgeSSEL[0].modefunc = functionBits;
	Chip_IOCON_SetPinMuxing(LPC_IOCON, pinMuxBridgeSSEL, sizeof(pinMuxBridgeSSEL) / sizeof(PINMUX_GRP_T));
}

/* Sets up system pin muxing */
void Board_SetupMuxing(void)
{
	/* Enable IOCON clock */
	Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_IOCON);

	Chip_IOCON_SetPinMuxing(LPC_IOCON, pinmuxing, pinmuxing_ct);

	/* Bridge SSEL requires detection to set state correctly */
	ConfigureBridgeSSEL();

	/* IOCON clock left on, this is needed if CLKIN is used. */
}

/* Set up and initialize clocking prior to call to main */
void Board_SetupClocking(void)
{
	/* Set the power mode */
	Chip_POWER_SetVoltage(BOARD_MAINCLOCKRATE);

#if BOARD_USECLKINSRC == (0)
#if ((BOARD_MAINCLOCKRATE == SYSCON_FRO12MHZ_FREQ) || \
	(BOARD_MAINCLOCKRATE == SYSCON_FRO48MHZ_FREQ) || \
	(BOARD_MAINCLOCKRATE == SYSCON_FRO96MHZ_FREQ))
	Chip_SetupFROClocking(BOARD_MAINCLOCKRATE);
    /* Set system clock divider to 1 */
	Chip_Clock_SetSysClockDiv(1);
#else
	/* Setup PLL based on (internal) IRC clocking */
	Chip_SetupIrcClocking(BOARD_MAINCLOCKRATE);
#endif
#else
	/* Setup PLL based on (external) CLKIN clocking */
	Chip_SetupExtInClocking(BOARD_MAINCLOCKRATE);
#endif

	/* Select the CLKOUT clocking source */
	Chip_Clock_SetCLKOUTSource(SYSCON_CLKOUTSRC_MAINCLK, 1);
}
/* Set up and initialize hardware prior to call to main */
void Board_SystemInit(void)
{
	/* Enable All SRAMs */
	Chip_SYSCON_PowerUp(SYSCON_PDRUNCFG_PD_SRAM0 | SYSCON_PDRUNCFG_PD_SRAM1 | SYSCON_PDRUNCFG_PD_SRAM2);
	Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_SRAM1);
	Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_SRAM2);

	/* Setup system clocking and muxing */
	Board_SetupMuxing();
	Board_SetupClocking();
}

/* Initialize the LEDs on the NXP LPC5411x LPCXpresso Board */
static void Board_LED_Init(void)
{
	uint32_t i;

	/* Pin muxing setup as part of board_sysinit */
	for (i = 0; i < ledBits_ct; i++) {
		Chip_GPIO_SetPinDIROutput(LPC_GPIO, ledBits[i].port, ledBits[i].pin);
		Chip_GPIO_SetPinState(LPC_GPIO, ledBits[i].port, ledBits[i].pin, true);
	}
}

/* Set state of all colors in RGB LED */
void Board_LED_RGB_Set(uint8_t led)
{
	uint32_t i;
	
	for (i=0; i<ledBits_ct; i++) {
		Chip_GPIO_SetPinState(LPC_GPIO, ledBits[i].port, ledBits[i].pin, !((led >> i) & 0x01));
	}
}

/* Exingush all LEDs at once */
void Board_LED_RGB_Off(void)
{
	uint32_t i;
	
	for (i=0; i<ledBits_ct; i++) {
		Chip_GPIO_SetPinState(LPC_GPIO, ledBits[i].port, ledBits[i].pin, true);
	}
}

void thread_idle_hook()
{
	uint32_t count;
	while(1)
	{
		count = 0xFFFFF;
		while(count--);
		Chip_GPIO_SetPinState(LPC_GPIO, 0, 29, true);
		count = 0xFFFFF;
		while(count--);
		Chip_GPIO_SetPinState(LPC_GPIO, 0, 29, false);
	}
}


/**
 * This function will initial LPC54xx board.
 */
void rt_hw_board_init()
{
	/* INMUX and IOCON are used by many apps, enable both INMUX and IOCON clock bits here. */
	Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_INPUTMUX);
	Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_IOCON);

	/* get system clock */
	SystemCoreClockUpdate();

	/* Initialize GPIO */
	Chip_GPIO_Init(LPC_GPIO);

	/* Initialize the LEDs. Be careful with below routine, once it's called some of the I/O will be set to output. */
	Board_LED_Init();
	
		/*  */
    rt_thread_idle_sethook(thread_idle_hook);
		/* init systick  1 systick = 1/(100M / 100) 100??systick = 1s*/
    SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);
    /* set pend exception priority */
    NVIC_SetPriority(PendSV_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

#ifdef RT_USING_HEAP
    rt_system_heap_init((void *)HEAP_BEGIN, (void *)HEAP_END);
#endif
#ifdef RT_USING_COMPONENTS_INIT
    /* initialization board with RT-Thread Components */
    rt_components_board_init();
#endif
#ifdef RT_USING_CONSOLE
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif
}


/**
 * This is the timer interrupt service routine.
 */
void SysTick_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}

