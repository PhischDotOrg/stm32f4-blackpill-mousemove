/*-
 * $Copyright$
-*/
#include <common/Infrastructure.hpp>
#include <phisch/log.h>

/* for vTaskStartScheduler */
#include <FreeRTOS.h>
#include <FreeRTOS/include/task.h>

#include <stm32/Cpu.hpp>

#include <stm32/Pll.hpp>
#include <stm32/Pwr.hpp>
#include <stm32/Flash.hpp>
#include <stm32/Gpio.hpp>
#include <stm32/Rcc.hpp>
#include <stm32/Scb.hpp>
#include <stm32/Nvic.hpp>

#include <gpio/GpioAccess.hpp>
#include <gpio/GpioEngine.hpp>
#include <gpio/GpioPin.hpp>

#include <stm32/Uart.hpp>
#include <uart/UartAccess.hpp>
#include <uart/UartDevice.hpp>

#include <tasks/Heartbeat.hpp>

#include <stm32/Itm.hpp>
#include <stm32/Tpi.hpp>
#include <stm32/CoreDbg.hpp>
#include <stm32/DbgMcu.hpp>

#include <usb/UsbCoreViaSTM32F4.hpp>
#include <usb/UsbDeviceViaSTM32F4.hpp>
#include <usb/InEndpointViaSTM32F4.hpp>
#include <usb/OutEndpointViaSTM32F4.hpp>

#include <usb/UsbTypes.hpp>

#include <usb/UsbDevice.hpp>
#include <usb/UsbInEndpoint.hpp>
#include <usb/UsbOutEndpoint.hpp>
#include <usb/UsbControlPipe.hpp>
#include <usb/UsbConfiguration.hpp>
#include <usb/UsbInterface.hpp>

#include <usb/UsbApplication.hpp>
#include <tasks/UsbMouse.hpp>
/*******************************************************************************
 *
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

/*******************************************************************************
 * USB Descriptors (defined in UsbDescriptors.cpp)
 ******************************************************************************/
extern const ::usb::UsbDeviceDescriptor_t usbDeviceDescriptor;
extern const ::usb::UsbStringDescriptors_t usbStringDescriptors;
extern const ::usb::UsbConfigurationDescriptor_t usbConfigurationDescriptor;
extern const uint8_t hidMouseReportDescriptor[50];
#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */


/*******************************************************************************
 * PLL Configuration
 ******************************************************************************/
static const constexpr stm32::PllCfg pllCfg = {
    .m_pllSource        = stm32::PllCfg::PllSource_t::e_PllSourceHSE,
    .m_hseSpeedInHz     = 25 * 1000 * 1000,
    .m_pllM             = 25,
    .m_pllN             = 336,
    .m_pllP             = stm32::PllCfg::PllP_t::e_PllP_Div4,
    .m_pllQ             = stm32::PllCfg::PllQ_t::e_PllQ_Div7,
    .m_sysclkSource     = stm32::PllCfg::SysclkSource_t::e_SysclkPLL,
    .m_ahbPrescaler     = stm32::PllCfg::AHBPrescaler_t::e_AHBPrescaler_None,
    .m_apb1Prescaler    = stm32::PllCfg::APBPrescaler_t::e_APBPrescaler_Div2,
    .m_apb2Prescaler    = stm32::PllCfg::APBPrescaler_t::e_APBPrescaler_None
};

const uint32_t SystemCoreClock = pllCfg.getSysclkSpeedInHz();

static_assert(pllCfg.isValid() == true,                            "PLL Configuration is not valid!");
static_assert(SystemCoreClock               == 84 * 1000 * 1000,   "Expected System Clock to be at 84 MHz!");
static_assert(pllCfg.getAhbSpeedInHz()      == 84 * 1000 * 1000,   "Expected AHB to be running at 84 MHz!");
static_assert(pllCfg.getApb1SpeedInHz()     == 42 * 1000 * 1000,   "Expected APB1 to be running at 42 MHz!");
static_assert(pllCfg.getApb2SpeedInHz()     == 84 * 1000 * 1000,   "Expected APB2 to be running at 84 MHz!");

/*******************************************************************************
 * System Devices
 ******************************************************************************/
static stm32::Scb                       scb(SCB);
static stm32::Nvic                      nvic(NVIC, scb);

static stm32::Pwr                       pwr(PWR);
static stm32::Flash                     flash(FLASH);
static stm32::Rcc                       rcc(RCC, pllCfg, flash, pwr);

/*******************************************************************************
 * GPIO Engine Handlers
 ******************************************************************************/
static stm32::Gpio::A                   gpio_A(rcc);
static gpio::GpioEngine                 gpio_engine_A(&gpio_A);

static stm32::Gpio::B                   gpio_B(rcc);
static gpio::GpioEngine                 gpio_engine_B(&gpio_B);

static stm32::Gpio::C                   gpio_C(rcc);
static gpio::GpioEngine                 gpio_engine_C(&gpio_C);

/*******************************************************************************
 * LEDs
 ******************************************************************************/
static gpio::AlternateFnPin             g_mco1(gpio_engine_A, 8);
static gpio::DigitalOutPin              g_led(gpio_engine_C, 13);

/*******************************************************************************
 * SWO Trace via the Cortex M4 Debug Infrastructure
 ******************************************************************************/
static gpio::AlternateFnPin swo(gpio_engine_B, 3);

static stm32::DbgMcuT<DBGMCU_BASE, decltype(swo)>                   dbgMcu(swo);
static stm32::CoreDbgT<CoreDebug_BASE>                              coreDbg;
static stm32::TpiT<TPI_BASE, decltype(coreDbg), decltype(dbgMcu)>   tpi(coreDbg, dbgMcu);
static stm32::ItmT<ITM_BASE, decltype(tpi)>                         itm(tpi, stm32::Itm::getDivisor(SystemCoreClock /*, 2'250'000 */));

/*******************************************************************************
 * UART
 ******************************************************************************/
static gpio::AlternateFnPin             uart_tx(gpio_engine_A, 3);
static gpio::AlternateFnPin             uart_rx(gpio_engine_A, 2);
static stm32::Uart::Usart2<gpio::AlternateFnPin>    uart_access(rcc, uart_rx, uart_tx);
uart::UartDevice                        g_uart(&uart_access);

/*******************************************************************************
 * USB Device
 ******************************************************************************/
static gpio::AlternateFnPin             usb_pin_dm(gpio_engine_A, 11);
static gpio::AlternateFnPin             usb_pin_dp(gpio_engine_A, 12);
static gpio::AlternateFnPin             usb_pin_vbus(gpio_engine_A, 9);
static gpio::AlternateFnPin             usb_pin_id(gpio_engine_A, 10);

/*******************************************************************************
 * USB Stack
 ******************************************************************************/
static stm32::usb::UsbFullSpeedCoreT<
  decltype(nvic),
  decltype(rcc),
  decltype(usb_pin_dm)
>                                               usbCore(nvic, rcc, usb_pin_dm, usb_pin_dp, usb_pin_vbus, usb_pin_id, /* p_rxFifoSzInWords = */ 256);
static stm32::usb::UsbDeviceViaSTM32F4          usbHwDevice(usbCore);
static stm32::usb::CtrlInEndpointViaSTM32F4     defaultHwCtrlInEndpoint(usbHwDevice, /* p_fifoSzInWords = */ 0x20);

static stm32::usb::IrqInEndpointViaSTM32F4      irqInHwEndp(usbHwDevice, /* p_fifoSzInWords = */ 128, 1);
static usb::UsbIrqInEndpointT                   irqInEndpoint(irqInHwEndp);

static usb::UsbHidInterface                     usbInterface(irqInEndpoint, hidMouseReportDescriptor, sizeof(hidMouseReportDescriptor));

static usb::UsbConfiguration                    usbConfiguration(usbInterface, usbConfigurationDescriptor);

static usb::UsbDevice                           genericUsbDevice(usbHwDevice, usbDeviceDescriptor, usbStringDescriptors, { &usbConfiguration });

static usb::UsbCtrlInEndpointT                                              ctrlInEndp(defaultHwCtrlInEndpoint);
static usb::UsbControlPipe                                                  defaultCtrlPipe(genericUsbDevice, ctrlInEndp);

static usb::UsbCtrlOutEndpointT<stm32::usb::CtrlOutEndpointViaSTM32F4>      ctrlOutEndp(defaultCtrlPipe);
static stm32::usb::CtrlOutEndpointViaSTM32F4                                defaultCtrlOutEndpoint(usbHwDevice, ctrlOutEndp);

static usb::UsbMouseApplicationT                 usbMouseApplication(usbInterface);

/*******************************************************************************
 * Tasks
 ******************************************************************************/
static tasks::HeartbeatT heartbeat_gn("hrtbt_g", g_led, 3, 500);
static tasks::UsbMouseMoverT<
    decltype(usbMouseApplication),
    tasks::UsbMouseMover::CircleT< /* nRadius = */ 50, /* nSpeed = */ 4>
>
usb_move("usb_move", /* p_priority */ 4, /* p_periodMs */ 20, usbMouseApplication);

/*******************************************************************************
 * Queues for Task Communication
 ******************************************************************************/
static SemaphoreHandle_t    usbMutex;

/******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */
/******************************************************************************/

#if defined(HOSTBUILD)
int
#else
[[noreturn]]
void
#endif
main(void) {
    rcc.setMCO(g_mco1, decltype(rcc)::MCO1Output_e::e_PLL, decltype(rcc)::MCOPrescaler_t::e_MCOPre_5);

    uart_access.setBaudRate(decltype(uart_access)::BaudRate_e::e_230400);

    const unsigned sysclk = pllCfg.getSysclkSpeedInHz() / 1000;
    const unsigned ahb    = pllCfg.getAhbSpeedInHz() / 1000;
    const unsigned apb1   = pllCfg.getApb1SpeedInHz() / 1000;
    const unsigned apb2   = pllCfg.getApb2SpeedInHz() / 1000;

    PrintStartupMessage(sysclk, ahb, apb1, apb2);

    /* Inform FreeRTOS about clock speed */
    if (SysTick_Config(SystemCoreClock / configTICK_RATE_HZ)) {
        PHISCH_LOG("FATAL: Capture Error!\r\n");
        goto bad;
    }

    usbHwDevice.start();

    usbMutex = xSemaphoreCreateMutex();
    if (usbMutex == nullptr) {
        PHISCH_LOG("Failed to create USB Mutex!\r\n");
        goto bad;
    }
    usb_move.setUsbMutex(usbMutex);
    PHISCH_LOG("Starting FreeRTOS Scheduler...\r\n");
    vTaskStartScheduler();

    usbHwDevice.stop();

bad:
    if (usbMutex != nullptr) {
        vSemaphoreDelete(usbMutex);
    }

    PHISCH_LOG("FATAL ERROR!\r\n");
    while (1) ;

#if defined(HOSTBUILD)
    return (0);
#endif
}

void
debug_printf(const char * const p_fmt, ...) {
    va_list va;
    va_start(va, p_fmt);

    ::tfp_format(&itm, decltype(itm)::putf, p_fmt, va);

    va_end(va);
}

void
OTG_FS_IRQHandler(void) {
    usbCore.handleIrq();
}

/******************************************************************************/
#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined (__cplusplus) */
/******************************************************************************/

