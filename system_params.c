#include <system_params.h>

struct system_params system_params = {
//.sys_console_dev="dumdum",
.sys_console_dev="usb_serial0",
.user_console_dev="usb_serial0",
//.user_console_dev="usart0",
//.user_console_dev="dumdum",
// POWER_MODE_SLOW_CLK = 16 Mhz  -> 6mA
// + WFI (wait for interrupt)
// + WFE (Wait for event) -> 1.2 mA
// POWER_MODE_FAST_CLK = 96 Mhz -> 270 mA
// + WFI (wait for interrupt)
// + WFE -> 100 mA, but messes upp timing and debugger
// POWER_MODE_DEEPSLEEP saves more, but requires external event to wake up
//.power_mode=POWER_MODE_SLOW_CLK|POWER_MODE_WAIT_WFI
.power_mode=POWER_MODE_SLOW_CLK|POWER_MODE_WAIT_WFI
};

