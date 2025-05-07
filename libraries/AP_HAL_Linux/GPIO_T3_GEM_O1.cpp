#include <AP_HAL/AP_HAL_Boards.h>

#include "GPIO_T3_GEM_O1.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_T3_GEM_O1

const unsigned Linux::GPIO_Sysfs::pin_table[] = {
    [T3_GEM_O1_GPIO_LED_A]  = 413,
    [T3_GEM_O1_GPIO_LED_B]  = 412,
    [T3_GEM_O1_GPIO_BUZZER] = 337,
    [T3_GEM_O1_GPIO_PWM1] = 344,
    [T3_GEM_O1_GPIO_PWM2] = 335,
    [T3_GEM_O1_GPIO_PWM3] = 339,
    [T3_GEM_O1_GPIO_PWM4] = 343,
    [T3_GEM_O1_GPIO_PWM5] = 345,
    [T3_GEM_O1_GPIO_PWM6] = 338,
};

const uint8_t Linux::GPIO_Sysfs::n_pins = _T3_GEM_O1_GPIO_MAX;

static_assert(ARRAY_SIZE(Linux::GPIO_Sysfs::pin_table) == _T3_GEM_O1_GPIO_MAX,
              "GPIO pin_table must have the same size of entries in enum gpio_t3_gem_o1");

#endif
