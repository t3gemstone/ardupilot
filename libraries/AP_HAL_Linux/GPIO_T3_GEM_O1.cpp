#include <AP_HAL/AP_HAL_Boards.h>

#include "GPIO_T3_GEM_O1.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_T3_GEM_O1

const unsigned Linux::GPIO_Sysfs::pin_table[] = {
    [T3_GEM_O1_GPIO_LED_GREEN] = 678,
    [T3_GEM_O1_GPIO_LED_RED]   = 679,
    [T3_GEM_O1_GPIO2]          = 533,
    [T3_GEM_O1_GPIO3]          = 532,
    [T3_GEM_O1_GPIO4]          = 577,
    [T3_GEM_O1_GPIO5]          = 641,
    [T3_GEM_O1_GPIO6]          = 643,
    [T3_GEM_O1_GPIO7]          = 524,
    [T3_GEM_O1_GPIO8]          = 515,
    [T3_GEM_O1_GPIO9]          = 519,
    [T3_GEM_O1_GPIO10]         = 518,
    [T3_GEM_O1_GPIO11]         = 517,
    [T3_GEM_O1_GPIO12]         = 642,
    [T3_GEM_O1_GPIO13]         = 644,
    [T3_GEM_O1_GPIO14]         = 640,
    [T3_GEM_O1_GPIO15]         = 639,
    [T3_GEM_O1_GPIO16]         = 633,
    [T3_GEM_O1_GPIO17]         = 634,
    [T3_GEM_O1_GPIO18]         = 637,
    [T3_GEM_O1_GPIO19]         = 638,
    [T3_GEM_O1_GPIO20]         = 636,
    [T3_GEM_O1_GPIO21]         = 635,
    [T3_GEM_O1_GPIO22]         = 580,
    [T3_GEM_O1_GPIO23]         = 522,
    [T3_GEM_O1_GPIO24]         = 525,
    [T3_GEM_O1_GPIO25]         = 581,
    [T3_GEM_O1_GPIO26]         = 575,
    [T3_GEM_O1_GPIO27]         = 572,
};

const uint8_t Linux::GPIO_Sysfs::n_pins = _T3_GEM_O1_GPIO_MAX;

static_assert(ARRAY_SIZE(Linux::GPIO_Sysfs::pin_table) == _T3_GEM_O1_GPIO_MAX,
              "GPIO pin_table must have the same size of entries in enum gpio_t3_gem_o1");

#endif
