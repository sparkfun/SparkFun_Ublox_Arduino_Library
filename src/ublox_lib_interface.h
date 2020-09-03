#include <zephyr.h>

#ifndef _UBLOX_LIB_INTERFACE_H_
#define _UBLOX_LIB_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

    uint8_t set_gpio_dev(struct device *gpio_dev);

    uint8_t gps_begin(struct device *i2c_dev);
    void check_ublox(void);
    void get_position(void);
    void get_datetime(void);

#ifdef __cplusplus
}
#endif

#endif  // _UBLOX_LIB_INTERFACE_H_