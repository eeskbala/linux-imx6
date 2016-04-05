#ifndef __LINUX_I2C_SSD253X_H
#define __LINUX_I2C_SSD253X_H

/* linux/i2c/ssd253x-ts.h */

struct ssd253x_platform_data {
        int gpio_irq;
        int gpio_reset;
};

#endif
