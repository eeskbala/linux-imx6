/*
 * Handle boot command line choice of touch screen
 */

#include <linux/types.h>
#include <linux/string.h>
#include <linux/printk.h>
#include <linux/init.h>

char ts_select[64] = "none";

static int __init touch_setup(char *choice)
{
        printk("#### %s(%s)\n", __FUNCTION__, choice);
        strcpy(ts_select, choice);
#if defined(CONFIG_TOUCHSCREEN_SSD253X)
        if (strcmp(choice, "ssd253x") == 0) {
                return 0;
        }
#endif
#if defined(CONFIG_TOUCHSCREEN_GT9XX)
        if (strcmp(choice, "gt9xx") == 0) {
                return 0;
        }
#endif
        printk("??? Invalid touch screen choice: %s\n", choice);
        return 1;
}
early_param("touch", touch_setup);
