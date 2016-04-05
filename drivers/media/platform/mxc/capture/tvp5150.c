/*
 * Copyright (C) 2014-2015, Analogue & Micro, Ltd.
 *
 * Adapted for TVP5150 from ADV7180
 * Copyright 2005-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/videodev2.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <linux/of_gpio.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-int-device.h>
#include "mxc_v4l2_capture.h"
#include "tvp5150_reg.h"
#include <media/tvp5150.h>

//#undef dev_dbg
//#define dev_dbg dev_err

static int tvp5150_probe(struct i2c_client *adapter,
			 const struct i2c_device_id *id);
static int tvp5150_detach(struct i2c_client *client);

static const struct i2c_device_id tvp5150_id[] = {
	{"tvp5150", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, tvp5150_id);

static const struct of_device_id tvp5150_dt_ids[] = {
       {
               .compatible = "ti,tvp5150",
       }, {
               /* sentinel */
       }
};
MODULE_DEVICE_TABLE(of, tvp5150_dt_ids);

static struct i2c_driver tvp5150_i2c_driver = {
	.driver = {
                .owner = THIS_MODULE,
                .name = "tvp5150",
                .of_match_table = tvp5150_dt_ids,
        },
	.probe = tvp5150_probe,
	.remove = tvp5150_detach,
	.id_table = tvp5150_id,
};

/*!
 * Maintains the information on the current state of the sensor.
 */
struct tvp5150_priv {
	struct sensor_data sen;
	v4l2_std_id std_id;
#define DVDDIO_REG	0
#define DVDD_REG	1
#define AVDD_REG	2
#define PVDD_REG	3
	struct regulator *regulators[4];
	int norm;
	u32 input;
	u32 output;
	int enable;
	int is_bt656;
#define GPIO_STANDBY		0	/* 1 - powerdown */
#define GPIO_RESET		1	/* 0 - reset */
#define GPIO_CNT		2
	int	gpios[GPIO_CNT];
	enum of_gpio_flags gpio_flags[GPIO_CNT];
};

const char *gpio_names[] = {
        [GPIO_STANDBY] = "pwdn-gpio",
        [GPIO_RESET] = "reset-gpio",
};

static void tvp5150_hard_reset(struct tvp5150_priv *adv);
static int set_power(struct tvp5150_priv *adv, int on);

/*! List of input video formats supported. The video formats is corresponding
 * with v4l2 id in video_fmt_t
 */
typedef enum {
	TVP5150_NTSC = 0,	/*!< Locked on (M) NTSC video signal. */
	TVP5150_PAL,		/*!< (B, G, H, I, N)PAL video signal. */
	TVP5150_NOT_LOCKED,	/*!< Not locked on a signal. */
} video_fmt_idx;

/*! Number of video standards supported (including 'not locked' signal). */
#define TVP5150_STD_MAX		(TVP5150_PAL + 1)

/*! Video format structure. */
typedef struct {
	int v4l2_id;		/*!< Video for linux ID. */
	char name[16];		/*!< Name (e.g., "NTSC", "PAL", etc.) */
	u16 raw_width;		/*!< Raw width. */
	u16 raw_height;		/*!< Raw height. */
	u16 active_width;	/*!< Active width. */
	u16 active_height;	/*!< Active height. */
	int frame_rate;		/*!< Frame rate. */
	u16 lines_per_field;
	u16 skip_lines;
} video_fmt_t;

/*! Description of video formats supported.
 *
 *  PAL: raw=720x625, active=720x576.
 *  NTSC: raw=720x525, active=720x480.
 */
static video_fmt_t video_fmts[] = {
	{			/*! NTSC */
	 .v4l2_id = V4L2_STD_NTSC,
	 .name = "NTSC",
	 .raw_width = 720,	/* SENS_FRM_WIDTH */
	 .raw_height = 525,	/* SENS_FRM_HEIGHT */
	 .active_width = 720,	/* ACT_FRM_WIDTH plus 1 */
	 .active_height = 480,	/* ACT_FRM_WIDTH plus 1 */
         .frame_rate = 30,
         .skip_lines = 3,
         .lines_per_field = 0,
	 },
	{			/*! (B, G, H, I, N) PAL */
	 .v4l2_id = V4L2_STD_PAL,
	 .name = "PAL",
	 .raw_width = 720,
	 .raw_height = 625,
	 .active_width = 720,
	 .active_height = 576,
         .frame_rate = 24,
	 },
	{			/*! Unlocked standard */
	 .v4l2_id = V4L2_STD_ALL,
	 .name = "Autodetect",
	 .raw_width = 720,
	 .raw_height = 625,
	 .active_width = 720,
	 .active_height = 576,
         .frame_rate = 0,
	 },
};

/*!* Standard index of TVP5150. */
static video_fmt_idx video_idx = TVP5150_PAL;

/*! @brief This mutex is used to provide mutual exclusion.
 *
 *  Create a mutex that can be used to provide mutually exclusive
 *  read/write access to the globally accessible data structures
 *  and variables that were defined above.
 */
static DEFINE_MUTEX(mutex);

#define IF_NAME                    "tvp5150"
#define TVP5150_INPUT_CTL              0x00	/* Input Control */
#define TVP5150_STATUS_1               0x10	/* Status #1 */
#define TVP5150_STATUS_2               0x12	/* Status #2 */
#define TVP5150_IDENT                  0x11	/* IDENT */
#define TVP5150_VSYNC_FIELD_CTL_1      0x31	/* VSYNC Field Control #1 */
#define TVP5150_MANUAL_WIN_CTL         0x3d	/* Manual Window Control */
#define TVP5150_PWR_MNG                0x0f     /* Power Management */

/* supported controls */
static struct v4l2_queryctrl tvp5150_qctrl[] = {
	{
                .id = V4L2_CID_BRIGHTNESS,
                .type = V4L2_CTRL_TYPE_INTEGER,
                .name = "Brightness",
                .minimum = 0,
                .maximum = 255,
                .step = 1,
                .default_value = 128,
                .flags = 0,
	}, {
                .id = V4L2_CID_SATURATION,
                .type = V4L2_CTRL_TYPE_INTEGER,
                .name = "Saturation",
                .minimum = 0,
                .maximum = 255,
                .step = 0x1,
                .default_value = 128,
                .flags = 0,
	}, {
                .id = V4L2_CID_CONTRAST,
                .type = V4L2_CTRL_TYPE_INTEGER,
                .name = "Contrast",
                .minimum = 0,
                .maximum = 255,
                .step = 0x1,
                .default_value = 128,
                .flags = 0,
	}, {
                .id = V4L2_CID_HUE,
                .type = V4L2_CTRL_TYPE_INTEGER,
                .name = "Hue",
                .minimum = -128,
                .maximum = 127,
                .step = 0x1,
                .default_value = 0,
                .flags = 0,
	}, {
                .id = V4L2_CID_RED_BALANCE,
                .type = V4L2_CTRL_TYPE_BOOLEAN,
                .name = "Input Selection",
                .default_value = 0,
	}
};

static int tvp5150_gpio_state(struct tvp5150_priv *adv, int index, int active)
{
	if (gpio_is_valid(adv->gpios[index])) {
		int state = (adv->gpio_flags[index] & OF_GPIO_ACTIVE_LOW) ? 1 : 0;

		state ^= active;
		gpio_set_value_cansleep(adv->gpios[index], state);
		pr_debug("%s: active=%d, %s(%d)=0x%x\n", __func__, active, gpio_names[index], adv->gpios[index], state);
		return 0;
	}
	return 1;
}

static int tvp5150_get_gpios(struct tvp5150_priv *adv, struct device *dev)
{
	int i;
	int ret;
	enum of_gpio_flags flags;

	for (i = 0; i < GPIO_CNT; i++) {
		adv->gpios[i] = of_get_named_gpio_flags(dev->of_node, gpio_names[i], 0, &flags);
		if (!gpio_is_valid(adv->gpios[i])) {
			dev_err(dev, "%s: gpio %s not available\n", __func__, gpio_names[i]);
		} else {
			int gflags = (flags & OF_GPIO_ACTIVE_LOW) ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW;

			if ((i == GPIO_STANDBY) || (i == GPIO_RESET))
				gflags ^= GPIOF_INIT_HIGH;
			if (flags == 2)
				gflags = GPIOF_IN;

			adv->gpio_flags[i] = flags;
			dev_dbg(dev, "%s: %s flags(%d -> %d)\n", __func__, gpio_names[i], flags, gflags);
			ret = devm_gpio_request_one(dev, adv->gpios[i], gflags, gpio_names[i]);
			if (ret < 0) {
				pr_info("%s: request of %s failed(%d)\n", __func__, gpio_names[i], ret);
				return ret;
			}
		}
	}
	msleep(7);
//	tvp5150_gpio_state(adv, GPIO_STANDBY, 0);  // power up
//	msleep(4);
	tvp5150_gpio_state(adv, GPIO_RESET, 0);    // remove reset
	msleep(4);
	return 0;
}

/***********************************************************************
 * I2C transfer.
 ***********************************************************************/

/*! Read one register from a TVP5150 i2c slave device.
 *
 *  @param *reg		register in the device we wish to access.
 *
 *  @return		       0 if success, an error code otherwise.
 */
static inline int tvp5150_read(struct tvp5150_priv *adv, u8 reg)
{
	int val;

	val = i2c_smbus_read_byte_data(adv->sen.i2c_client, reg);
	if (val < 0) {
		dev_dbg(&adv->sen.i2c_client->dev,
			"%s:read reg error: reg=%2x\n", __func__, reg);
		return -1;
	}
	return val;
}

/*! Write one register of a TVP5150 i2c slave device.
 *
 *  @param *reg		register in the device we wish to access.
 *
 *  @return		       0 if success, an error code otherwise.
 */
static int tvp5150_write_reg(struct tvp5150_priv *adv, u8 reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(adv->sen.i2c_client, reg, val);
	if (ret < 0) {
		dev_dbg(&adv->sen.i2c_client->dev,
			"%s:write reg error:reg=%2x,val=%2x\n", __func__,
			reg, val);
		return -1;
	}
	return 0;
}

static void dump_reg_range(struct tvp5150_priv *adv, char *s, u8 init,
                           const u8 end, int max_line)
{
	int i = 0;

	while (init != (u8)(end + 1)) {
		if ((i % max_line) == 0) {
			if (i > 0)
				printk("\n");
			printk("tvp5150: %s reg 0x%02x = ", s, init);
		}
		printk("%02x ", tvp5150_read(adv, init));

		init++;
		i++;
	}
	printk("\n");
}

static int tvp5150_log_status(struct tvp5150_priv *adv)
{
	printk("tvp5150: Video input source selection #1 = 0x%02x\n",
			tvp5150_read(adv, TVP5150_VD_IN_SRC_SEL_1));
	printk("tvp5150: Analog channel controls = 0x%02x\n",
			tvp5150_read(adv, TVP5150_ANAL_CHL_CTL));
	printk("tvp5150: Operation mode controls = 0x%02x\n",
			tvp5150_read(adv, TVP5150_OP_MODE_CTL));
	printk("tvp5150: Miscellaneous controls = 0x%02x\n",
			tvp5150_read(adv, TVP5150_MISC_CTL));
	printk("tvp5150: Autoswitch mask= 0x%02x\n",
			tvp5150_read(adv, TVP5150_AUTOSW_MSK));
	printk("tvp5150: Color killer threshold control = 0x%02x\n",
			tvp5150_read(adv, TVP5150_COLOR_KIL_THSH_CTL));
	printk("tvp5150: Luminance processing controls #1 #2 and #3 = %02x %02x %02x\n",
			tvp5150_read(adv, TVP5150_LUMA_PROC_CTL_1),
			tvp5150_read(adv, TVP5150_LUMA_PROC_CTL_2),
			tvp5150_read(adv, TVP5150_LUMA_PROC_CTL_3));
	printk("tvp5150: Brightness control = 0x%02x\n",
			tvp5150_read(adv, TVP5150_BRIGHT_CTL));
	printk("tvp5150: Color saturation control = 0x%02x\n",
			tvp5150_read(adv, TVP5150_SATURATION_CTL));
	printk("tvp5150: Hue control = 0x%02x\n",
			tvp5150_read(adv, TVP5150_HUE_CTL));
	printk("tvp5150: Contrast control = 0x%02x\n",
			tvp5150_read(adv, TVP5150_CONTRAST_CTL));
	printk("tvp5150: Outputs and data rates select = 0x%02x\n",
			tvp5150_read(adv, TVP5150_DATA_RATE_SEL));
	printk("tvp5150: Configuration shared pins = 0x%02x\n",
			tvp5150_read(adv, TVP5150_CONF_SHARED_PIN));
	printk("tvp5150: Active video cropping start = 0x%02x%02x\n",
			tvp5150_read(adv, TVP5150_ACT_VD_CROP_ST_MSB),
			tvp5150_read(adv, TVP5150_ACT_VD_CROP_ST_LSB));
	printk("tvp5150: Active video cropping stop  = 0x%02x%02x\n",
			tvp5150_read(adv, TVP5150_ACT_VD_CROP_STP_MSB),
			tvp5150_read(adv, TVP5150_ACT_VD_CROP_STP_LSB));
	printk("tvp5150: Genlock/RTC = 0x%02x\n",
			tvp5150_read(adv, TVP5150_GENLOCK));
	printk("tvp5150: Horizontal sync start = 0x%02x\n",
			tvp5150_read(adv, TVP5150_HORIZ_SYNC_START));
	printk("tvp5150: Vertical blanking start = 0x%02x\n",
			tvp5150_read(adv, TVP5150_VERT_BLANKING_START));
	printk("tvp5150: Vertical blanking stop = 0x%02x\n",
			tvp5150_read(adv, TVP5150_VERT_BLANKING_STOP));
	printk("tvp5150: Chrominance processing control #1 and #2 = %02x %02x\n",
			tvp5150_read(adv, TVP5150_CHROMA_PROC_CTL_1),
			tvp5150_read(adv, TVP5150_CHROMA_PROC_CTL_2));
	printk("tvp5150: Interrupt reset register B = 0x%02x\n",
			tvp5150_read(adv, TVP5150_INT_RESET_REG_B));
	printk("tvp5150: Interrupt enable register B = 0x%02x\n",
			tvp5150_read(adv, TVP5150_INT_ENABLE_REG_B));
	printk("tvp5150: Interrupt configuration register B = 0x%02x\n",
			tvp5150_read(adv, TVP5150_INTT_CONFIG_REG_B));
	printk("tvp5150: Video standard = 0x%02x\n",
			tvp5150_read(adv, TVP5150_VIDEO_STD));
	printk("tvp5150: Chroma gain factor: Cb=0x%02x Cr=0x%02x\n",
			tvp5150_read(adv, TVP5150_CB_GAIN_FACT),
			tvp5150_read(adv, TVP5150_CR_GAIN_FACTOR));
	printk("tvp5150: Macrovision on counter = 0x%02x\n",
			tvp5150_read(adv, TVP5150_MACROVISION_ON_CTR));
	printk("tvp5150: Macrovision off counter = 0x%02x\n",
			tvp5150_read(adv, TVP5150_MACROVISION_OFF_CTR));
	printk("tvp5150: ITU-R BT.656.%d timing(TVP5150AM1 only)\n",
			(tvp5150_read(adv, TVP5150_REV_SELECT) & 1) ? 3 : 4);
	printk("tvp5150: Device ID = %02x%02x\n",
			tvp5150_read(adv, TVP5150_MSB_DEV_ID),
			tvp5150_read(adv, TVP5150_LSB_DEV_ID));
	printk("tvp5150: ROM version = (hex) %02x.%02x\n",
			tvp5150_read(adv, TVP5150_ROM_MAJOR_VER),
			tvp5150_read(adv, TVP5150_ROM_MINOR_VER));
	printk("tvp5150: Vertical line count = 0x%02x%02x\n",
			tvp5150_read(adv, TVP5150_VERT_LN_COUNT_MSB),
			tvp5150_read(adv, TVP5150_VERT_LN_COUNT_LSB));
	printk("tvp5150: Interrupt status register B = 0x%02x\n",
			tvp5150_read(adv, TVP5150_INT_STATUS_REG_B));
	printk("tvp5150: Interrupt active register B = 0x%02x\n",
			tvp5150_read(adv, TVP5150_INT_ACTIVE_REG_B));
	printk("tvp5150: Status regs #1 to #5 = %02x %02x %02x %02x %02x\n",
			tvp5150_read(adv, TVP5150_STATUS_REG_1),
			tvp5150_read(adv, TVP5150_STATUS_REG_2),
			tvp5150_read(adv, TVP5150_STATUS_REG_3),
			tvp5150_read(adv, TVP5150_STATUS_REG_4),
			tvp5150_read(adv, TVP5150_STATUS_REG_5));

	dump_reg_range(adv, "Teletext filter 1",   TVP5150_TELETEXT_FIL1_INI,
			TVP5150_TELETEXT_FIL1_END, 8);
	dump_reg_range(adv, "Teletext filter 2",   TVP5150_TELETEXT_FIL2_INI,
			TVP5150_TELETEXT_FIL2_END, 8);

	printk("tvp5150: Teletext filter enable = 0x%02x\n",
			tvp5150_read(adv, TVP5150_TELETEXT_FIL_ENA));
	printk("tvp5150: Interrupt status register A = 0x%02x\n",
			tvp5150_read(adv, TVP5150_INT_STATUS_REG_A));
	printk("tvp5150: Interrupt enable register A = 0x%02x\n",
			tvp5150_read(adv, TVP5150_INT_ENABLE_REG_A));
	printk("tvp5150: Interrupt configuration = 0x%02x\n",
			tvp5150_read(adv, TVP5150_INT_CONF));
	printk("tvp5150: VDP status register = 0x%02x\n",
			tvp5150_read(adv, TVP5150_VDP_STATUS_REG));
	printk("tvp5150: FIFO word count = 0x%02x\n",
			tvp5150_read(adv, TVP5150_FIFO_WORD_COUNT));
	printk("tvp5150: FIFO interrupt threshold = 0x%02x\n",
			tvp5150_read(adv, TVP5150_FIFO_INT_THRESHOLD));
	printk("tvp5150: FIFO reset = 0x%02x\n",
			tvp5150_read(adv, TVP5150_FIFO_RESET));
	printk("tvp5150: Line number interrupt = 0x%02x\n",
			tvp5150_read(adv, TVP5150_LINE_NUMBER_INT));
	printk("tvp5150: Pixel alignment register = 0x%02x%02x\n",
			tvp5150_read(adv, TVP5150_PIX_ALIGN_REG_HIGH),
			tvp5150_read(adv, TVP5150_PIX_ALIGN_REG_LOW));
	printk("tvp5150: FIFO output control = 0x%02x\n",
			tvp5150_read(adv, TVP5150_FIFO_OUT_CTRL));
	printk("tvp5150: Full field enable = 0x%02x\n",
			tvp5150_read(adv, TVP5150_FULL_FIELD_ENA));
	printk("tvp5150: Full field mode register = 0x%02x\n",
			tvp5150_read(adv, TVP5150_FULL_FIELD_MODE_REG));

	dump_reg_range(adv, "CC   data",   TVP5150_CC_DATA_INI,
			TVP5150_CC_DATA_END, 8);

	dump_reg_range(adv, "WSS  data",   TVP5150_WSS_DATA_INI,
			TVP5150_WSS_DATA_END, 8);

	dump_reg_range(adv, "VPS  data",   TVP5150_VPS_DATA_INI,
			TVP5150_VPS_DATA_END, 8);

	dump_reg_range(adv, "VITC data",   TVP5150_VITC_DATA_INI,
			TVP5150_VITC_DATA_END, 10);

	dump_reg_range(adv, "Line mode",   TVP5150_LINE_MODE_INI,
			TVP5150_LINE_MODE_END, 8);
	return 0;
}

struct i2c_reg_value {
	unsigned char reg;
	unsigned char value;
};

/* Default values as sugested at TVP5150AM1 datasheet */
static const struct i2c_reg_value tvp5150_init_default[] = {
	{ /* 0x00 */
		TVP5150_VD_IN_SRC_SEL_1,0x00
	},
	{ /* 0x01 */
		TVP5150_ANAL_CHL_CTL,0x15
	},
	{ /* 0x02 */
		TVP5150_OP_MODE_CTL,0x00
	},
	{ /* 0x03 */
		TVP5150_MISC_CTL,0x01
	},
	{ /* 0x06 */
		TVP5150_COLOR_KIL_THSH_CTL,0x10
	},
	{ /* 0x07 */
		TVP5150_LUMA_PROC_CTL_1,0x60
	},
	{ /* 0x08 */
		TVP5150_LUMA_PROC_CTL_2,0x00
	},
	{ /* 0x09 */
		TVP5150_BRIGHT_CTL,0x80
	},
	{ /* 0x0a */
		TVP5150_SATURATION_CTL,0x80
	},
	{ /* 0x0b */
		TVP5150_HUE_CTL,0x00
	},
	{ /* 0x0c */
		TVP5150_CONTRAST_CTL,0x80
	},
	{ /* 0x0d */
		TVP5150_DATA_RATE_SEL,0x40 /* 0x47 */
	},
	{ /* 0x0e */
		TVP5150_LUMA_PROC_CTL_3,0x00
	},
	{ /* 0x0f */
		TVP5150_CONF_SHARED_PIN,0x08
	},
	{ /* 0x11 */
		TVP5150_ACT_VD_CROP_ST_MSB,0x00
	},
	{ /* 0x12 */
		TVP5150_ACT_VD_CROP_ST_LSB,0x00
	},
	{ /* 0x13 */
		TVP5150_ACT_VD_CROP_STP_MSB,0x00
	},
	{ /* 0x14 */
		TVP5150_ACT_VD_CROP_STP_LSB,0x00
	},
	{ /* 0x15 */
		TVP5150_GENLOCK,0x01
	},
	{ /* 0x16 */
		TVP5150_HORIZ_SYNC_START,0x80
	},
	{ /* 0x18 */
		TVP5150_VERT_BLANKING_START,0x00
	},
	{ /* 0x19 */
		TVP5150_VERT_BLANKING_STOP,0x00
	},
	{ /* 0x1a */
		TVP5150_CHROMA_PROC_CTL_1,0x0c
	},
	{ /* 0x1b */
		TVP5150_CHROMA_PROC_CTL_2,0x14
	},
	{ /* 0x1c */
		TVP5150_INT_RESET_REG_B,0x00
	},
	{ /* 0x1d */
		TVP5150_INT_ENABLE_REG_B,0x00
	},
	{ /* 0x1e */
		TVP5150_INTT_CONFIG_REG_B,0x00
	},
	{ /* 0x28 */
		TVP5150_VIDEO_STD,0x00
	},
	{ /* 0x2e */
		TVP5150_MACROVISION_ON_CTR,0x0f
	},
	{ /* 0x2f */
		TVP5150_MACROVISION_OFF_CTR,0x01
	},
	{ /* 0xbb */
		TVP5150_TELETEXT_FIL_ENA,0x00
	},
	{ /* 0xc0 */
		TVP5150_INT_STATUS_REG_A,0x00
	},
	{ /* 0xc1 */
		TVP5150_INT_ENABLE_REG_A,0x00
	},
	{ /* 0xc2 */
		TVP5150_INT_CONF,0x04
	},
	{ /* 0xc8 */
		TVP5150_FIFO_INT_THRESHOLD,0x80
	},
	{ /* 0xc9 */
		TVP5150_FIFO_RESET,0x00
	},
	{ /* 0xca */
		TVP5150_LINE_NUMBER_INT,0x00
	},
	{ /* 0xcb */
		TVP5150_PIX_ALIGN_REG_LOW,0x4e
	},
	{ /* 0xcc */
		TVP5150_PIX_ALIGN_REG_HIGH,0x00
	},
	{ /* 0xcd */
		TVP5150_FIFO_OUT_CTRL,0x01
	},
	{ /* 0xcf */
		TVP5150_FULL_FIELD_ENA,0x00
	},
	{ /* 0xd0 */
		TVP5150_LINE_MODE_INI,0x00
	},
	{ /* 0xfc */
		TVP5150_FULL_FIELD_MODE_REG,0x7f
	},
	{ /* end of data */
		0xff,0xff
	}
};

/* Default values as sugested at TVP5150AM1 datasheet */
static const struct i2c_reg_value tvp5150_init_enable[] = {
	{
		TVP5150_CONF_SHARED_PIN, 2
	},{	/* Automatic offset and AGC enabled */
		TVP5150_ANAL_CHL_CTL, 0x15
	},{	/* Activate YCrCb output 0x9 or 0xd ? */
		TVP5150_MISC_CTL, 0x6f
	},{	/* Activates video std autodetection for all standards */
		TVP5150_AUTOSW_MSK, 0x0
	},{	/* Default format: 0x47. For 4:2:2: 0x40 */
		TVP5150_DATA_RATE_SEL, 0x40 /* 0x47 */
	},{
		TVP5150_CHROMA_PROC_CTL_1, 0x0c
	},{
		TVP5150_CHROMA_PROC_CTL_2, 0x54
	},{	/* Non documented, but initialized on WinTV USB2 */
		0x27, 0x20
	},{
		0xff,0xff
	}
};

struct tvp5150_vbi_type {
	unsigned int vbi_type;
	unsigned int ini_line;
	unsigned int end_line;
	unsigned int by_field :1;
};

struct i2c_vbi_ram_value {
	u16 reg;
	struct tvp5150_vbi_type type;
	unsigned char values[16];
};

/* This struct have the values for each supported VBI Standard
 * by
 tvp5150_vbi_types should follow the same order as vbi_ram_default
 * value 0 means rom position 0x10, value 1 means rom position 0x30
 * and so on. There are 16 possible locations from 0 to 15.
 */

static struct i2c_vbi_ram_value vbi_ram_default[] =
{
	/* FIXME: Current api doesn't handle all VBI types, those not
	   yet supported are placed under #if 0 */
#if 0
	{0x010, /* Teletext, SECAM, WST System A */
		{V4L2_SLICED_TELETEXT_SECAM,6,23,1},
		{ 0xaa, 0xaa, 0xff, 0xff, 0xe7, 0x2e, 0x20, 0x26,
		  0xe6, 0xb4, 0x0e, 0x00, 0x00, 0x00, 0x10, 0x00 }
	},
#endif
	{0x030, /* Teletext, PAL, WST System B */
		{V4L2_SLICED_TELETEXT_B,6,22,1},
		{ 0xaa, 0xaa, 0xff, 0xff, 0x27, 0x2e, 0x20, 0x2b,
		  0xa6, 0x72, 0x10, 0x00, 0x00, 0x00, 0x10, 0x00 }
	},
#if 0
	{0x050, /* Teletext, PAL, WST System C */
		{V4L2_SLICED_TELETEXT_PAL_C,6,22,1},
		{ 0xaa, 0xaa, 0xff, 0xff, 0xe7, 0x2e, 0x20, 0x22,
		  0xa6, 0x98, 0x0d, 0x00, 0x00, 0x00, 0x10, 0x00 }
	},
	{0x070, /* Teletext, NTSC, WST System B */
		{V4L2_SLICED_TELETEXT_NTSC_B,10,21,1},
		{ 0xaa, 0xaa, 0xff, 0xff, 0x27, 0x2e, 0x20, 0x23,
		  0x69, 0x93, 0x0d, 0x00, 0x00, 0x00, 0x10, 0x00 }
	},
	{0x090, /* Tetetext, NTSC NABTS System C */
		{V4L2_SLICED_TELETEXT_NTSC_C,10,21,1},
		{ 0xaa, 0xaa, 0xff, 0xff, 0xe7, 0x2e, 0x20, 0x22,
		  0x69, 0x93, 0x0d, 0x00, 0x00, 0x00, 0x15, 0x00 }
	},
	{0x0b0, /* Teletext, NTSC-J, NABTS System D */
		{V4L2_SLICED_TELETEXT_NTSC_D,10,21,1},
		{ 0xaa, 0xaa, 0xff, 0xff, 0xa7, 0x2e, 0x20, 0x23,
		  0x69, 0x93, 0x0d, 0x00, 0x00, 0x00, 0x10, 0x00 }
	},
	{0x0d0, /* Closed Caption, PAL/SECAM */
		{V4L2_SLICED_CAPTION_625,22,22,1},
		{ 0xaa, 0x2a, 0xff, 0x3f, 0x04, 0x51, 0x6e, 0x02,
		  0xa6, 0x7b, 0x09, 0x00, 0x00, 0x00, 0x27, 0x00 }
	},
#endif
	{0x0f0, /* Closed Caption, NTSC */
		{V4L2_SLICED_CAPTION_525,21,21,1},
		{ 0xaa, 0x2a, 0xff, 0x3f, 0x04, 0x51, 0x6e, 0x02,
		  0x69, 0x8c, 0x09, 0x00, 0x00, 0x00, 0x27, 0x00 }
	},
	{0x110, /* Wide Screen Signal, PAL/SECAM */
		{V4L2_SLICED_WSS_625,23,23,1},
		{ 0x5b, 0x55, 0xc5, 0xff, 0x00, 0x71, 0x6e, 0x42,
		  0xa6, 0xcd, 0x0f, 0x00, 0x00, 0x00, 0x3a, 0x00 }
	},
#if 0
	{0x130, /* Wide Screen Signal, NTSC C */
		{V4L2_SLICED_WSS_525,20,20,1},
		{ 0x38, 0x00, 0x3f, 0x00, 0x00, 0x71, 0x6e, 0x43,
		  0x69, 0x7c, 0x08, 0x00, 0x00, 0x00, 0x39, 0x00 }
	},
	{0x150, /* Vertical Interval Timecode (VITC), PAL/SECAM */
		{V4l2_SLICED_VITC_625,6,22,0},
		{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x8f, 0x6d, 0x49,
		  0xa6, 0x85, 0x08, 0x00, 0x00, 0x00, 0x4c, 0x00 }
	},
	{0x170, /* Vertical Interval Timecode (VITC), NTSC */
		{V4l2_SLICED_VITC_525,10,20,0},
		{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x8f, 0x6d, 0x49,
		  0x69, 0x94, 0x08, 0x00, 0x00, 0x00, 0x4c, 0x00 }
	},
#endif
	{0x190, /* Video Program System (VPS), PAL */
		{V4L2_SLICED_VPS,16,16,0},
		{ 0xaa, 0xaa, 0xff, 0xff, 0xba, 0xce, 0x2b, 0x0d,
		  0xa6, 0xda, 0x0b, 0x00, 0x00, 0x00, 0x60, 0x00 }
	},
	/* 0x1d0 User programmable */

	/* End of struct */
	{ (u16)-1 }
};

static inline void tvp5150_selmux(struct tvp5150_priv *adv)
{
	int opmode = 0;
	int input = 0;
	unsigned char val;

	if ((adv->output & TVP5150_BLACK_SCREEN) || !adv->enable)
		input = 8;

	switch (adv->input) {
	case TVP5150_COMPOSITE1:
		input |= 2;
		/* fall through */
	case TVP5150_COMPOSITE0:
		break;
	case TVP5150_SVIDEO:
	default:
		input |= 1;
		break;
	}

	dev_dbg(&adv->sen.i2c_client->dev, "Selecting video route: route input=%i, output=%i "
               "=> tvp5150 input=%i, opmode=%i\n",
               adv->input, adv->output,
               input, opmode);

	tvp5150_write_reg(adv, TVP5150_OP_MODE_CTL, opmode);
	tvp5150_write_reg(adv, TVP5150_VD_IN_SRC_SEL_1, input);

	/* Svideo should enable YCrCb output and disable GPCL output
	 * For Composite and TV, it should be the reverse
	 */
	val = tvp5150_read(adv, TVP5150_MISC_CTL);
	if (adv->input == TVP5150_SVIDEO)
		val = (val & ~0x40) | 0x10;
	else
		val = (val & ~0x10) | 0x40;
	tvp5150_write_reg(adv, TVP5150_MISC_CTL, val);
};

#if 0 // Not used yet
static int tvp5150_s_routing(struct tvp5150_priv *adv,
			     u32 input, u32 output, u32 config)
{
	adv->input = input;
	adv->output = output;
	tvp5150_selmux(adv);
	return 0;
}
#endif

static int tvp5150_write_inittab(struct tvp5150_priv *adv,
                                 const struct i2c_reg_value *regs)
{
	while (regs->reg != 0xff) {
		tvp5150_write_reg(adv, regs->reg, regs->value);
		regs++;
	}
	return 0;
}

static int tvp5150_vdp_init(struct tvp5150_priv *adv,
                            const struct i2c_vbi_ram_value *regs)
{
	unsigned int i;

	/* Disable Full Field */
	tvp5150_write_reg(adv, TVP5150_FULL_FIELD_ENA, 0);

	/* Before programming, Line mode should be at 0xff */
	for (i = TVP5150_LINE_MODE_INI; i <= TVP5150_LINE_MODE_END; i++)
		tvp5150_write_reg(adv, i, 0xff);

	/* Load Ram Table */
	while (regs->reg != (u16)-1) {
		tvp5150_write_reg(adv, TVP5150_CONF_RAM_ADDR_HIGH, regs->reg >> 8);
		tvp5150_write_reg(adv, TVP5150_CONF_RAM_ADDR_LOW, regs->reg);

		for (i = 0; i < 16; i++)
			tvp5150_write_reg(adv, TVP5150_VDP_CONF_RAM_DATA, regs->values[i]);

		regs++;
	}
	return 0;
}

/***********************************************************************
 * mxc_v4l2_capture interface.
 ***********************************************************************/

void get_std(struct tvp5150_priv *adv)
{
	int idx;
	v4l2_std_id std_id;
	u8 std, std_status;
        int lines;
        u8 reg;

	dev_dbg(&adv->sen.i2c_client->dev, "In tvp5150_get_std\n");

	mutex_lock(&mutex);
	std = tvp5150_read(adv, TVP5150_VIDEO_STD);
        reg = tvp5150_read(adv, TVP5150_VERT_LN_COUNT_MSB);
        lines = reg << 8;
        reg = tvp5150_read(adv, TVP5150_VERT_LN_COUNT_LSB);
        lines |= reg;

	if ((std & VIDEO_STD_MASK) == VIDEO_STD_AUTO_SWITCH_BIT)
		/* use the standard status register */
		std_status = tvp5150_read(adv, TVP5150_STATUS_REG_5);
	else
		/* use the standard register itself */
		std_status = std;

	switch (std_status & VIDEO_STD_MASK) {
	case VIDEO_STD_NTSC_MJ_BIT:
	case VIDEO_STD_NTSC_MJ_BIT_AS:
                std_id = V4L2_STD_NTSC;
                idx = TVP5150_NTSC;
                break;

	case VIDEO_STD_PAL_BDGHIN_BIT:
	case VIDEO_STD_PAL_BDGHIN_BIT_AS:
                std_id = V4L2_STD_PAL;
                idx = TVP5150_PAL;
                break;

	default:
                printk("%s - unknown std = 0x%02x/0x%02x, lines = %d\n", __FUNCTION__, std, std_status, lines);
                std_id = V4L2_STD_PAL;
                idx = TVP5150_PAL;
                break;
	}

        dev_dbg(&adv->sen.i2c_client->dev, "%s - std = 0x%02x/0x%02x, lines = %d, idx = %d\n", __FUNCTION__, std, std_status, lines, idx);

	if (adv->std_id != std_id) {
		adv->std_id = std_id;
		video_idx = idx;
		adv->sen.pix.width = video_fmts[idx].active_width;
		adv->sen.pix.height = video_fmts[idx].active_height;
		adv->sen.spix.swidth = video_fmts[idx].raw_width - 1;
		adv->sen.spix.sheight = video_fmts[idx].raw_height;
		adv->sen.spix.top = video_fmts[idx].skip_lines;
                adv->sen.spix.swidth = video_fmts[idx].active_width;
                adv->sen.spix.left = video_fmts[idx].lines_per_field;
                dev_dbg(&adv->sen.i2c_client->dev, "%s.%d - sensor = %dx%d, %dx%d+%dx%d\n", __FUNCTION__, __LINE__,
                        adv->sen.pix.width, adv->sen.pix.height, 
                        adv->sen.spix.swidth, adv->sen.spix.sheight, adv->sen.spix.top, adv->sen.spix.left);
	}
	mutex_unlock(&mutex);
}

/*!
 * Return attributes of current video standard.
 * Since this device autodetects the current standard, this function also
 * sets the values that need to be changed if the standard changes.
 * There is no set std equivalent function.
 *
 *  @return		None.
 */
static void tvp5150_get_std(struct tvp5150_priv *adv, v4l2_std_id *std)
{
	dev_dbg(&adv->sen.i2c_client->dev, "In tvp5150_get_std\n");

	/* Make sure power is on */
	set_power(adv, 1);
	if (adv->std_id == V4L2_STD_ALL)
		get_std(adv);

	*std = adv->std_id;
}

/***********************************************************************
 * IOCTL Functions from v4l2_int_ioctl_desc.
 ***********************************************************************/

/*!
 * ioctl_g_ifparm - V4L2 sensor interface handler for vidioc_int_g_ifparm_num
 * s: pointer to standard V4L2 device structure
 * p: pointer to standard V4L2 vidioc_int_g_ifparm_num ioctl structure
 *
 * Gets slave interface parameters.
 * Calculates the required xclk value to support the requested
 * clock parameters in p.  This value is returned in the p
 * parameter.
 *
 * vidioc_int_g_ifparm returns platform-specific information about the
 * interface settings used by the sensor.
 *
 * Called on open.
 */
static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	struct tvp5150_priv *adv = s->priv;
	dev_dbg(&adv->sen.i2c_client->dev, "tvp5150:ioctl_g_ifparm\n");

	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	/* Initialize structure to 0s then set any non-0 values. */
	memset(p, 0, sizeof(*p));
	p->if_type = V4L2_IF_TYPE_BT656_INTERLACED; /* This is the only possibility. */
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	//p->u.bt656.nobt_hs_inv = 1;
	//p->u.bt656.bt_sync_correct = 1;

	/* TVP5150 has a dedicated clock so no clock settings needed. */

	return 0;
}

/*!
 * Sets the camera power.
 *
 * s  pointer to the camera device
 * on if 1, power is to be turned on.  0 means power is to be turned off
 *
 * ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 * This is called on open, close, suspend and resume.
 */
static int set_power(struct tvp5150_priv *adv, int on)
{
	dev_dbg(&adv->sen.i2c_client->dev, "tvp5150:ioctl_s_power\n");

	if (on != adv->sen.on) {
		if (on) {
                        tvp5150_gpio_state(adv, GPIO_STANDBY, 0);  // power up

			if (tvp5150_write_reg(adv, TVP5150_PWR_MNG, 0x04) != 0)
				return -EIO;

			/*! TVP5150 initialization. */
			tvp5150_hard_reset(adv);
			/*
			 * Wait for video format detection to be stable
			 */
			msleep(400);
			get_std(adv);
		} else {
			if (tvp5150_write_reg(adv, TVP5150_PWR_MNG, 0x24) != 0)
				return -EIO;

                        tvp5150_gpio_state(adv, GPIO_STANDBY, 1);  // power down
		}
		adv->sen.on = on;
	}
	return 0;
}

static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	return set_power(s->priv, on);
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct tvp5150_priv *adv = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;

	dev_dbg(&adv->sen.i2c_client->dev, "In tvp5150:ioctl_g_parm\n");

	switch (a->type) {
	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = adv->sen.streamcap.capability;
		cparm->timeperframe = adv->sen.streamcap.timeperframe;
		cparm->capturemode = adv->sen.streamcap.capturemode;
		break;

	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		break;

	default:
		pr_debug("ioctl_g_parm:type is unknown %d\n", a->type);
		break;
	}

	return 0;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 *
 * This driver cannot change these settings.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct tvp5150_priv *adv = s->priv;
	dev_dbg(&adv->sen.i2c_client->dev, "In tvp5150:ioctl_s_parm\n");

	switch (a->type) {
	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		break;
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
	default:
		pr_debug("   type is unknown - %d\n", a->type);
		return -EINVAL;
	}

	return 0;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct tvp5150_priv *adv = s->priv;
        v4l2_std_id std;

	dev_dbg(&adv->sen.i2c_client->dev, "tvp5150:ioctl_g_fmt_cap\n");

        tvp5150_get_std(adv, &std);
	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   Returning size of %dx%d\n",
			 adv->sen.pix.width, adv->sen.pix.height);
		f->fmt.pix = adv->sen.pix;
                dev_dbg(&adv->sen.i2c_client->dev, "%s.%d - sensor = %dx%d, %dx%d+%dx%d\n", __FUNCTION__, __LINE__,
                        adv->sen.pix.width, adv->sen.pix.height, 
                        adv->sen.spix.swidth, adv->sen.spix.sheight, adv->sen.spix.top, adv->sen.spix.left);
		break;

	case V4L2_BUF_TYPE_PRIVATE:
		f->fmt.pix.pixelformat = (u32)std;
		break;

	default:
		f->fmt.pix = adv->sen.pix;
		f->fmt.spix = adv->sen.spix;
		break;
	}

	return 0;
}

/*!
 * ioctl_queryctrl - V4L2 sensor interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the video_control[] array.  Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int ioctl_queryctrl(struct v4l2_int_device *s,
			   struct v4l2_queryctrl *qc)
{
	struct tvp5150_priv *adv = s->priv;
	int i;

	dev_dbg(&adv->sen.i2c_client->dev, "tvp5150:ioctl_queryctrl\n");

	for (i = 0; i < ARRAY_SIZE(tvp5150_qctrl); i++)
		if (qc->id && qc->id == tvp5150_qctrl[i].id) {
			memcpy(qc, &(tvp5150_qctrl[i]),
				sizeof(*qc));
			return 0;
		}

	return -EINVAL;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	struct tvp5150_priv *adv = s->priv;
	int ret = 0;

	dev_dbg(&adv->sen.i2c_client->dev, "In tvp5150:ioctl_g_ctrl\n");

	/* Make sure power on */
	set_power(adv, 1);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_BRIGHTNESS\n");
		adv->sen.brightness = tvp5150_read(adv, TVP5150_BRIGHT_CTL);
		vc->value = adv->sen.brightness;
		break;
	case V4L2_CID_CONTRAST:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_CONTRAST\n");
		adv->sen.contrast = tvp5150_read(adv, TVP5150_CONTRAST_CTL);
		vc->value = adv->sen.contrast;
		break;
	case V4L2_CID_SATURATION:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_SATURATION\n");
		adv->sen.saturation = tvp5150_read(adv, TVP5150_SATURATION_CTL);
		vc->value = adv->sen.saturation;
		break;
	case V4L2_CID_HUE:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_HUE\n");
		adv->sen.hue = tvp5150_read(adv, TVP5150_HUE_CTL);
		vc->value = adv->sen.hue;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_AUTO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_DO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_RED_BALANCE:
                /* Used to control input selection */
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_RED_BALANCE\n");
                if (adv->input == TVP5150_COMPOSITE0) {
                        vc->value = 0;
                } else {
                        vc->value = 1;
                }
		break;
	case V4L2_CID_BLUE_BALANCE:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_BLUE_BALANCE\n");
		vc->value = adv->sen.blue;
		break;
	case V4L2_CID_GAMMA:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_GAMMA\n");
		break;
	case V4L2_CID_EXPOSURE:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_EXPOSURE\n");
		vc->value = adv->sen.ae_mode;
		break;
	case V4L2_CID_AUTOGAIN:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_AUTOGAIN\n");
		break;
	case V4L2_CID_GAIN:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_GAIN\n");
		break;
	case V4L2_CID_HFLIP:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_HFLIP\n");
		break;
	case V4L2_CID_VFLIP:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_VFLIP\n");
		break;
	default:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   Default case\n");
		vc->value = 0;
		ret = -EPERM;
		break;
	}

	return ret;
}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	struct tvp5150_priv *adv = s->priv;
	int retval = 0;
	u8 tmp;
        s8 hue_tmp;

	dev_dbg(&adv->sen.i2c_client->dev, "In tvp5150:ioctl_s_ctrl\n");

	/* Make sure power on */
	set_power(adv, 1);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_BRIGHTNESS\n");
		tmp = vc->value;
		tvp5150_write_reg(adv, TVP5150_BRIGHT_CTL, tmp);
		adv->sen.brightness = vc->value;
		break;
	case V4L2_CID_CONTRAST:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_CONTRAST\n");
		tmp = vc->value;
		tvp5150_write_reg(adv, TVP5150_CONTRAST_CTL, tmp);
		adv->sen.contrast = vc->value;
		break;
	case V4L2_CID_SATURATION:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_SATURATION\n");
		tmp = vc->value;
		tvp5150_write_reg(adv, TVP5150_SATURATION_CTL, tmp);
		adv->sen.saturation = vc->value;
		break;
	case V4L2_CID_HUE:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_HUE\n");
		hue_tmp = vc->value;
		tvp5150_write_reg(adv, TVP5150_HUE_CTL, hue_tmp);
		adv->sen.hue = vc->value;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_AUTO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_DO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_RED_BALANCE:
                /* Used to control input selection */
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_RED_BALANCE\n");
                if (vc->value == 0) {
                        adv->input = TVP5150_COMPOSITE0;
                } else {
                        adv->input = TVP5150_COMPOSITE1;
                }
                tvp5150_selmux(adv);
		break;
	case V4L2_CID_BLUE_BALANCE:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_BLUE_BALANCE\n");
		break;
	case V4L2_CID_GAMMA:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_GAMMA\n");
		break;
	case V4L2_CID_EXPOSURE:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_EXPOSURE\n");
		break;
	case V4L2_CID_AUTOGAIN:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_AUTOGAIN\n");
		break;
	case V4L2_CID_GAIN:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_GAIN\n");
		break;
	case V4L2_CID_HFLIP:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_HFLIP\n");
		break;
	case V4L2_CID_VFLIP:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   V4L2_CID_VFLIP\n");
		break;
	default:
		dev_dbg(&adv->sen.i2c_client->dev,
			"   Default case\n");
		retval = -EPERM;
		break;
	}

	return retval;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	struct tvp5150_priv *adv = s->priv;

	if (fsize->index > TVP5150_STD_MAX)
		return -EINVAL;

	fsize->pixel_format = adv->sen.pix.pixelformat;
	fsize->discrete.width = video_fmts[fsize->index].active_width;
	fsize->discrete.height = video_fmts[fsize->index].active_height;
	return 0;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name,
						"tvp5150_decoder");
	((struct v4l2_dbg_chip_ident *)id)->ident = V4L2_IDENT_TVP5150;

	return 0;
}

/*!
 * ioctl_enum_fmt_cap - V4L2 sensor interface handler for VIDIOC_ENUM_FMT
 * @s: pointer to standard V4L2 device structure
 * @fmt: pointer to standard V4L2 fmt description structure
 *
 * Return 0.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
			      struct v4l2_fmtdesc *fmt)
{
	struct tvp5150_priv *adv = s->priv;

	if (fmt->index > TVP5150_STD_MAX)
		return -EINVAL;
	fmt->pixelformat = adv->sen.pix.pixelformat;
	return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	struct tvp5150_priv *adv = s->priv;

	dev_dbg(&adv->sen.i2c_client->dev, "In tvp5150:ioctl_init\n");
	return 0;
}

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct tvp5150_priv *adv = s->priv;

	dev_dbg(&adv->sen.i2c_client->dev, "tvp5150:ioctl_dev_init\n");
        dev_dbg(&adv->sen.i2c_client->dev, "%s.%d - sensor = %dx%d, %dx%d+%dx%d\n", __FUNCTION__, __LINE__,
                adv->sen.pix.width, adv->sen.pix.height, 
                adv->sen.spix.swidth, adv->sen.spix.sheight, adv->sen.spix.top, adv->sen.spix.left);
	return 0;
}

/*!
 * This structure defines all the ioctls for this module.
 */
static struct v4l2_int_ioctl_desc tvp5150_ioctl_desc[] = {

	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func*)ioctl_dev_init},

	/*!
	 * Delinitialise the dev. at slave detach.
	 * The complement of ioctl_dev_init.
	 */
/*	{vidioc_int_dev_exit_num, (v4l2_int_ioctl_func *)ioctl_dev_exit}, */

	{vidioc_int_s_power_num, (v4l2_int_ioctl_func*)ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func*)ioctl_g_ifparm},
/*	{vidioc_int_g_needs_reset_num,
				(v4l2_int_ioctl_func *)ioctl_g_needs_reset}, */
/*	{vidioc_int_reset_num, (v4l2_int_ioctl_func *)ioctl_reset}, */
	{vidioc_int_init_num, (v4l2_int_ioctl_func*)ioctl_init},

	/*!
	 * VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
	 */
	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_enum_fmt_cap},

	/*!
	 * VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.
	 * This ioctl is used to negotiate the image capture size and
	 * pixel format without actually making it take effect.
	 */
/*	{vidioc_int_try_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_try_fmt_cap}, */

	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func*)ioctl_g_fmt_cap},

	/*!
	 * If the requested format is supported, configures the HW to use that
	 * format, returns error code if format not supported or HW can't be
	 * correctly configured.
	 */
/*	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_s_fmt_cap}, */

	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func*)ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func*)ioctl_s_parm},
	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func*)ioctl_queryctrl},
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func*)ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func*)ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *)ioctl_enum_framesizes},
	{vidioc_int_g_chip_ident_num,
				(v4l2_int_ioctl_func *)ioctl_g_chip_ident},
};

static struct v4l2_int_slave tvp5150_slave = {
	.ioctls = tvp5150_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(tvp5150_ioctl_desc),
};

static struct v4l2_int_device tvp5150_int_device = {
	.module = THIS_MODULE,
	.name = "tvp5150",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &tvp5150_slave,
	},
};

/***********************************************************************
 * I2C client and driver.
 ***********************************************************************/

/*! TVP5150 Reset function.
 *
 *  @return		None.
 */
static void tvp5150_hard_reset(struct tvp5150_priv *adv)
{
	dev_dbg(&adv->sen.i2c_client->dev,
		"In tvp5150:tvp5150_hard_reset\n");

	/* Initializes TVP5150 to its default values */
	tvp5150_write_inittab(adv, tvp5150_init_default);

	/* Initializes VDP registers */
	tvp5150_vdp_init(adv, vbi_ram_default);

	/* Selects decoder input */
	tvp5150_selmux(adv);

	/* Initializes TVP5150 to stream enabled values */
	tvp5150_write_inittab(adv, tvp5150_init_enable);

        /* Select output format */
        if (adv->is_bt656) {
                tvp5150_write_reg(adv, TVP5150_DATA_RATE_SEL, 0x47);
        } else {
                tvp5150_write_reg(adv, TVP5150_DATA_RATE_SEL, 0x40);
        }

	/* Initialize image preferences */
//??	v4l2_ctrl_handler_setup(&decoder->hdl);

//??	tvp5150_set_std(adv, adv->norm);
        if (0)
                tvp5150_log_status(adv);
}

/*! TVP5150 I2C attach function.
 *
 *  @param *adapter	struct i2c_adapter *.
 *
 *  @return		Error code indicating success or failure.
 */

/*!
 * TVP5150 I2C probe function.
 * Function set in i2c_driver struct.
 * Called by insmod.
 *
 *  @param *adapter	I2C adapter descriptor.
 *
 *  @return		Error code indicating success or failure.
 */
static int tvp5150_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct tvp5150_priv *adv;
        int msb_id, lsb_id, msb_rom, lsb_rom;
	int ret = 0;
	struct device *dev = &client->dev;

	pr_debug("In tvp5150_probe\n");

	adv = kzalloc(sizeof(struct tvp5150_priv), GFP_KERNEL);
	if (!adv)
		return -ENOMEM;

	ret = tvp5150_get_gpios(adv, dev);
	if (ret)
		return -ENODEV;

	ret = of_property_read_u32(dev->of_node, "ipu_id", &adv->sen.ipu_id);
	if (ret) {
		dev_err(dev, "ipu_id missing or invalid\n");
		return -ENODEV;
	}
	ret = of_property_read_u32(dev->of_node, "csi_id", &(adv->sen.csi));
	if (ret) {
		dev_err(dev, "csi_id invalid\n");
		return -ENODEV;
	}
	ret = of_property_read_u32(dev->of_node, "is-bt656", &(adv->is_bt656));
	if (ret) {
                adv->is_bt656 = 0;
	}
	adv->sen.i2c_client = client;
	adv->sen.streamcap.timeperframe.denominator = 30;
	adv->sen.streamcap.timeperframe.numerator = 1;
	adv->std_id = V4L2_STD_ALL;
	adv->norm = V4L2_STD_ALL;	/* Default is autodetect */
	adv->input = TVP5150_COMPOSITE0;
	adv->enable = 1;
	video_idx = TVP5150_NOT_LOCKED;
	adv->sen.pix.width = video_fmts[video_idx].raw_width;
	adv->sen.pix.height = video_fmts[video_idx].raw_height;
        adv->sen.spix.swidth = video_fmts[video_idx].raw_width - 1;
        adv->sen.spix.sheight = video_fmts[video_idx].raw_height;
        adv->sen.spix.top = video_fmts[video_idx].skip_lines;
        adv->sen.spix.swidth = video_fmts[video_idx].active_width;
        adv->sen.spix.left = video_fmts[video_idx].lines_per_field;
	adv->sen.pix.pixelformat = V4L2_PIX_FMT_UYVY;  /* YUV422 */
	adv->sen.pix.priv = 1;  /* 1 is used to indicate TV in */
	adv->sen.on = true;
        dev_dbg(&adv->sen.i2c_client->dev, "%s.%d - sensor = %dx%d, %dx%d+%dx%d\n", __FUNCTION__, __LINE__,
                adv->sen.pix.width, adv->sen.pix.height, 
                adv->sen.spix.swidth, adv->sen.spix.sheight, adv->sen.spix.top, adv->sen.spix.left);

	dev_dbg(&adv->sen.i2c_client->dev,
		"%s:tvp5150 probe i2c address is 0x%02X\n",
		__func__, adv->sen.i2c_client->addr);

	/*! Read the revision ID of the tvin chip */
	msb_id = tvp5150_read(adv, TVP5150_MSB_DEV_ID);
	lsb_id = tvp5150_read(adv, TVP5150_LSB_DEV_ID);
	msb_rom = tvp5150_read(adv, TVP5150_ROM_MAJOR_VER);
	lsb_rom = tvp5150_read(adv, TVP5150_ROM_MINOR_VER);

	if (msb_rom == 4 && lsb_rom == 0) { /* Is TVP5150AM1 */
		pr_info("tvp%02x%02xam1 detected.\n", msb_id, lsb_id);
		/* ITU-T BT.656.4 timing */
		tvp5150_write_reg(adv, TVP5150_REV_SELECT, 0);
	} else {
		if (msb_rom == 3 || lsb_rom == 0x21) { /* Is TVP5150A */
			pr_info("tvp%02x%02xa detected.\n", msb_id, lsb_id);
		} else {
			pr_err("*** unknown tvp%02x%02x chip detected.\n",
					msb_id, lsb_id);
			pr_err("*** Rom ver is %d.%d\n", msb_rom, lsb_rom);
                        return -ENODEV;
		}
	}

	set_power(adv, 0);

	/*
	 * This function attaches this structure to the /dev/video0 device.
	 */
	tvp5150_int_device.priv = adv;
	i2c_set_clientdata(client, adv);
	ret = v4l2_int_device_register(&tvp5150_int_device);

	return ret;
}

/*!
 * TVP5150 I2C detach function.
 * Called on rmmod.
 *
 *  @param *client	struct i2c_client*.
 *
 *  @return		Error code indicating success or failure.
 */
static int tvp5150_detach(struct i2c_client *client)
{
	struct tvp5150_priv *adv = i2c_get_clientdata(client);
	int i;

	if (!adv)
		return 0;
	dev_dbg(&adv->sen.i2c_client->dev,
		"%s:Removing %s video decoder @ 0x%02X from adapter %s\n",
		__func__, IF_NAME, client->addr << 1, client->adapter->name);

	set_power(adv, 0);

	for (i = 0; i < ARRAY_SIZE(adv->regulators); i++) {
		struct regulator *reg = adv->regulators[i];

		if (reg) {
			regulator_disable(reg);
			regulator_put(reg);
		}
	}

	v4l2_int_device_unregister(&tvp5150_int_device);
	kfree(adv);
	return 0;
}

/*!
 * TVP5150 init function.
 * Called on insmod.
 *
 * @return    Error code indicating success or failure.
 */
static __init int tvp5150_init(void)
{
	u8 err = 0;

	pr_debug("In tvp5150_init\n");

	/* Tells the i2c driver what functions to call for this driver. */
	err = i2c_add_driver(&tvp5150_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d \n",
			__func__, err);

	return err;
}

/*!
 * TVP5150 cleanup function.
 * Called on rmmod.
 *
 * @return   Error code indicating success or failure.
 */
static void __exit tvp5150_clean(void)
{
	pr_debug("In tvp5150_clean\n");
	i2c_del_driver(&tvp5150_i2c_driver);
}

module_init(tvp5150_init);
module_exit(tvp5150_clean);

MODULE_AUTHOR("Analogue & Micro, Ltd");
MODULE_DESCRIPTION("Texas Instruments TVP5150 video decoder driver");
MODULE_LICENSE("GPL");
