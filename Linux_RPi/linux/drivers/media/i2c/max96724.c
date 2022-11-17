// SPDX-License-Identifier: GPL-2.0+
/*
 * Maxim MAX96724 Quad GMSL2/1 Deserializer Driver
 *
 * Copyright (C) 2022 Analog Devices, Inc.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/kdev_t.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/videodev2.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

// References:
// https://datasheets.maximintegrated.com/en/ds/MAX96724-MAX96724R.pdf
// https://github.com/raspberrypi/linux/blob/rpi-5.4.y/drivers/media/i2c/imx219.c


// register addresses
typedef enum
{
	REG_PCLK_FREQ               = 0x0009,
	REG_REG13                   = 0x000d,
	REG_PWR1                    = 0x0013,
	REG_VPRBS                   = 0x021c,
	REG_BACKTOP12               = 0x040b,
	REG_MIPI_PHY0               = 0x08a0,
	REG_MIPI_PHY2               = 0x08a2,
	REG_MIPI_PHY4               = 0x08a4,
	REG_MIPI_TX10_OF_MIPI_TX2   = 0x098a,
	REG_PATGEN_0                = 0x1050,
	REG_PATGEN_1                = 0x1051,
	REG_VS_DLY_2                = 0x1052,
	REG_VS_HIGH_2               = 0x1055,
	REG_VS_LOW_2                = 0x1058,
	REG_V2H_2                   = 0x105b,
	REG_HS_HIGH_1               = 0x105e,
	REG_HS_LOW_1                = 0x1060,
	REG_HS_CNT_1                = 0x1062,
	REG_V2D_2                   = 0x1064,
	REG_DE_HIGH_1               = 0x1067,
	REG_DE_LOW_1                = 0x1069,
	REG_DE_CNT_1                = 0x106b,
	REG_GRAD_INCR               = 0x106d
}Reg;

// see t_LOCK2 in datasheet (45-60ms)
static const uint16_t MAX96724_MIN_DELAY_US   = 45000;
static const uint16_t MAX96724_DELAY_RANGE_US = 15000;

struct i2c_client* i2c_client_cmd = NULL;

static const uint16_t FHD_WIDTH = 1920;
static const uint16_t FHD_HEIGHT = 1080;

typedef enum
{
	VIDEO_FMT_CODE_RGB888,
	VIDEO_FMT_CODE_RAW10
}VideoFmtCode;

uint8_t video_fmt_code_index = VIDEO_FMT_CODE_RGB888;

static const uint32_t video_fmt_codes[] =
{
	// https://elixir.bootlin.com/linux/v5.4.83/source/include/uapi/linux/media-bus-format.h
	MEDIA_BUS_FMT_RGB888_1X24,  // native for pixel mode
	MEDIA_BUS_FMT_SGRBG10_1X10  // BA10 (10-bit Bayer GRGR/BGBG), for camera AR0234
};

struct max96724_priv
{
	struct i2c_client*                  client;
	struct regmap*                      regmap;
	struct gpio_desc*                   gpiod_pwdn;
	struct media_pad                    pads[1];
	struct v4l2_subdev                  sd;
	struct v4l2_mbus_framefmt           fmt;
	struct v4l2_ctrl_handler            ctrl_handler;
	struct v4l2_fwnode_bus_mipi_csi2    mipi;
	struct mutex                        mutex;
	bool                                streaming;
};


static inline struct max96724_priv* to_max96724
	(
	struct v4l2_subdev*     _sd
	)
{
	return container_of( _sd, struct max96724_priv, sd );
}


static int max96724_read
	(
	struct max96724_priv*   priv,
	u16                     reg,
	u8*                     val
	)
{
	unsigned int reg_val = 0;
	int ret = regmap_read( priv->regmap, reg, &reg_val );

	if( ret )
	{
		dev_err( &priv->client->dev, "max96724_read() 0x%04x failed\n", reg );
	}
	else
	{
		*val = reg_val;
	}

	return ret;
}

static int max96724_write
	(
	struct max96724_priv*   priv,
	unsigned int            reg,
	u8                      val
	)
{
	int ret = regmap_write( priv->regmap, reg, val );

	if( ret )
	{
		dev_err( &priv->client->dev, "max96724_write() 0x%04x failed\n", reg );
	}

	return ret;
}

static int max96724_write_with_mask
	(
	struct max96724_priv*   priv,
	unsigned int            reg,
	u8                      mask,
	u8                      val
	)
{
	int ret = regmap_update_bits( priv->regmap, reg, mask, val );

	if( ret )
	{
		dev_err( &priv->client->dev, "max96724_write_with_mask() 0x%04x failed\n", reg );
	}

	return ret;
}

static int max96724_write_multi_regs
	(
	struct max96724_priv*   priv,
	unsigned int            reg,
	unsigned int            val,
	size_t                  count
	)
{
	uint8_t data[3] = { 0 };
	size_t i = 0;
	int ret = 0;
    
	for( i = 0; i < count; i++ )
	{
		data[i] = ( val >> ( 8 * ( count - 1 - i ) ) ) & 0xff;
	}

	ret = regmap_bulk_write( priv->regmap, reg, &data, count );

	if( ret )
	{
		dev_err( &priv->client->dev, "max96724_write_multi_regs() 0x%04x failed\n", reg );
	}

	return ret;
}


static int __maybe_unused max96724_reset
	(
	struct max96724_priv*  priv
	)
{
	int ret = max96724_write_with_mask( priv, REG_PWR1, 0x40, 0x40 ); // RESET_ALL

	// see t_LOCK2 in datasheet
	msleep( ( MAX96724_MIN_DELAY_US + MAX96724_DELAY_RANGE_US ) / 1000 );

	if( ret )
	{
		dev_err( &priv->client->dev, "max96724_reset() failed\n" );
	}

	return ret;
}


static const struct regmap_config max96724_i2c_regmap =
{
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x1271
};


static int max96724_mipi_enable
	(
	struct max96724_priv*   priv,
	bool                    enable
	)
{
	int ret = 0;

	if( enable )
	{
		ret =  max96724_write_with_mask( priv, REG_BACKTOP12, 0x02, 0x02 ); // CSI_OUT_EN       enabled
		ret += max96724_write_with_mask( priv, REG_MIPI_PHY0, 0x80, 0x80 ); // FORCE_CSI_OUT_EN enabled
	}
	else
	{
		ret  = max96724_write_with_mask( priv, REG_MIPI_PHY0, 0x80, 0x00 ); // FORCE_CSI_OUT_EN disabled
		ret += max96724_write_with_mask( priv, REG_BACKTOP12, 0x02, 0x00 ); // CSI_OUT_EN       disabled
	}

	if( ret )
	{
		dev_err( &priv->client->dev, "max96724_mipi_enable() failed\n" );
	}

	return ret;
}


static void max96724_pattern_colorbar_enable
	(
	struct max96724_priv*   priv
	)
{
	/////////////////////////////////////////
	// given values for 1920x1080 @ 30 fps
	/////////////////////////////////////////
	const uint16_t H_RES    = 1920;     // lines
	const uint16_t H_FP     = 88;       // PCLKs
	const uint16_t H_SYNCW  = 44;       // PCLKs
	const uint16_t H_BP     = 148;      // PCLKs

	const uint16_t V_RES    = 1080;     // lines
	const uint16_t V_FP     = 4;        // lines
	const uint16_t V_SYNCW  = 5;        // lines
	const uint16_t V_BP     = 36;       // lines

	const uint32_t VS_DLY   = 0;        // PCLKs
	const uint32_t HS_DLY   = 0;        // PCLKs

	const uint8_t GRAD_INC = 4;

	/////////////////////////////////////////
	// calculated values
	/////////////////////////////////////////
	const uint32_t H_TOTAL = H_RES + H_FP + H_SYNCW + H_BP;             //    2200
	const uint16_t V_TOTAL = V_RES + V_FP + V_SYNCW + V_BP;             //    1125
	const uint32_t VS_HIGH = V_SYNCW * H_TOTAL;                         //   11000
	const uint32_t VS_LOW = ( V_RES + V_FP + V_BP ) * H_TOTAL;          // 2464000
	const uint16_t HS_LOW = H_RES + H_FP + H_BP;                        //    2156
	const uint32_t V2D = ( V_SYNCW + V_BP ) * H_TOTAL + H_SYNCW + H_BP; //   90392
	const uint16_t DE_LOW = H_FP + H_SYNCW + H_BP;                      //     280

	max96724_write_with_mask( priv, REG_PATGEN_0, 0x04, 0x00 );     // do not invert DE
	max96724_write_with_mask( priv, REG_PATGEN_0, 0x08, 0x00 );     // do not invert HS
	max96724_write_with_mask( priv, REG_PATGEN_0, 0x10, 0x10 );     // invert VS

	max96724_write_multi_regs( priv, REG_VS_DLY_2,  VS_DLY,  3 );
	max96724_write_multi_regs( priv, REG_VS_HIGH_2, VS_HIGH, 3 );
	max96724_write_multi_regs( priv, REG_VS_LOW_2,  VS_LOW,  3 );
	max96724_write_multi_regs( priv, REG_V2H_2,     HS_DLY,  3 );
	max96724_write_multi_regs( priv, REG_HS_HIGH_1, H_SYNCW, 2 );
	max96724_write_multi_regs( priv, REG_HS_LOW_1,  HS_LOW,  2 );
	max96724_write_multi_regs( priv, REG_HS_CNT_1,  V_TOTAL, 2 );
	max96724_write_multi_regs( priv, REG_V2D_2,     V2D,     3 );
	max96724_write_multi_regs( priv, REG_DE_HIGH_1, H_RES,   2 );
	max96724_write_multi_regs( priv, REG_DE_LOW_1,  DE_LOW,  2 );
	max96724_write_multi_regs( priv, REG_DE_CNT_1,  V_RES,   2 );

	max96724_write_with_mask( priv, REG_PATGEN_1,   0x30, 0x20 );   // patgen_mode = colorbar

	max96724_write( priv, REG_GRAD_INCR, GRAD_INC );

	max96724_write_with_mask( priv, REG_VPRBS,      0x80, 0x00 );   // Pipe2 ONLY, patgen_clk_src => Value: 0
	max96724_write_with_mask( priv, REG_PCLK_FREQ,  0x03, 0x01 );   // PCLK f = 75 MHz
	max96724_write_with_mask( priv, REG_MIPI_PHY0,  0x80, 0x80 );   // force_csi_out => Value: 1
	max96724_write_with_mask( priv, REG_PATGEN_0,   0xe3, 0xe3 );   // Generate VS and HS and DE, free-running mode, do not touch inversion
}

static void max96724_pattern_colorbar_disable
	(
	struct max96724_priv*   priv
	)
{
	max96724_write( priv, REG_PATGEN_1, 0x00 ); // patgen_mode = none
	max96724_write( priv, REG_PATGEN_0, 0x03 ); // patgen reset
}


static int max96724_start_streaming
	(
	struct max96724_priv*  priv
	)
{
	int ret = pm_runtime_get_sync( &priv->client->dev );

	if( ret < 0 )
	{
		dev_err( &priv->client->dev, "max96724_start_streaming() could not sync PM\n" );
		pm_runtime_put_noidle( &priv->client->dev );
		return ret;
	}

	ret = max96724_mipi_enable( priv, true );

	if( ret )
	{
		dev_err( &priv->client->dev, "max96724_start_streaming() failed\n" );
		goto err_rpm_put;
	}

	return 0;

err_rpm_put:
	pm_runtime_put( &priv->client->dev );
	return ret;
}

static int max96724_stop_streaming
	(
	struct max96724_priv*  priv
	)
{
	int ret = max96724_mipi_enable( priv, false );

	if( ret )
	{
		dev_err( &priv->client->dev, "max96724_stop_streaming() failed\n" );
	}

	pm_runtime_put( &priv->client->dev );

	return ret;
}


static int max96724_set_stream
	(
	struct v4l2_subdev* sd,
	int                 enable
	)
{
	struct max96724_priv* priv = to_max96724( sd );
	int ret = 0;

	mutex_lock( &priv->mutex );

	if( enable == priv->streaming )
	{
		mutex_unlock( &priv->mutex );
		return 0;
	}

	if( enable )
	{
		ret = max96724_start_streaming( priv );

		if( ret )
		{
			dev_err( &priv->client->dev, "max96724_set_stream() failed\n" );
			goto err_unlock;
		}
	}
	else
	{
		max96724_stop_streaming( priv );
	}

	priv->streaming = enable;
	mutex_unlock( &priv->mutex );
	return ret;

err_unlock:
	mutex_unlock( &priv->mutex );

	return ret;
}


static int max96724_enum_mbus_code
	(
	struct v4l2_subdev*                 sd,
	struct v4l2_subdev_pad_config*      cfg,
	struct v4l2_subdev_mbus_code_enum*  code
	)
{
	if( 0 == code->pad )
	{
		if( code->index >= 1 )
		{
			return -EINVAL;
		}

		code->code = video_fmt_codes[video_fmt_code_index];
	}
	else
	{
		return -EINVAL;
	}

	return 0;
}


static int max96724_set_pad_format
	(
	struct v4l2_subdev*             sd,
	struct v4l2_subdev_pad_config*  cfg,
	struct v4l2_subdev_format*      format
	)
{
	format->format.width = FHD_WIDTH;
	format->format.height = FHD_HEIGHT;
	format->format.code = video_fmt_codes[video_fmt_code_index];
	format->format.field = V4L2_FIELD_NONE;

	return 0;
}

static int max96724_enum_frame_size
	(
	struct v4l2_subdev*                 sd,
	struct v4l2_subdev_pad_config*      cfg,
	struct v4l2_subdev_frame_size_enum* fse
	)
{
	if( fse->code != video_fmt_codes[video_fmt_code_index]
	 || fse->index >= 1
	)
	{
		return -EINVAL;
	}

	fse->min_width = FHD_WIDTH;
	fse->max_width = FHD_WIDTH;
	fse->min_height = FHD_HEIGHT;
	fse->max_height = FHD_HEIGHT;

	return 0;
}

struct v4l2_rect fhd_crop =
{
	.left = 0,
	.top = 0,
	.width = FHD_WIDTH,
	.height = FHD_HEIGHT
};

static const struct v4l2_rect *__max96724_get_pad_crop
	(
	struct max96724_priv*           priv,
	struct v4l2_subdev_pad_config*  cfg,
	unsigned int                    pad, 
	enum v4l2_subdev_format_whence  which
	)
{
	switch( which )
	{
		case V4L2_SUBDEV_FORMAT_TRY:
			return v4l2_subdev_get_try_crop( &priv->sd, cfg, pad );

		case V4L2_SUBDEV_FORMAT_ACTIVE:
			return &fhd_crop;
	}

	return NULL;
}

static int max96724_get_selection
	(
	struct v4l2_subdev*             sd,
	struct v4l2_subdev_pad_config*  cfg,
	struct v4l2_subdev_selection*   sel
	)
{
	switch( sel->target )
	{
		case V4L2_SEL_TGT_CROP: 
		{
			struct max96724_priv* priv = to_max96724( sd );

			mutex_lock( &priv->mutex );
			sel->r = *__max96724_get_pad_crop( priv, cfg, sel->pad, sel->which );
			mutex_unlock( &priv->mutex );

			return 0;
		}

		case V4L2_SEL_TGT_NATIVE_SIZE:
			sel->r.top = 0;
			sel->r.left = 0;
			sel->r.width = FHD_WIDTH;
			sel->r.height = FHD_HEIGHT;

			return 0;

		case V4L2_SEL_TGT_CROP_DEFAULT:
		case V4L2_SEL_TGT_CROP_BOUNDS:
			sel->r.top = 0;
			sel->r.left = 0;
			sel->r.width = FHD_WIDTH;
			sel->r.height = FHD_HEIGHT;

			return 0;
	}

	return -EINVAL;
}


static const struct v4l2_subdev_core_ops max96724_core_ops =
{
	.subscribe_event   = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe
};

static const struct v4l2_subdev_video_ops max96724_video_ops =
{
	.s_stream = max96724_set_stream
};

static const struct v4l2_subdev_pad_ops max96724_pad_ops =
{
	.enum_mbus_code  = max96724_enum_mbus_code,
	.set_fmt         = max96724_set_pad_format,
	.get_fmt         = max96724_set_pad_format,
	.enum_frame_size = max96724_enum_frame_size,
	.get_selection	 = max96724_get_selection
};

static const struct v4l2_subdev_ops max96724_subdev_ops =
{
	.core  = &max96724_core_ops,
	.video = &max96724_video_ops,
	.pad   = &max96724_pad_ops
};


static int max96724_parse_dt
	(
	struct max96724_priv *priv
	)
{
	struct fwnode_handle* endpoint;

	struct v4l2_fwnode_endpoint ep_cfg =
	{
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
    
	int ret = 0;

	endpoint = fwnode_graph_get_next_endpoint( dev_fwnode( &priv->client->dev ), NULL );

	if( !endpoint )
	{
		dev_err( &priv->client->dev, "max96724_parse_dt() endpoint node not found\n" );
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse( endpoint, &ep_cfg );

	if( ret )
	{
		dev_err( &priv->client->dev, "max96724_parse_dt() could not parse endpoint\n" );
		return -EINVAL;
	}

	if( 2 != ep_cfg.bus.mipi_csi2.num_data_lanes )
	{
		dev_err( &priv->client->dev, "max96724_parse_dt() only 2 data lanes are supported\n" );
		return -EINVAL;
	}

	fwnode_handle_put( endpoint );
	priv->mipi = ep_cfg.bus.mipi_csi2;

	return 0;
}


static void max96724_mipi_configure
	(
	struct max96724_priv*  priv
	)
{
	int ret = max96724_mipi_enable( priv, false );

	if( ret )
	{
		dev_err( &priv->client->dev, "max96724_mipi_configure() could not disable MIPI\n" );
	}

	// select MIPI output as 4 ports with 2 data lanes each (phy_4x2)
	ret = max96724_write( priv, REG_MIPI_PHY0, 0x01 );

	if( ret )
	{
		dev_err( &priv->client->dev, "max96724_mipi_configure() could not set MIPI to phy_4x2\n" );
	}

	// configure 2 data lane for PHY2 (port E)
	ret = max96724_write_with_mask( priv, REG_MIPI_TX10_OF_MIPI_TX2, 0xc0, 0x40 ); // CSI2_LANE_CNT

	if( ret )
	{
		dev_err( &priv->client->dev, "max96724_mipi_configure() could not configure 2-lane for D-PHY\n" );
	}

	// configure lane mapping for PHY3 (bits 7:4) and PHY2 (bits 3:0)
	// 76 54  32 10
	// == ==  == ==
	// 11 10  01 00  ==  0xE4
	//  |  |   |  |                            MAX96724
	//  |  |   |  --  PHY2 D0 -> port E D0    pins (43,44)   \  going to the RPi4
	//  |  |   -----  PHY2 D1 -> port E D1    pins (47,48)   /   CSI-2 connector
	//  |  |
	//  |  ---------  PHY3 D0 -> port F D2    pins (49,50)   \        not
	//  ------------  PHY3 D1 -> port F D3    pins (53,54)   /     connected
	//
	ret = max96724_write( priv, REG_MIPI_PHY4, 0xe4 ); // phy3_lane_map, phy2_lane_map

	if( ret )
	{
		dev_err( &priv->client->dev, "max96724_mipi_configure() could not configure lane mapping for port E\n" );
	}

	// enable PHY2 only
	// 7 6 5 4 3 2 1 0
	// 0 1 0 0 0 0 0 0  ==  0x40
	// | | | |
	// | | | |
	// | | | ---  enable PHY0
	// | | -----  enable PHY1
	// | -------  enable PHY2
	// ---------  enable PHY3
	ret = max96724_write_with_mask( priv, REG_MIPI_PHY2, 0xf0, 0x40 ); // phy_Stdby_n

	if( ret )
	{
		dev_err( &priv->client->dev, "max96724_mipi_configure() could not enable PHY2 only\n" );
	}
}


static int max96724_open
    (
    struct v4l2_subdev*     sd,
    struct v4l2_subdev_fh*  fh
    )
{
	struct max96724_priv* priv = to_max96724( sd );
	struct v4l2_mbus_framefmt* try_fmt_img = v4l2_subdev_get_try_format( sd, fh->pad, 0 );
	struct v4l2_rect* try_crop;

	mutex_lock( &priv->mutex );

	try_fmt_img->width = FHD_WIDTH;
	try_fmt_img->height = FHD_HEIGHT;
	try_fmt_img->code = video_fmt_codes[video_fmt_code_index];
	try_fmt_img->field = V4L2_FIELD_NONE;

	try_crop = v4l2_subdev_get_try_crop( sd, fh->pad, 0 );
	try_crop->top = 0;
	try_crop->left = 0;
	try_crop->width = FHD_WIDTH;
	try_crop->height = FHD_HEIGHT;

	mutex_unlock( &priv->mutex );

	return 0;
}

static const struct v4l2_subdev_internal_ops max96724_internal_ops =
{
	.open = max96724_open
};


static int max96724_power_on
	(
	struct device*  dev
	)
{
	struct i2c_client* client = to_i2c_client( dev );
	struct v4l2_subdev* sd = i2c_get_clientdata( client );
	struct max96724_priv* priv = to_max96724( sd );

	// active LOW -> pwr up
	gpiod_set_value_cansleep( priv->gpiod_pwdn, 1 );
	// wait at least 45ms
	usleep_range( MAX96724_MIN_DELAY_US, MAX96724_MIN_DELAY_US + MAX96724_DELAY_RANGE_US );

	return 0;
}

static int max96724_power_off
	(
	struct device*  dev
	)
{
	struct i2c_client* client = to_i2c_client( dev );
	struct v4l2_subdev* sd = i2c_get_clientdata( client );
	struct max96724_priv* priv = to_max96724( sd );

	// active LOW -> pwr down
	gpiod_set_value_cansleep( priv->gpiod_pwdn, 0 );

	return 0;
}


static void max96724_set_default_format
	(
	struct max96724_priv*  priv
	)
{
	struct v4l2_mbus_framefmt* fmt;
	fmt = &priv->fmt;
	fmt->width = FHD_WIDTH;
	fmt->height = FHD_HEIGHT;
	fmt->code = video_fmt_codes[0];
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT( fmt->colorspace );
}


static int max96724_v4l2_register
	(
	struct max96724_priv*  priv
	)
{
	int ret = 0;

	v4l2_i2c_subdev_init( &priv->sd, priv->client, &max96724_subdev_ops );

	priv->sd.internal_ops = &max96724_internal_ops;
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_EVENTS;
	priv->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	priv->pads[0].flags = MEDIA_PAD_FL_SOURCE;

	max96724_set_default_format( priv );

	ret = media_entity_pads_init( &priv->sd.entity, 1, priv->pads );

	if( ret )
	{
		dev_err( &priv->client->dev, "max96724_v4l2_register() failed to init entity pads\n" );
		goto error_handler;
	}

	v4l2_set_subdevdata( &priv->sd, priv );

	ret = v4l2_async_register_subdev( &priv->sd );

	if( ret < 0 )
	{
		dev_err( &priv->client->dev, "max96724_v4l2_register() failed to register subdevice\n" );
		goto error_media_entity;
	}

	// enable runtime PM and turn off the device
	pm_runtime_set_active( &priv->client->dev );
	pm_runtime_enable( &priv->client->dev );
	pm_runtime_idle( &priv->client->dev );

	return 0;

error_media_entity:
	media_entity_cleanup( &priv->sd.entity );

error_handler:
	v4l2_ctrl_handler_free( &priv->ctrl_handler );

	max96724_power_off( &priv->client->dev );

	return ret;
}


static int max96724_probe
	(
	struct i2c_client*  client
	)
{
	const uint8_t MAX96724_ID = 0xa2; // REG13 - device ID
	int ret = 0;
	u8 regval = 0;
	struct max96724_priv* priv;

	priv = devm_kzalloc( &client->dev, sizeof( *priv ), GFP_KERNEL );

	if( !priv )
	{
		return -ENOMEM;
	}

	// set I2C client
	priv->client = client;
	i2c_set_clientdata( client, priv );

	// init registry map
	priv->regmap = devm_regmap_init_i2c( client, &max96724_i2c_regmap );

	if( IS_ERR( priv->regmap ) )
	{
		return PTR_ERR( priv->regmap );
	}

	// check device ID
	ret = max96724_read( priv, REG_REG13, &regval );

	if( ret )
	{
		return ret;
	}
	else if( MAX96724_ID != regval )
	{
		return -ENODEV;
	}

	// request enable pin
	priv->gpiod_pwdn = devm_gpiod_get_optional( &client->dev, "enable", GPIOD_OUT_HIGH );

	if( IS_ERR( priv->gpiod_pwdn ) )
	{
		return PTR_ERR( priv->gpiod_pwdn );
	}

	gpiod_set_consumer_name( priv->gpiod_pwdn, "max96724-pwdn" );
	// active LOW -> wake up
	gpiod_set_value_cansleep( priv->gpiod_pwdn, 1 );

	if( priv->gpiod_pwdn )
	{
		pr_info( "max96724_probe() GPIO - waiting for at least 45ms\n" );
		// wait at least 45ms
		usleep_range( MAX96724_MIN_DELAY_US, MAX96724_MIN_DELAY_US + MAX96724_DELAY_RANGE_US );
	}

	// I2C client commands
	i2c_client_cmd = client;

	// device tree
	ret = max96724_parse_dt( priv );

	if( ret )
	{
		return ret;
	}

	// MIPI config
	max96724_mipi_configure( priv );

	// init V4L2 subdevice
	ret = max96724_v4l2_register( priv );

	return ret;
}


static int __maybe_unused max96724_suspend
	(
	struct device*  dev
	)
{
	struct i2c_client* client = to_i2c_client( dev );
	struct v4l2_subdev* sd = i2c_get_clientdata( client );
	struct max96724_priv* priv = to_max96724( sd );

	if( priv->streaming )
	{
		max96724_stop_streaming( priv );
	}

	return 0;
}

static int __maybe_unused max96724_resume
	(
	struct device*  dev
	)
{
	struct i2c_client* client = to_i2c_client( dev );
	struct v4l2_subdev* sd = i2c_get_clientdata( client );
	struct max96724_priv* priv = to_max96724( sd );
	int ret = 0;

	if( priv->streaming )
	{
		ret = max96724_start_streaming( priv );;

		if( ret )
		{
			dev_err( &priv->client->dev, "max96724_resume() could not start streaming\n" );
			goto error;
		}
	}

	return 0;

error:
	max96724_stop_streaming( priv );
	priv->streaming = false;

	return ret;
}


static int max96724_remove
	(
	struct i2c_client*  client
	)
{
	struct v4l2_subdev* sd = i2c_get_clientdata( client );
	struct max96724_priv* priv = to_max96724( sd );

	v4l2_async_unregister_subdev( &priv->sd );

	media_entity_cleanup( &sd->entity );

	pm_runtime_disable( &client->dev );

	if( !pm_runtime_status_suspended( &client->dev ) )
	{
		max96724_power_off( &client->dev );
	}

	pm_runtime_set_suspended( &client->dev );

	return 0;
}


static const struct i2c_device_id max96724_id[] =
{
	{ "max96724", 0 },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE( i2c, max96724_id );

static const struct of_device_id max96724_dt_ids[] =
{
	{ .compatible = "adi,max96724" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE( of, max96724_dt_ids );


static const struct dev_pm_ops max96724_pm_ops =
{
	SET_SYSTEM_SLEEP_PM_OPS( max96724_suspend, max96724_resume )
	SET_RUNTIME_PM_OPS( max96724_power_off, max96724_power_on, NULL )
};

static struct i2c_driver max96724_i2c_driver =
{
	.driver =
	{
		.name = "max96724",
		.of_match_table = max96724_dt_ids,
        .pm = &max96724_pm_ops
	},
	.probe_new = max96724_probe,
	.remove    = max96724_remove,
	.id_table  = max96724_id
};

module_i2c_driver( max96724_i2c_driver );


/////////////////////////////
// I2C module parameters
// the number and order of items must match the definitions from high-level apps
/////////////////////////////
enum max96724_i2c_cmd
{
	// I2C commands
	MAX96724_I2C_CMD_NONE,
	MAX96724_I2C_CMD_WRITE,
	MAX96724_I2C_CMD_READ,
	// I2C commands
	MAX96724_I2C_CMD_START_PATTERN_COLORBARS,
	MAX96724_I2C_CMD_STOP_PATTERN,
	MAX96724_I2C_CMD_SET_VIDEO_FMT_CODE_RGB888,
	MAX96724_I2C_CMD_SET_VIDEO_FMT_CODE_RAW10
};

uint8_t  i2c_bus_nr = 10;           // use only base 10
uint8_t  i2c_slave_addr = 0x27;     // allowed base 10 [0 .. 255] and 16 [0x00 .. 0xFF]

uint8_t  i2c_cmd = MAX96724_I2C_CMD_NONE;

uint16_t i2c_reg_addr = 0;          // allowed base 10 [0 .. 65535] and 16 [0x0000 .. 0xFFFF]

uint16_t i2c_value = 0;             // allowed base 10 [0 .. 65535] and 16 [0x0000 .. 0xFFFF]
bool     i2c_value_is_word = false; // allowed y/Y or n/N (not t/T or f/F)

/////////////////////////////
// I2C bus nr
/////////////////////////////
int max96724_param_i2c_bus_nr
	(
	const char*                 val,
	const struct kernel_param*  kp
	)
{
	int res = param_set_byte( val, kp );

	if( res )
	{
		pr_warn( "max96724_param_i2c_bus_nr() failed\n" );
	}

	return res;
}

const struct kernel_param_ops param_i2c_bus_nr_ops =
{
	.set = &max96724_param_i2c_bus_nr,
	.get = &param_get_byte
};

module_param_cb( i2c_bus_nr, &param_i2c_bus_nr_ops, &i2c_bus_nr, S_IRUGO | S_IWUSR );


/////////////////////////////
// I2C slave address
/////////////////////////////
int max96724_param_i2c_slave_addr
	(
	const char*                 val,
	const struct kernel_param*  kp
	)
{
	int res = param_set_byte( val, kp );

	if( res )
	{
		pr_warn( "max96724_param_i2c_slave_addr() failed\n" );
	}

	return res;
}

const struct kernel_param_ops param_i2c_slave_addr_ops =
{
	.set = &max96724_param_i2c_slave_addr,
	.get = &param_get_byte
};

module_param_cb( i2c_slave_addr, &param_i2c_slave_addr_ops, &i2c_slave_addr, S_IRUGO | S_IWUSR );

/////////////////////////////
// I2C command
/////////////////////////////
int max96724_param_i2c_cmd
	(
	const char*                 val,
	const struct kernel_param*  kp
	)
{
	struct v4l2_subdev* sd = NULL;
	struct max96724_priv* priv = NULL;

	uint8_t tx_buffer[4] = { 0 };
	uint8_t rx_buffer[2] = { 0 };

	int res = param_set_byte( val, kp );

	if( 0 == res )
	{
		if( NULL != i2c_client_cmd )
		{
			switch( i2c_cmd )
			{
				case MAX96724_I2C_CMD_WRITE:
					if( !i2c_value_is_word )
					{
						// address is word && value is byte
						tx_buffer[0] = ( uint8_t )( ( i2c_reg_addr >> 8 ) & 0x00FF );
						tx_buffer[1] = ( uint8_t )( i2c_reg_addr & 0x00FF );
						tx_buffer[2] = ( uint8_t )( i2c_value & 0x00FF );

						if( 3 != i2c_master_send( i2c_client_cmd, tx_buffer, 3 ) )
						{
							pr_warn( "max96724_param_i2c_cmd() - WRITE failed\n" );
						}
					}
					else
					{
						// address is word && value is word
						tx_buffer[0] = ( uint8_t )( ( i2c_reg_addr >> 8 ) & 0x00FF );
						tx_buffer[1] = ( uint8_t )( i2c_reg_addr & 0x00FF );
						tx_buffer[2] = ( uint8_t )( ( i2c_value >> 8 ) & 0x00FF );
						tx_buffer[3] = ( uint8_t )( i2c_value & 0x00FF );

						if( 4 != i2c_master_send( i2c_client_cmd, tx_buffer, 4 ) )
						{
							pr_warn( "max96724_param_i2c_cmd() - WRITE failed\n" );
						}
					}
					break;

				case MAX96724_I2C_CMD_READ:
					if( !i2c_value_is_word )
					{
						// address is word && value is byte
						tx_buffer[0] = ( uint8_t )( ( i2c_reg_addr >> 8 ) & 0x00FF );
						tx_buffer[1] = ( uint8_t )( i2c_reg_addr & 0x00FF );

						if( 2 == i2c_master_send( i2c_client_cmd, tx_buffer, 2 ) )
						{
							if( 1 == i2c_master_recv( i2c_client_cmd, rx_buffer, 1 ) )
							{
								i2c_value = rx_buffer[0];
							}
							else
							{
								pr_warn( "max96724_param_i2c_cmd() - READ-RX failed\n" );
							}
						}
						else
						{
							pr_warn( "max96724_param_i2c_cmd() - READ-TX failed\n" );
						}
					}
					else
					{
						// address is word && value is word
						pr_warn( "max96724_param_i2c_cmd() - READ a word from 16-bit address not implemented\n" );
					}
					break;

				case MAX96724_I2C_CMD_START_PATTERN_COLORBARS:
					{
						if( i2c_client_cmd )
						{
							sd = i2c_get_clientdata( i2c_client_cmd );
							priv = to_max96724( sd );
							max96724_pattern_colorbar_enable( priv );
						}
						else
						{
							pr_warn( "max96724_param_i2c_cmd() - I2C client is NULL\n" );
						}
					}
					break;

				case MAX96724_I2C_CMD_STOP_PATTERN:
					{
						if( i2c_client_cmd )
						{
							sd = i2c_get_clientdata( i2c_client_cmd );
							priv = to_max96724( sd );
							max96724_pattern_colorbar_disable( priv );
						}
						else
						{
							pr_warn( "max96724_param_i2c_cmd() - I2C client is NULL\n" );
						}
					}
					break;

				case MAX96724_I2C_CMD_SET_VIDEO_FMT_CODE_RGB888:
					video_fmt_code_index = VIDEO_FMT_CODE_RGB888;
					break;

				case MAX96724_I2C_CMD_SET_VIDEO_FMT_CODE_RAW10:
					video_fmt_code_index = VIDEO_FMT_CODE_RAW10;
					break;

				case MAX96724_I2C_CMD_NONE:
				default:
					pr_warn( "max96724_param_i2c_cmd() - unknown command\n" );
					break;
			}
		}
		else
		{
			pr_warn( "max96724_param_i2c_cmd() - i2c_client_cmd is NULL\n" );
		}
	}
	else
	{
		pr_warn( "max96724_param_i2c_cmd() failed\n" );
	}

	return res;
}

const struct kernel_param_ops param_i2c_cmd_ops =
{
	.set = &max96724_param_i2c_cmd,
	.get = &param_get_byte
};

module_param_cb( i2c_cmd, &param_i2c_cmd_ops, &i2c_cmd, S_IRUGO | S_IWUSR );

/////////////////////////////
// I2C register address
/////////////////////////////
int max96724_param_i2c_reg_addr
	(
	const char*                 val,
	const struct kernel_param*  kp
	)
{
	int res = param_set_ushort( val, kp );

	if( res )
	{
		pr_warn( "max96724_param_i2c_reg_addr() failed\n" );
	}

	return res;
}

const struct kernel_param_ops param_i2c_reg_addr_ops =
{
	.set = &max96724_param_i2c_reg_addr,
	.get = &param_get_ushort
};

module_param_cb( i2c_reg_addr, &param_i2c_reg_addr_ops, &i2c_reg_addr, S_IRUGO | S_IWUSR );

/////////////////////////////
// I2C value
/////////////////////////////
int max96724_param_i2c_value
	(
	const char*                 val,
	const struct kernel_param*  kp
	)
{
	int res = param_set_ushort( val, kp );

	if( res )
	{
		pr_warn( "max96724_param_i2c_value() failed\n" );
	}

	return res;
}

const struct kernel_param_ops param_i2c_value_ops =
{
	.set = &max96724_param_i2c_value,
	.get = &param_get_ushort
};

module_param_cb( i2c_value, &param_i2c_value_ops, &i2c_value, S_IRUGO | S_IWUSR );

/////////////////////////////
// I2C value_is_word
/////////////////////////////
int max96724_param_i2c_value_is_word
	(
	const char*                 val,
	const struct kernel_param*  kp
	)
{
	int res = param_set_bool( val, kp );

	if( res )
	{
		pr_warn( "max96724_param_i2c_value_is_word() failed\n" );
	}

	return res;
}

const struct kernel_param_ops param_i2c_value_is_word_ops =
{
	.set = &max96724_param_i2c_value_is_word,
	.get = &param_get_bool
};

module_param_cb( i2c_value_is_word, &param_i2c_value_is_word_ops, &i2c_value_is_word, S_IRUGO | S_IWUSR );

MODULE_DESCRIPTION( "Maxim MAX96724 Quad GMSL2/1 Deserializer Driver" );
MODULE_AUTHOR( "Analog Devices, Inc." );
MODULE_LICENSE( "GPL" );
