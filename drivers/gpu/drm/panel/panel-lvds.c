/*
 * rcar_du_crtc.c  --  R-Car Display Unit CRTCs
 *
 * Copyright (C) 2016 Laurent Pinchart
 * Copyright (C) 2016 Renesas Electronics Corporation
 *
 * Contact: Laurent Pinchart (laurent.pinchart@ideasonboard.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/backlight.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_panel.h>

#include <video/display_timing.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>

enum panel_type {
	PANEL_LVDS,
	PANEL_DPI
};

struct panel_lvds {
	struct drm_panel panel;
	struct device *dev;

	const char *label;
	unsigned int width;
	unsigned int height;
	struct videomode video_mode;
	unsigned int bus_format;
	bool data_mirror;

	struct backlight_device *backlight;

	struct gpio_desc *enable_gpio;
	struct gpio_desc *reset_gpio;
};

static inline struct panel_lvds *to_panel_lvds(struct drm_panel *panel)
{
	return container_of(panel, struct panel_lvds, panel);
}

static int panel_lvds_disable(struct drm_panel *panel)
{
	struct panel_lvds *lvds = to_panel_lvds(panel);

	if (lvds->backlight) {
		lvds->backlight->props.power = FB_BLANK_POWERDOWN;
		lvds->backlight->props.state |= BL_CORE_FBBLANK;
		backlight_update_status(lvds->backlight);
	}

	return 0;
}

static int panel_lvds_unprepare(struct drm_panel *panel)
{
	struct panel_lvds *lvds = to_panel_lvds(panel);

	if (lvds->enable_gpio)
		gpiod_set_value_cansleep(lvds->enable_gpio, 0);

	return 0;
}

static int panel_lvds_prepare(struct drm_panel *panel)
{
	struct panel_lvds *lvds = to_panel_lvds(panel);

	if (lvds->enable_gpio)
		gpiod_set_value_cansleep(lvds->enable_gpio, 1);

	return 0;
}

static int panel_lvds_enable(struct drm_panel *panel)
{
	struct panel_lvds *lvds = to_panel_lvds(panel);

	if (lvds->backlight) {
		lvds->backlight->props.state &= ~BL_CORE_FBBLANK;
		lvds->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(lvds->backlight);
	}

	return 0;
}

static int panel_lvds_get_modes(struct drm_panel *panel)
{
	struct panel_lvds *lvds = to_panel_lvds(panel);
	struct drm_connector *connector = lvds->panel.connector;
	struct drm_display_mode *mode;

	mode = drm_mode_create(lvds->panel.drm);
	if (!mode)
		return 0;

	drm_display_mode_from_videomode(&lvds->video_mode, mode);
	mode->type |= DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = lvds->width;
	connector->display_info.height_mm = lvds->height;
	drm_display_info_set_bus_formats(&connector->display_info,
					 &lvds->bus_format, 1);
	drm_bus_flags_from_videomode(&lvds->video_mode,
				     &connector->display_info.bus_flags);
	connector->display_info.bus_flags |= lvds->data_mirror
					  ? DRM_BUS_FLAG_DATA_LSB_TO_MSB
					  : DRM_BUS_FLAG_DATA_MSB_TO_LSB;

	return 1;
}

static const struct drm_panel_funcs panel_lvds_funcs = {
	.disable = panel_lvds_disable,
	.unprepare = panel_lvds_unprepare,
	.prepare = panel_lvds_prepare,
	.enable = panel_lvds_enable,
	.get_modes = panel_lvds_get_modes,
};

static int panel_lvds_parse_dt(struct panel_lvds *lvds)
{
	struct device_node *np = lvds->dev->of_node;
	struct display_timing timing;
	const char *mapping;
	int ret;
	enum panel_type type;

	ret = of_get_display_timing(np, "panel-timing", &timing);
	if (ret < 0)
		return ret;

	videomode_from_timing(&timing, &lvds->video_mode);

	ret = of_property_read_u32(np, "width-mm", &lvds->width);
	if (ret < 0) {
		dev_err(lvds->dev, "%pOF: invalid or missing %s DT property\n",
			np, "width-mm");
		return -ENODEV;
	}
	ret = of_property_read_u32(np, "height-mm", &lvds->height);
	if (ret < 0) {
		dev_err(lvds->dev, "%pOF: invalid or missing %s DT property\n",
			np, "height-mm");
		return -ENODEV;
	}

	of_property_read_string(np, "label", &lvds->label);

	ret = of_property_read_string(np, "data-mapping", &mapping);
	if (ret < 0) {
		dev_err(lvds->dev, "%pOF: invalid or missing %s DT property\n",
			np, "data-mapping");
		return -ENODEV;
	}

	type = (enum panel_type)of_device_get_match_data(lvds->dev);
	switch (type) {
	case PANEL_LVDS:
		if (!strcmp(mapping, "jeida-18")) {
			lvds->bus_format = MEDIA_BUS_FMT_RGB666_1X7X3_SPWG;
		} else if (!strcmp(mapping, "jeida-24")) {
			lvds->bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA;
		} else if (!strcmp(mapping, "vesa-24")) {
			lvds->bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_SPWG;
		}
		break;
	case PANEL_DPI:
		if (!strcmp(mapping, "rgb24")) {
			lvds->bus_format = MEDIA_BUS_FMT_RGB888_1X24;
		} else if (!strcmp(mapping, "rgb565")) {
			lvds->bus_format = MEDIA_BUS_FMT_RGB565_1X16;
		} else if (!strcmp(mapping, "bgr666")) {
			lvds->bus_format = MEDIA_BUS_FMT_RGB666_1X18;
		} else if (!strcmp(mapping, "lvds666")) {
			lvds->bus_format = MEDIA_BUS_FMT_RGB666_1X24_CPADHI;
		}
	};

	if (!lvds->bus_format) {
		dev_err(lvds->dev, "%pOF: invalid or missing %s DT property\n",
			np, "data-mapping");
		return -EINVAL;
	}

	lvds->data_mirror = of_property_read_bool(np, "data-mirror");

	return 0;
}

static int panel_lvds_probe(struct platform_device *pdev)
{
	struct panel_lvds *lvds;
	struct device_node *np;
	int ret;

	lvds = devm_kzalloc(&pdev->dev, sizeof(*lvds), GFP_KERNEL);
	if (!lvds)
		return -ENOMEM;

	lvds->dev = &pdev->dev;

	ret = panel_lvds_parse_dt(lvds);
	if (ret < 0)
		return ret;

	/* Get GPIOs and backlight controller. */
	lvds->enable_gpio = devm_gpiod_get_optional(lvds->dev, "enable",
						     GPIOD_OUT_LOW);
	if (IS_ERR(lvds->enable_gpio)) {
		ret = PTR_ERR(lvds->enable_gpio);
		dev_err(lvds->dev, "failed to request %s GPIO: %d\n",
			"enable", ret);
		return ret;
	}

	lvds->reset_gpio = devm_gpiod_get_optional(lvds->dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(lvds->reset_gpio)) {
		ret = PTR_ERR(lvds->reset_gpio);
		dev_err(lvds->dev, "failed to request %s GPIO: %d\n",
			"reset", ret);
		return ret;
	}

	np = of_parse_phandle(lvds->dev->of_node, "backlight", 0);
	if (np) {
		lvds->backlight = of_find_backlight_by_node(np);
		of_node_put(np);

		if (!lvds->backlight)
			return -EPROBE_DEFER;
	}

	/*
	 * TODO: Handle all power supplies specified in the DT node in a generic
	 * way for panels that don't care about power supply ordering. LVDS
	 * panels that require a specific power sequence will need a dedicated
	 * driver.
	 */

	/* Register the panel. */
	drm_panel_init(&lvds->panel);
	lvds->panel.dev = lvds->dev;
	lvds->panel.funcs = &panel_lvds_funcs;

	ret = drm_panel_add(&lvds->panel);
	if (ret < 0)
		goto error;

	dev_set_drvdata(lvds->dev, lvds);
	return 0;

error:
	put_device(&lvds->backlight->dev);
	return ret;
}

static int panel_lvds_remove(struct platform_device *pdev)
{
	struct panel_lvds *lvds = dev_get_drvdata(&pdev->dev);

	drm_panel_detach(&lvds->panel);
	drm_panel_remove(&lvds->panel);

	panel_lvds_disable(&lvds->panel);

	if (lvds->backlight)
		put_device(&lvds->backlight->dev);

	return 0;
}

static const struct of_device_id panel_lvds_of_table[] = {
	{ .compatible = "panel-lvds", .data = (void *)PANEL_LVDS },
	{ .compatible = "panel-dpi", .data = (void *)PANEL_DPI },
	{ /* Sentinel */ },
};

MODULE_DEVICE_TABLE(of, panel_lvds_of_table);

static struct platform_driver panel_lvds_driver = {
	.probe		= panel_lvds_probe,
	.remove		= panel_lvds_remove,
	.driver		= {
		.name	= "panel-generic",
		.of_match_table = panel_lvds_of_table,
	},
};

module_platform_driver(panel_lvds_driver);

MODULE_AUTHOR("Laurent Pinchart <laurent.pinchart@ideasonboard.com>");
MODULE_DESCRIPTION("LVDS Panel Driver");
MODULE_LICENSE("GPL");
