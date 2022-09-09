/*
 * dbmdx-export.h  --  DBMDX exported interface
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DBMDX_EXPORT_H
#define _DBMDX_EXPORT_H

#include <sound/soc.h>
#include <linux/i2c.h>

#ifdef SND_SOC_COMPONENT
int dbmdx_remote_add_codec_controls(struct snd_soc_component *component);
#else
int dbmdx_remote_add_codec_controls(struct snd_soc_codec *codec);
#endif /* SND_SOC_COMPONENT */

typedef void (*event_cb)(int);
void dbmdx_remote_register_event_callback(event_cb func);

enum i2c_freq_t {
	I2C_FREQ_SLOW,
	I2C_FREQ_FAST
};
typedef void (*set_i2c_freq_cb)(struct i2c_adapter*, enum i2c_freq_t);
void dbmdx_remote_register_set_i2c_freq_callback(set_i2c_freq_cb func);

int dbmdx_debug_read(u8 *src, u8 *dest, int size);
int dbmdx_debug_write(u8 *dest, u8 *src, int size);

#endif
