/*
 * Register map access API
 *
 * Copyright 2011 Wolfson Microelectronics plc
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/err.h>

#define CREATE_TRACE_POINTS
#include <trace/events/regmap.h>

#include "internal.h"

static int _regmap_bus_read(void *context, unsigned int reg,
			    unsigned int *val);
static int _regmap_bus_formatted_write(void *context, unsigned int reg,
				       unsigned int val);
static int _regmap_bus_raw_write(void *context, unsigned int reg,
				 unsigned int val);

bool regmap_writeable(struct regmap *map, unsigned int reg)
{
	if (map->max_register && reg > map->max_register)
		return false;

	if (map->writeable_reg)
		return map->writeable_reg(map->dev, reg);

	return true;
}

bool regmap_readable(struct regmap *map, unsigned int reg)
{
	if (map->max_register && reg > map->max_register)
		return false;

	if (map->readable_reg)
		return map->readable_reg(map->dev, reg);

	return true;
}

bool regmap_volatile(struct regmap *map, unsigned int reg)
{
	if (map->max_register && reg > map->max_register)
		return false;

	if (map->volatile_reg)
		return map->volatile_reg(map->dev, reg);

	return true;
}

bool regmap_precious(struct regmap *map, unsigned int reg)
{
	if (map->max_register && reg > map->max_register)
		return false;

	if (map->precious_reg)
		return map->precious_reg(map->dev, reg);

	return false;
}

static bool regmap_volatile_range(struct regmap *map, unsigned int reg,
	unsigned int num)
{
	unsigned int i;

	for (i = 0; i < num; i++)
		if (!regmap_volatile(map, reg + i))
			return false;

	return true;
}

static void regmap_format_4_12_write(struct regmap *map,
				     unsigned int reg, unsigned int val)
{
	__be16 *out = map->work_buf;
	*out = cpu_to_be16((reg << 12) | val);
}

static void regmap_format_7_9_write(struct regmap *map,
				    unsigned int reg, unsigned int val)
{
	__be16 *out = map->work_buf;
	*out = cpu_to_be16((reg << 9) | val);
}

static void regmap_format_10_14_write(struct regmap *map,
				    unsigned int reg, unsigned int val)
{
	u8 *out = map->work_buf;

	out[2] = val;
	out[1] = (val >> 8) | (reg << 6);
	out[0] = reg >> 2;
}

static void regmap_format_8(void *buf, unsigned int val)
{
	u8 *b = buf;

	b[0] = val;
}

static void regmap_format_16(void *buf, unsigned int val)
{
	__be16 *b = buf;

	b[0] = cpu_to_be16(val);
}

static unsigned int regmap_parse_8(void *buf)
{
	u8 *b = buf;

	return b[0];
}

static unsigned int regmap_parse_16(void *buf)
{
	__be16 *b = buf;

	b[0] = be16_to_cpu(b[0]);

	return b[0];
}

static void dev_get_regmap_release(struct device *dev, void *res)
{
	/*
	 * We don't actually have anything to do here; the goal here
	 * is not to manage the regmap but to provide a simple way to
	 * get the regmap back given a struct device.
	 */
}

/**
 * regmap_init(): Initialise register map
 *
 * @dev: Device that will be interacted with
 * @bus: Bus-specific callbacks to use with device
 * @bus_context: Data passed to bus-specific callbacks
 * @config: Configuration for register map
 *
 * The return value will be an ERR_PTR() on error or a valid pointer to
 * a struct regmap.  This function should generally not be called
 * directly, it should be called by bus-specific init functions.
 */
struct regmap *regmap_init(struct device *dev,
			   const struct regmap_bus *bus,
			   void *bus_context,
			   const struct regmap_config *config)
{
	struct regmap *map, **m;
	int ret = -EINVAL;

	if (!config)
		goto err;

	map = kzalloc(sizeof(*map), GFP_KERNEL);
	if (map == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	mutex_init(&map->lock);
	map->format.buf_size = (config->reg_bits + config->val_bits) / 8;
	map->format.reg_bytes = config->reg_bits / 8;
	map->format.val_bytes = config->val_bits / 8;
	map->dev = dev;
	map->bus = bus;
	map->bus_context = bus_context;
	map->max_register = config->max_register;
	map->writeable_reg = config->writeable_reg;
	map->readable_reg = config->readable_reg;
	map->volatile_reg = config->volatile_reg;
	map->precious_reg = config->precious_reg;
	map->cache_type = config->cache_type;
	map->name = config->name;

	if (config->read_flag_mask || config->write_flag_mask) {
		map->read_flag_mask = config->read_flag_mask;
		map->write_flag_mask = config->write_flag_mask;
	} else if (bus) {
		map->read_flag_mask = bus->read_flag_mask;
	}

	if (!bus) {
		map->reg_read  = config->reg_read;
		map->reg_write = config->reg_write;

		map->defer_caching = false;
		goto skip_format_initialization;
	} else {
		map->reg_read  = _regmap_bus_read;
	}


	switch (config->reg_bits) {
	case 4:
		switch (config->val_bits) {
		case 12:
			map->format.format_write = regmap_format_4_12_write;
			break;
		default:
			goto err_map;
		}
		break;

	case 7:
		switch (config->val_bits) {
		case 9:
			map->format.format_write = regmap_format_7_9_write;
			break;
		default:
			goto err_map;
		}
		break;

	case 10:
		switch (config->val_bits) {
		case 14:
			map->format.format_write = regmap_format_10_14_write;
			break;
		default:
			goto err_map;
		}
		break;

	case 8:
		map->format.format_reg = regmap_format_8;
		break;

	case 16:
		map->format.format_reg = regmap_format_16;
		break;

	default:
		goto err_map;
	}

	switch (config->val_bits) {
	case 8:
		map->format.format_val = regmap_format_8;
		map->format.parse_val = regmap_parse_8;
		break;
	case 16:
		map->format.format_val = regmap_format_16;
		map->format.parse_val = regmap_parse_16;
		break;
	}

	if (!map->format.format_write &&
	    !(map->format.format_reg && map->format.format_val))
		goto err_map;

	map->work_buf = kmalloc(map->format.buf_size, GFP_KERNEL);
	if (map->work_buf == NULL) {
		ret = -ENOMEM;
		goto err_map;
	}

	if (map->format.format_write) {
		map->defer_caching = false;
		map->reg_write = _regmap_bus_formatted_write;
	} else if (map->format.format_val) {
		map->defer_caching = true;
		map->reg_write = _regmap_bus_raw_write;
	}

skip_format_initialization:
	regmap_debugfs_init(map);

	ret = regcache_init(map, config);
	if (ret < 0)
		goto err_free_workbuf;

	/* Add a devres resource for dev_get_regmap() */
	m = devres_alloc(dev_get_regmap_release, sizeof(*m), GFP_KERNEL);
	if (!m) {
		ret = -ENOMEM;
		goto err_cache;
	}
	*m = map;
	devres_add(dev, m);

	return map;

err_cache:
	regcache_exit(map);
err_free_workbuf:
	kfree(map->work_buf);
err_map:
	kfree(map);
err:
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(regmap_init);

static void devm_regmap_release(struct device *dev, void *res)
{
	regmap_exit(*(struct regmap **)res);
}

/**
 * devm_regmap_init(): Initialise managed register map
 *
 * @dev: Device that will be interacted with
 * @bus: Bus-specific callbacks to use with device
 * @config: Configuration for register map
 *
 * The return value will be an ERR_PTR() on error or a valid pointer
 * to a struct regmap.  This function should generally not be called
 * directly, it should be called by bus-specific init functions.  The
 * map will be automatically freed by the device management code.
 */
struct regmap *devm_regmap_init(struct device *dev,
				const struct regmap_bus *bus,
				const struct regmap_config *config)
{
	struct regmap **ptr, *regmap;

	ptr = devres_alloc(devm_regmap_release, sizeof(*ptr), GFP_KERNEL);
	if (!ptr)
		return ERR_PTR(-ENOMEM);

	regmap = regmap_init(dev, bus, config);
	if (!IS_ERR(regmap)) {
		*ptr = regmap;
		devres_add(dev, ptr);
	} else {
		devres_free(ptr);
	}

	return regmap;
}
EXPORT_SYMBOL_GPL(devm_regmap_init);

/**
 * regmap_reinit_cache(): Reinitialise the current register cache
 *
 * @map: Register map to operate on.
 * @config: New configuration.  Only the cache data will be used.
 *
 * Discard any existing register cache for the map and initialize a
 * new cache.  This can be used to restore the cache to defaults or to
 * update the cache configuration to reflect runtime discovery of the
 * hardware.
 */
int regmap_reinit_cache(struct regmap *map, const struct regmap_config *config)
{
	int ret;

	mutex_lock(&map->lock);

	regcache_exit(map);

	map->max_register = config->max_register;
	map->writeable_reg = config->writeable_reg;
	map->readable_reg = config->readable_reg;
	map->volatile_reg = config->volatile_reg;
	map->precious_reg = config->precious_reg;
	map->cache_type = config->cache_type;

	map->cache_bypass = false;
	map->cache_only = false;

	ret = regcache_init(map, config);

	mutex_unlock(&map->lock);

	return ret;
}

/**
 * regmap_exit(): Free a previously allocated register map
 */
void regmap_exit(struct regmap *map)
{
	regcache_exit(map);
	regmap_debugfs_exit(map);
	if (map->bus->free_context)
		map->bus->free_context(map->bus_context);
	kfree(map->work_buf);
	kfree(map);
}
EXPORT_SYMBOL_GPL(regmap_exit);

static int dev_get_regmap_match(struct device *dev, void *res, void *data)
{
	struct regmap **r = res;
	if (!r || !*r) {
		WARN_ON(!r || !*r);
		return 0;
	}

	/* If the user didn't specify a name match any */
	if (data)
		return (*r)->name == data;
	else
		return 1;
}

/**
 * dev_get_regmap(): Obtain the regmap (if any) for a device
 *
 * @dev: Device to retrieve the map for
 * @name: Optional name for the register map, usually NULL.
 *
 * Returns the regmap for the device if one is present, or NULL.  If
 * name is specified then it must match the name specified when
 * registering the device, if it is NULL then the first regmap found
 * will be used.  Devices with multiple register maps are very rare,
 * generic code should normally not need to specify a name.
 */
struct regmap *dev_get_regmap(struct device *dev, const char *name)
{
	struct regmap **r = devres_find(dev, dev_get_regmap_release,
					dev_get_regmap_match, (void *)name);

	if (!r)
		return NULL;
	return *r;
}
EXPORT_SYMBOL_GPL(dev_get_regmap);

static int _regmap_raw_write(struct regmap *map, unsigned int reg,
			     const void *val, size_t val_len)
{
	u8 *u8 = map->work_buf;
	void *buf;
	int ret = -ENOTSUPP;
	size_t len;
	int i;

	BUG_ON(!map->bus);

	/* Check for unwritable registers before we start */
	if (map->writeable_reg)
		for (i = 0; i < val_len / map->format.val_bytes; i++)
			if (!map->writeable_reg(map->dev, reg + i))
				return -EINVAL;

	map->format.format_reg(map->work_buf, reg);

	u8[0] |= map->write_flag_mask;

	trace_regmap_hw_write_start(map->dev, reg,
				    val_len / map->format.val_bytes);

	/* If we're doing a single register write we can probably just
	 * send the work_buf directly, otherwise try to do a gather
	 * write.
	 */
	if (val == map->work_buf + map->format.reg_bytes)
		ret = map->bus->write(map->bus_context, map->work_buf,
				      map->format.reg_bytes + val_len);
	else if (map->bus->gather_write)
		ret = map->bus->gather_write(map->bus_context, map->work_buf,
					     map->format.reg_bytes,
					     val, val_len);

	/* If that didn't work fall back on linearising by hand. */
	if (ret == -ENOTSUPP) {
		len = map->format.reg_bytes + val_len;
		buf = kmalloc(len, GFP_KERNEL);
		if (!buf)
			return -ENOMEM;

		memcpy(buf, map->work_buf, map->format.reg_bytes);
		memcpy(buf + map->format.reg_bytes, val, val_len);
		ret = map->bus->write(map->bus_context, buf, len);

		kfree(buf);
	}

	trace_regmap_hw_write_done(map->dev, reg,
				   val_len / map->format.val_bytes);

	return ret;
}

static int _regmap_bus_formatted_write(void *context, unsigned int reg,
				       unsigned int val)
{
	int ret;
	struct regmap *map = context;

	BUG_ON(!map->bus || !map->format.format_write);

	map->format.format_write(map, reg, val);

	trace_regmap_hw_write_start(map->dev, reg, 1);

	ret = map->bus->write(map->bus_context, map->work_buf,
			      map->format.buf_size);

	trace_regmap_hw_write_done(map->dev, reg, 1);

	return ret;
}

static int _regmap_bus_raw_write(void *context, unsigned int reg,
				 unsigned int val)
{
	struct regmap *map = context;

	BUG_ON(!map->bus || !map->format.format_val);

	map->format.format_val(map->work_buf + map->format.reg_bytes, val);
	return _regmap_raw_write(map, reg,
				 map->work_buf +
				 map->format.reg_bytes,
				 map->format.val_bytes);
}

static inline void *_regmap_map_get_context(struct regmap *map)
{
	return (map->bus) ? map : map->bus_context;
}

int _regmap_write(struct regmap *map, unsigned int reg,
		  unsigned int val)
{
	int ret;
	void *context = _regmap_map_get_context(map);

	if (!map->cache_bypass && !map->defer_caching) {
		ret = regcache_write(map, reg, val);
		if (ret != 0)
			return ret;
		if (map->cache_only) {
			map->cache_dirty = true;
			return 0;
		}
	}

	trace_regmap_reg_write(map->dev, reg, val);

	return map->reg_write(context, reg, val);
}

/**
 * regmap_write(): Write a value to a single register
 *
 * @map: Register map to write to
 * @reg: Register to write to
 * @val: Value to be written
 *
 * A value of zero will be returned on success, a negative errno will
 * be returned in error cases.
 */
int regmap_write(struct regmap *map, unsigned int reg, unsigned int val)
{
	int ret;

	mutex_lock(&map->lock);

	ret = _regmap_write(map, reg, val);

	mutex_unlock(&map->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(regmap_write);

/**
 * regmap_raw_write(): Write raw values to one or more registers
 *
 * @map: Register map to write to
 * @reg: Initial register to write to
 * @val: Block of data to be written, laid out for direct transmission to the
 *       device
 * @val_len: Length of data pointed to by val.
 *
 * This function is intended to be used for things like firmware
 * download where a large block of data needs to be transferred to the
 * device.  No formatting will be done on the data provided.
 *
 * A value of zero will be returned on success, a negative errno will
 * be returned in error cases.
 */
int regmap_raw_write(struct regmap *map, unsigned int reg,
		     const void *val, size_t val_len)
{
	size_t val_count = val_len / map->format.val_bytes;
	int ret;

	if (!map->bus)
		return -EINVAL;
	WARN_ON(!regmap_volatile_range(map, reg, val_count) &&
		map->cache_type != REGCACHE_NONE);

	mutex_lock(&map->lock);

	ret = _regmap_raw_write(map, reg, val, val_len);

	mutex_unlock(&map->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(regmap_raw_write);

static int _regmap_raw_read(struct regmap *map, unsigned int reg, void *val,
			    unsigned int val_len)
{
	u8 *u8 = map->work_buf;
	int ret;

	BUG_ON(!map->bus);

	map->format.format_reg(map->work_buf, reg);

	/*
	 * Some buses or devices flag reads by setting the high bits in the
	 * register addresss; since it's always the high bits for all
	 * current formats we can do this here rather than in
	 * formatting.  This may break if we get interesting formats.
	 */
	u8[0] |= map->read_flag_mask;

	trace_regmap_hw_read_start(map->dev, reg,
				   val_len / map->format.val_bytes);

	ret = map->bus->read(map->bus_context, map->work_buf,
			     map->format.reg_bytes,
			     val, val_len);

	trace_regmap_hw_read_done(map->dev, reg,
				  val_len / map->format.val_bytes);

	return ret;
}

static int _regmap_bus_read(void *context, unsigned int reg,
			   unsigned int *val)
{
	int ret;
	struct regmap *map = context;

	if (!map->format.parse_val)
		return -EINVAL;

	ret = _regmap_raw_read(map, reg, map->work_buf, map->format.val_bytes);
	if (ret == 0)
		*val = map->format.parse_val(map->work_buf);

	return ret;
}

static int _regmap_read(struct regmap *map, unsigned int reg,
			unsigned int *val)
{
	int ret;
	void *context = _regmap_map_get_context(map);

	BUG_ON(!map->reg_read);

	if (!map->cache_bypass) {
		ret = regcache_read(map, reg, val);
		if (ret == 0)
			return 0;
	}

	if (map->cache_only)
		return -EBUSY;

	ret = map->reg_read(context, reg, val);
	if (ret == 0)
		trace_regmap_reg_read(map->dev, reg, *val);

	return ret;
}

/**
 * regmap_read(): Read a value from a single register
 *
 * @map: Register map to write to
 * @reg: Register to be read from
 * @val: Pointer to store read value
 *
 * A value of zero will be returned on success, a negative errno will
 * be returned in error cases.
 */
int regmap_read(struct regmap *map, unsigned int reg, unsigned int *val)
{
	int ret;

	mutex_lock(&map->lock);

	ret = _regmap_read(map, reg, val);

	mutex_unlock(&map->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(regmap_read);

/**
 * regmap_raw_read(): Read raw data from the device
 *
 * @map: Register map to write to
 * @reg: First register to be read from
 * @val: Pointer to store read value
 * @val_len: Size of data to read
 *
 * A value of zero will be returned on success, a negative errno will
 * be returned in error cases.
 */
int regmap_raw_read(struct regmap *map, unsigned int reg, void *val,
		    size_t val_len)
{
	size_t val_count = val_len / map->format.val_bytes;
	int ret;

	if (!map->bus)
		return -EINVAL;
	WARN_ON(!regmap_volatile_range(map, reg, val_count) &&
		map->cache_type != REGCACHE_NONE);

	mutex_lock(&map->lock);

	ret = _regmap_raw_read(map, reg, val, val_len);

	mutex_unlock(&map->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(regmap_raw_read);

/**
 * regmap_bulk_read(): Read multiple registers from the device
 *
 * @map: Register map to write to
 * @reg: First register to be read from
 * @val: Pointer to store read value, in native register size for device
 * @val_count: Number of registers to read
 *
 * A value of zero will be returned on success, a negative errno will
 * be returned in error cases.
 */
int regmap_bulk_read(struct regmap *map, unsigned int reg, void *val,
		     size_t val_count)
{
	int ret, i;
	size_t val_bytes = map->format.val_bytes;
	bool vol = regmap_volatile_range(map, reg, val_count);

	if (!map->bus)
		return -EINVAL;
	if (!map->format.parse_val)
		return -EINVAL;

	if (vol || map->cache_type == REGCACHE_NONE) {
		ret = regmap_raw_read(map, reg, val, val_bytes * val_count);
		if (ret != 0)
			return ret;

		for (i = 0; i < val_count * val_bytes; i += val_bytes)
			map->format.parse_val(val + i);
	} else {
		for (i = 0; i < val_count; i++) {
			ret = regmap_read(map, reg + i, val + (i * val_bytes));
			if (ret != 0)
				return ret;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(regmap_bulk_read);

static int _regmap_update_bits(struct regmap *map, unsigned int reg,
			       unsigned int mask, unsigned int val,
			       bool *change)
{
	int ret;
	unsigned int tmp, orig;

	mutex_lock(&map->lock);

	ret = _regmap_read(map, reg, &orig);
	if (ret != 0)
		goto out;

	tmp = orig & ~mask;
	tmp |= val & mask;

	if (tmp != orig) {
		ret = _regmap_write(map, reg, tmp);
		*change = true;
	} else {
		*change = false;
	}

out:
	mutex_unlock(&map->lock);

	return ret;
}

/**
 * regmap_update_bits: Perform a read/modify/write cycle on the register map
 *
 * @map: Register map to update
 * @reg: Register to update
 * @mask: Bitmask to change
 * @val: New value for bitmask
 *
 * Returns zero for success, a negative number on error.
 */
int regmap_update_bits(struct regmap *map, unsigned int reg,
		       unsigned int mask, unsigned int val)
{
	bool change;
	return _regmap_update_bits(map, reg, mask, val, &change);
}
EXPORT_SYMBOL_GPL(regmap_update_bits);

/**
 * regmap_update_bits_check: Perform a read/modify/write cycle on the
 *                           register map and report if updated
 *
 * @map: Register map to update
 * @reg: Register to update
 * @mask: Bitmask to change
 * @val: New value for bitmask
 * @change: Boolean indicating if a write was done
 *
 * Returns zero for success, a negative number on error.
 */
int regmap_update_bits_check(struct regmap *map, unsigned int reg,
			     unsigned int mask, unsigned int val,
			     bool *change)
{
	return _regmap_update_bits(map, reg, mask, val, change);
}
EXPORT_SYMBOL_GPL(regmap_update_bits_check);

static int __init regmap_initcall(void)
{
	regmap_debugfs_initcall();

	return 0;
}
postcore_initcall(regmap_initcall);
