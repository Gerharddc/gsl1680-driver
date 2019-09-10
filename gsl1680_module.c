/*
 *  "gsl1680"  touchscreen driver
 *	
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <linux/gpio.h>
#include <linux/input/gsl1680.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/input/mt.h>
#include <linux/of_gpio.h>
#include <linux/firmware.h>

#define DRV_NAME		"gsl1680"

/* I2C slave address */
#define gsl1680_I2C_SLAVE_ADDR	0x40

#define gsl1680_DATA_REG	0x80
#define gsl1680_STATUS_REG	0xe0

#define gsl1680_XMAX	800
#define gsl1680_YMAX	480
#define gsl1680_REV		0

#define gsl1680_MAX_POINTS	10

//#define gsl1680_USE_PROTOCOL_B

#define FIRMWARE_GSL1680 "gsl1680.bin"
MODULE_FIRMWARE(FIRMWARE_GSL1680);

/* gsl1680 touch screen information */
struct gsl1680_info {
	int xres; /* x resolution */
	int yres; /* y resolution */
	int xy_reverse; /* if need reverse in the x,y value x=xres-1-x, y=yres-1-y*/
	unsigned int_gpio; // The interrupt pin    
};

struct gsl1680_data {
	struct gsl1680_info	info;
	struct i2c_client	*client;
	struct workqueue_struct	*workq;
	struct input_dev	*input;
	int	x_vals[gsl1680_MAX_POINTS];
	int	y_vals[gsl1680_MAX_POINTS];
	int ids[gsl1680_MAX_POINTS];
	u8 fingers;

#ifdef gsl1680_USE_PROTOCOL_B
	int prev_ids[gsl1680_MAX_POINTS];
	u8 prev_fingers;
#endif
};


static struct gsl1680_data gsl1680;

//I2C_CLIENT_INSMOD;

static int gsl1680_write_u8(u8 addr, u8 data) 
{
	return i2c_smbus_write_byte_data(gsl1680.client, addr, data);
}

static int gsl1680_write_block(u8 addr, u8 *data, int len) 
{
	return i2c_smbus_write_block_data(gsl1680.client, addr, len, data);
}

static int gsl1680_read_block(u8 addr, u8 *data)
{
	return i2c_smbus_read_block_data(gsl1680.client, addr, data);
}

static int gsl1680_clr_reg(void)
{
	int ret;

	ret = gsl1680_write_u8(0xe0, 0x88);
	if (ret < 0) return ret;
	msleep(20);

	ret = gsl1680_write_u8(0x80, 0x01);
	if (ret < 0) return ret;
	msleep(5);

	ret = gsl1680_write_u8(0xe4, 0x04);
	if (ret < 0) return ret;
	msleep(5);

	ret = gsl1680_write_u8(0xe0, 0x00);
	if (ret < 0) return ret;
	msleep(20);

	return 0;
}

static int gsl1680_reset_chip(void)
{
	u8 i;
	int ret;

	ret = gsl1680_write_u8(gsl1680_STATUS_REG, 0x88);
	if (ret < 0) return ret;
	msleep(10);

	ret = gsl1680_write_u8(0xe4, 0x04);
	if (ret < 0) return ret;
	msleep(10);

	for (i = 0; i < 4; i++)
	{
		ret = gsl1680_write_u8(0xbc, 0x00);
		if (ret < 0) return ret;
		msleep(10);
	}

	return 0;
}

static inline int gsl1680_load_fw(void)
{
	int ret;
	const struct firmware *fw;
	u8 addr;
	u8 Wrbuf[4];
	u32 source_line = 0;
	u32 source_len;
	u32 cur_byte_pos = 0;

	printk(KERN_INFO "GSL1680 firmware being loaded.\n");
	ret = request_firmware(&fw, FIRMWARE_GSL1680, &gsl1680.input->dev);
	if (ret) 
	{
        printk(KERN_ERR "Failed to load %s, %d.\n", FIRMWARE_GSL1680, ret);
        return -EINVAL;
    }

	source_len = fw->size / 5;

	for (source_line = 0; source_line < source_len; source_line++) {
		cur_byte_pos = source_line * 5;
		addr = fw->data[cur_byte_pos];
		Wrbuf[0] = fw->data[cur_byte_pos + 1];
		Wrbuf[1] = fw->data[cur_byte_pos + 2];
		Wrbuf[2] = fw->data[cur_byte_pos + 3];
		Wrbuf[3] = fw->data[cur_byte_pos + 4];

        ret = gsl1680_write_block(addr, Wrbuf, 4);
        if (ret < 0)
        {
        	printk(KERN_ERR "Failed to upload firmware %d.\n", ret);
        	return ret;
        }
    }

    printk(KERN_INFO "GSL1680 firmware loaded.\n");

	release_firmware(fw);

	return 0;
}

static inline int gsl1680_startup_chip(void)
{
	int ret;

    ret = gsl1680_write_u8(0xe0, 0x00);
    if (ret < 0) return ret;

    return 0;
}

static inline void gsl1680_toggle_chip(unsigned gpio_reset)
{
	printk(KERN_INFO "Toggle GSL1680 wake.\n");
	gpio_set_value(gpio_reset, 1);
	msleep(50);
	gpio_set_value(gpio_reset, 0);
	msleep(50);
	gpio_set_value(gpio_reset, 1);
	msleep(30);
}

static inline int gsl1680_init_chip(void)
{
	int ret;

	// CTP startup sequence
	printk(KERN_INFO "GSL1680 clr reg.\n");
	ret = gsl1680_clr_reg();
	if (ret != 0) return ret;

	printk(KERN_INFO "GSL1680 reset chip.\n");
	ret = gsl1680_reset_chip();
	if (ret != 0) return ret;

	printk(KERN_INFO "GSL1680 load fw.\n");
	ret = gsl1680_load_fw();
	if (ret != 0) return ret;

	printk(KERN_INFO "GSL1680 reset chip 2.\n");
	ret = gsl1680_reset_chip();
	if (ret != 0) return ret;

	printk(KERN_INFO "GSL1680 startup chip.\n");
	ret = gsl1680_startup_chip();
	if (ret != 0) return ret;

	printk(KERN_INFO "Init done.\n");
	return 0;
}

static int gsl1680_read_sensor(void)
{
	int ret;
	u8 i;
	u8 touch_data[24] = {0};

	ret = gsl1680_read_block(gsl1680_DATA_REG, touch_data);
	if (ret < 0) {
		dev_err(&gsl1680.client->dev,
			"Read block failed: %d\n", ret);
		
		return ret;
	}

	gsl1680.fingers = touch_data[0];
	for (i = 0; i < gsl1680.fingers; i++)
	{
		gsl1680.x_vals[i] = ( (((u32)touch_data[(i*4)+5])<<8) | (u32)touch_data[(i*4)+4] ) & 0x00000FFF; // 12 bits of X coord
		gsl1680.y_vals[i] = ( (((u32)touch_data[(i*4)+7])<<8) | (u32)touch_data[(i*4)+6] ) & 0x00000FFF;
		gsl1680.ids[i] = (u32)touch_data[(i*4)+7] >> 4; // finger that did the touch
	}

	return 0;
}

static int gsl1680_register_input(void)
{
	int ret;
	struct input_dev *dev;

	dev = gsl1680.input = input_allocate_device();
	if (dev == NULL)
		return -ENOMEM;

	dev->name = "GSL1680 Touch Screen";

	set_bit(EV_KEY, dev->evbit);
	set_bit(EV_ABS, dev->evbit);
	set_bit(BTN_TOUCH, dev->keybit);

	input_set_abs_params(dev, ABS_MT_POSITION_X, 0, gsl1680.info.xres-1, 0, 0);
	input_set_abs_params(dev, ABS_MT_POSITION_Y, 0, gsl1680.info.yres-1, 0, 0);
	input_set_abs_params(dev, ABS_X, 0, gsl1680.info.xres-1, 0, 0);
	input_set_abs_params(dev, ABS_Y, 0, gsl1680.info.yres-1, 0, 0);

	ret = input_register_device(dev);
	if (ret < 0)
		goto bail1;

	return 0;

bail1:
	input_free_device(dev);
	return ret;
}

static void gsl1680_wq(struct work_struct *work)
{
	struct input_dev *dev = gsl1680.input;
	u8 i = 0;

	if (gsl1680_read_sensor() < 0)
		goto restore_irq;

#ifdef gsl1680_USE_PROTOCOL_B
	for (i = 0; i < gsl1680.fingers; i++)
	{
		input_mt_slot(dev, gsl1680.ids[i]);
	}

	//for (i = 0; i < gsl1680.prev_ids)

	//gsl1680.prev_ids = gsl1680.ids;
#else // PROTOCOL A
	for (i = 0; i < gsl1680.fingers; i++)
	{
		input_report_abs(dev, ABS_MT_POSITION_X, gsl1680.x_vals[i]);
		input_report_abs(dev, ABS_MT_POSITION_Y, gsl1680.y_vals[i]);
		input_mt_sync(dev);
	}

	// Report an empty round
	if (gsl1680.fingers == 0)
		input_mt_sync(dev);
#endif

	input_sync(dev);
  

restore_irq:
	enable_irq(gsl1680.client->irq);
	// Clear the interrupt if needed
	if (gpio_get_value(gsl1680.info.int_gpio) == 1)
		gsl1680_read_sensor();
}

static DECLARE_WORK(gsl1680_work, gsl1680_wq);

static irqreturn_t gsl1680_interrupt(int irq, void *dev_id)
{
	disable_irq_nosync(gsl1680.client->irq);

	queue_work(gsl1680.workq, &gsl1680_work);

	return IRQ_HANDLED;
}

static int gsl1680_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct gsl1680_init_data *pdata = i2c->dev.platform_data;
	int ret;

	if (pdata == NULL)
	{
		dev_err(&i2c->dev, "No platform data for GSL1680 driver\n");
		return -ENODEV;
	}

	/* Request pinmuxing, if necessary */
	if (pdata->pinmux_gsl_pins != NULL)
	{
		ret = pdata->pinmux_gsl_pins();
		if (ret < 0) {
			dev_err(&i2c->dev, "muxing GPIOs failed\n");
			return -ENODEV;
		}
	}

	if ((gpio_request(pdata->gpio_int, "GPIO pen down interrupt") == 0) &&
	    (gpio_direction_input(pdata->gpio_int) == 0)) {
		gpio_export(pdata->gpio_int, 0);
	} else {
		dev_warn(&i2c->dev, "Could not obtain GPIO for GPIO pen down\n");
		return -ENODEV;
	}

	if ((gpio_request(pdata->gpio_reset, "GSL1680 reset") == 0) &&
	    (gpio_direction_output(pdata->gpio_reset, 1) == 0)) {

		// Toggle the chip to get it active
		gsl1680_toggle_chip(pdata->gpio_reset);

		gpio_export(pdata->gpio_reset, 0);
	} else {
		dev_warn(&i2c->dev, "Could not obtain GPIO for GSL1680 reset\n");
		ret = -ENODEV;
		goto bail0;
	}

	/* Use Pen Down GPIO as sampling interrupt */
	i2c->irq = gpio_to_irq(pdata->gpio_int);

	if(!i2c->irq)
	{
		dev_err(&i2c->dev, "gsl1680 irq < 0 \n");
		ret = -ENOMEM;
		goto bail1;
	}

	/* Attach the I2C client */
	gsl1680.client =  i2c;
	i2c_set_clientdata(i2c, &gsl1680);

	dev_info(&i2c->dev, "Touchscreen registered with bus id (%d) with slave address 0x%x\n",
			i2c_adapter_id(gsl1680.client->adapter), gsl1680.client->addr);

	gsl1680.info.xres = gsl1680_YMAX;
	gsl1680.info.yres = gsl1680_YMAX;
	gsl1680.info.xy_reverse = gsl1680_REV;
	gsl1680.info.int_gpio = pdata->gpio_int;

	// Try to initialize the chip
	ret = gsl1680_init_chip();
	if (ret != 0)
	{
		dev_err(&i2c->dev, "can't init gsl1680: %d\n", ret);
		goto bail1;
	}

	/* Register the input device. */
	ret = gsl1680_register_input();
	if (ret < 0) 
	{
		dev_err(&i2c->dev, "can't register input: %d\n", ret);
		goto bail1;
	}

	/* Create a worker thread */
	gsl1680.workq = create_singlethread_workqueue(DRV_NAME);
	if (gsl1680.workq == NULL) 
	{
		dev_err(&i2c->dev, "can't create work queue\n");
		ret = -ENOMEM;
		goto bail2;
	}


	/* Register for the interrupt and enable it. Our handler will
	*  start getting invoked after this call. */
	ret = request_irq(i2c->irq, gsl1680_interrupt, IRQF_TRIGGER_RISING,
	i2c->name, &gsl1680);
	if (ret < 0) 
	{
		dev_err(&i2c->dev, "can't get irq %d: %d\n", i2c->irq, ret);
		goto bail3;
	}

	return 0;

bail3:
	destroy_workqueue(gsl1680.workq);
	gsl1680.workq = NULL;

bail2:
	input_unregister_device(gsl1680.input);

bail1:
	gpio_free(pdata->gpio_reset);
	
bail0:
	gpio_free(pdata->gpio_int);

	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int gsl1680_suspend(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	disable_irq(i2c->irq);
	flush_workqueue(gsl1680.workq);
	// TODO: sleep the IC	

	return 0;
}

static int gsl1680_resume(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	enable_irq(i2c->irq);
	// TODO: wake the IC

	return 0;
}
#endif

static int gsl1680_remove(struct i2c_client *i2c)
{
	struct gsl1680_init_data *pdata = i2c->dev.platform_data;

	gpio_free(pdata->gpio_int);
	gpio_free(pdata->gpio_reset);
	destroy_workqueue(gsl1680.workq);
	free_irq(i2c->irq, &gsl1680);
	input_unregister_device(gsl1680.input);
	i2c_set_clientdata(i2c, NULL);

	dev_info(&i2c->dev, "driver removed\n");

	return 0;
}

static struct i2c_device_id gsl1680_id[] = {
	{"gsl1680", 0},
	{},
};

static const struct dev_pm_ops gsl1680_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(gsl1680_suspend, gsl1680_resume)
};

static struct i2c_driver gsl1680_i2c_drv = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= DRV_NAME,
		.pm		= &gsl1680_pm_ops,
	},
	.probe          = gsl1680_probe,
	.remove         = gsl1680_remove,
	.id_table       = gsl1680_id,
};

static int __init gsl1680_init(void)
{
	int ret;

	memset(&gsl1680, 0, sizeof(gsl1680));

	/* Probe for gsl1680 on I2C. */
	ret = i2c_add_driver(&gsl1680_i2c_drv);
	if (ret < 0) {
		printk(KERN_WARNING DRV_NAME " can't add i2c driver: %d\n", ret);
	}

	return ret;
}

static void __exit gsl1680_exit(void)
{
	i2c_del_driver(&gsl1680_i2c_drv);
}

module_init(gsl1680_init);
module_exit(gsl1680_exit);

MODULE_AUTHOR("Gerhard de Clercq");
MODULE_DESCRIPTION("GSL1680 Touchscreen Driver");
MODULE_LICENSE("GPL");