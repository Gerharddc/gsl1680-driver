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

#include "gsl1680.h"

#define DRV_NAME		"gsl1680"

/* I2C slave address */
#define fusion_F0710A_I2C_SLAVE_ADDR	0x40

#define gsl1680_XMAX	800
#define gsl1680_YMAX	480

/* gsl1680 touch screen information */
struct gsl1680_info {
	int xres; /* x resolution */
	int yres; /* y resolution */
	int xy_reverse; /* if need reverse in the x,y value x=xres-1-x, y=yres-1-y*/    
};

struct gsl1680_data {
	struct gsl1680_info		info;
	struct i2c_client		*client;
	struct workqueue_struct	*workq;
	struct input_dev		*input;
};


static struct gsl1680_data gsl1680;

static unsigned short normal_i2c[] = { gsl1680_I2C_SLAVE_ADDR, I2C_CLIENT_END };

//I2C_CLIENT_INSMOD;

static int gsl1680write_u8(u8 addr, u8 data) 
{
	return i2c_smbus_write_byte_data(gsl1680.client, addr, data);
}

static int gsl1680_read_u8(u8 addr)
{
	return i2c_smbus_read_byte_data(gsl1680.client, addr);
}

static int gsl1680_read_block(u8 addr, u8 len, u8 *data)
{
#if 0
	/* When i2c_smbus_read_i2c_block_data() takes a block length parameter, we can do
	 * this. lm-sensors lists hints this has been fixed, but I can't tell whether it
	 * was or will be merged upstream. */

	return i2c_smbus_read_i2c_block_data(&gsl1680.client, addr, data);
#else
	u8 msgbuf0[1] = { addr };
	u16 slave = gsl1680.client->addr;
	u16 flags = gsl1680.client->flags;
	struct i2c_msg msg[2] = { { slave, flags, 1, msgbuf0 },
				  { slave, flags | I2C_M_RD, len, data }
	};

	return i2c_transfer(gsl1680.client->adapter, msg, ARRAY_SIZE(msg));
#endif
}


static int fusion_F0710A_register_input(void)
{
	int ret;
	struct input_dev *dev;

	dev = gsl1680.input = input_allocate_device();
	if (dev == NULL)
		return -ENOMEM;

	dev->name = "gsl1680";

	set_bit(EV_KEY, dev->evbit);
	set_bit(EV_ABS, dev->evbit);

	input_set_abs_params(dev, ABS_MT_POSITION_X, 0, fusion_F0710A.info.xres-1, 0, 0);
	input_set_abs_params(dev, ABS_MT_POSITION_Y, 0, fusion_F0710A.info.yres-1, 0, 0);
#ifdef CONFIG_ANDROID
	input_set_abs_params(dev, ABS_MT_TRACKING_ID, 0, 15, 0, 0);
#else
	input_set_abs_params(dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(dev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
#endif

	input_set_abs_params(dev, ABS_X, 0, fusion_F0710A.info.xres-1, 0, 0);
	input_set_abs_params(dev, ABS_Y, 0, fusion_F0710A.info.yres-1, 0, 0);
	input_set_abs_params(dev, ABS_PRESSURE, 0, 255, 0, 0);

	ret = input_register_device(dev);
	if (ret < 0)
		goto bail1;

	return 0;

bail1:
	input_free_device(dev);
	return ret;
}

#define WC_RETRY_COUNT 		3
static int gsl1680_write_complete(void)
{
	int ret, i;

	for(i=0; i<WC_RETRY_COUNT; i++)
	{
		ret = gsl1680_write_u8(fusion_F0710A_SCAN_COMPLETE, 0);
		if(ret == 0)
			break;
		else
			dev_err(&fusion_F0710A.client->dev, "Write complete failed(%d): %d\n", i, ret);
	}

	return ret;
}

#define DATA_START	gsl1680_DATA_INFO
#define	DATA_END	gsl1680_SEC_TIDTS
#define DATA_LEN	(DATA_END - DATA_START + 1)
#define DATA_OFF(x)	((x) - DATA_START)

static int fusion_F0710A_read_sensor(void)
{
	int ret;
	u8 data[DATA_LEN];

#define DATA(x) (data[DATA_OFF(x)])
	/* To ensure data coherency, read the sensor with a single transaction. */
	ret = fusion_F0710A_read_block(DATA_START, DATA_LEN, data);
	if (ret < 0) {
		dev_err(&fusion_F0710A.client->dev,
			"Read block failed: %d\n", ret);
		
		return ret;
	}

	fusion_F0710A.f_num = DATA(fusion_F0710A_DATA_INFO)&0x03;
	
	fusion_F0710A.y1 = DATA(fusion_F0710A_POS_X1_HI) << 8;
	fusion_F0710A.y1 |= DATA(fusion_F0710A_POS_X1_LO);
	fusion_F0710A.x1 = DATA(fusion_F0710A_POS_Y1_HI) << 8;
	fusion_F0710A.x1 |= DATA(fusion_F0710A_POS_Y1_LO);
	fusion_F0710A.z1 = DATA(fusion_F0710A_FIR_PRESS);
	fusion_F0710A.tip1 = DATA(fusion_F0710A_FIR_TIDTS)&0x0f;
	fusion_F0710A.tid1 = (DATA(fusion_F0710A_FIR_TIDTS)&0xf0)>>4;
	
	
	fusion_F0710A.y2 = DATA(fusion_F0710A_POS_X2_HI) << 8;
	fusion_F0710A.y2 |= DATA(fusion_F0710A_POS_X2_LO);
	fusion_F0710A.x2 = DATA(fusion_F0710A_POS_Y2_HI) << 8;
	fusion_F0710A.x2 |= DATA(fusion_F0710A_POS_Y2_LO);
	fusion_F0710A.z2 = DATA(fusion_F0710A_SEC_PRESS);
	fusion_F0710A.tip2 = DATA(fusion_F0710A_SEC_TIDTS)&0x0f;
	fusion_F0710A.tid2 =(DATA(fusion_F0710A_SEC_TIDTS)&0xf0)>>4;
#undef DATA

	return 0;
}

#define val_cut_max(x, max, reverse)	\
do					\
{					\
	if(x > max)			\
		x = max;		\
	if(reverse)			\
		x = (max) - (x);	\
}					\
while(0)

static void fusion_F0710A_wq(struct work_struct *work)
{
	struct input_dev *dev = fusion_F0710A.input;
	int save_points = 0;
	int x1 = 0, y1 = 0, z1 = 0, x2 = 0, y2 = 0, z2 = 0;

	if (fusion_F0710A_read_sensor() < 0)
		goto restore_irq;

#ifdef DEBUG
	printk(KERN_DEBUG "tip1, tid1, x1, y1, z1 (%x,%x,%d,%d,%d); tip2, tid2, x2, y2, z2 (%x,%x,%d,%d,%d)\n",
		fusion_F0710A.tip1, fusion_F0710A.tid1, fusion_F0710A.x1, fusion_F0710A.y1, fusion_F0710A.z1,
		fusion_F0710A.tip2, fusion_F0710A.tid2, fusion_F0710A.x2, fusion_F0710A.y2, fusion_F0710A.z2);
#endif /* DEBUG */

	val_cut_max(fusion_F0710A.x1, fusion_F0710A.info.xres-1, fusion_F0710A.info.xy_reverse);
	val_cut_max(fusion_F0710A.y1, fusion_F0710A.info.yres-1, fusion_F0710A.info.xy_reverse);
	val_cut_max(fusion_F0710A.x2, fusion_F0710A.info.xres-1, fusion_F0710A.info.xy_reverse);
	val_cut_max(fusion_F0710A.y2, fusion_F0710A.info.yres-1, fusion_F0710A.info.xy_reverse);

	if(fusion_F0710A.tip1 == 1)
	{
		if(fusion_F0710A.tid1 == 1)
		{
			/* first point */
			x1 = fusion_F0710A.x1;
			y1 = fusion_F0710A.y1;
			z1 = fusion_F0710A.z1;
			save_points |= fusion_F0710A_SAVE_PT1;
		}
		else if(fusion_F0710A.tid1 == 2)
		{
			/* second point ABS_DISTANCE second point pressure, BTN_2 second point touch */
			x2 = fusion_F0710A.x1;
			y2 = fusion_F0710A.y1;
			z2 = fusion_F0710A.z1;
			save_points |= fusion_F0710A_SAVE_PT2;
		}
	}

	if(fusion_F0710A.tip2 == 1)
	{
		if(fusion_F0710A.tid2 == 2)
		{
			/* second point ABS_DISTANCE second point pressure, BTN_2 second point touch */
			x2 = fusion_F0710A.x2;
			y2 = fusion_F0710A.y2;
			z2 = fusion_F0710A.z2;
			save_points |= fusion_F0710A_SAVE_PT2;
		}
		else if(fusion_F0710A.tid2 == 1)/* maybe this will never happen */
		{
			/* first point */
			x1 = fusion_F0710A.x2;
			y1 = fusion_F0710A.y2;
			z1 = fusion_F0710A.z2;
			save_points |= fusion_F0710A_SAVE_PT1;
		}
	}

	input_report_abs(dev, ABS_MT_TOUCH_MAJOR, z1);
	input_report_abs(dev, ABS_MT_WIDTH_MAJOR, 1);
	input_report_abs(dev, ABS_MT_POSITION_X, x1);
	input_report_abs(dev, ABS_MT_POSITION_Y, y1);
	input_mt_sync(dev);
	input_report_abs(dev, ABS_MT_TOUCH_MAJOR, z2);
	input_report_abs(dev, ABS_MT_WIDTH_MAJOR, 2);
	input_report_abs(dev, ABS_MT_POSITION_X, x2);
	input_report_abs(dev, ABS_MT_POSITION_Y, y2);
	input_mt_sync(dev);

	input_report_abs(dev, ABS_X, x1);
	input_report_abs(dev, ABS_Y, y1);
	input_report_abs(dev, ABS_PRESSURE, z1);
	input_report_key(dev, BTN_TOUCH, fusion_F0710A.tip1);

	input_sync(dev);

restore_irq:
	enable_irq(gsl1680.client->irq);

	/* Clear gsl1680 interrupt */
	gsl1680_write_complete();
}
static DECLARE_WORK(gsl1680_work, gsl1680_wq);

static irqreturn_t gsl1680_interrupt(int irq, void *dev_id)
{
	disable_irq_nosync(gsl1680.client->irq);

	queue_work(gsl1680.workq, &gsl1680_work);

	return IRQ_HANDLED;
}

const static u8* g_ver_product[4] = {
	"10Z8", "70Z7", "43Z6", ""
};

static int gsl1680_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct gsl1680_init_data *pdata = i2c->dev.platform_data;
	int ret;
	u8 ver_product, ver_id;
	u32 version;

	if (pdata == NULL)
	{
		dev_err(&i2c->dev, "No platform data for Fusion driver\n");
		return -ENODEV;
	}

	/* Request pinmuxing, if necessary */
	if (pdata->pinmux_fusion_pins != NULL)
	{
		ret = pdata->pinmux_fusion_pins();
		if (ret < 0) {
			dev_err(&i2c->dev, "muxing GPIOs failed\n");
			return -ENODEV;
		}
	}

	if ((gpio_request(pdata->gpio_int, "Fusion pen down interrupt") == 0) &&
	    (gpio_direction_input(pdata->gpio_int) == 0)) {
		gpio_export(pdata->gpio_int, 0);
	} else {
		dev_warn(&i2c->dev, "Could not obtain GPIO for Fusion pen down\n");
		return -ENODEV;
	}

	if ((gpio_request(pdata->gpio_reset, "Fusion reset") == 0) &&
	    (gpio_direction_output(pdata->gpio_reset, 1) == 0)) {

		/* Generate a 0 => 1 edge explicitly, and wait for startup... */
		gpio_set_value(pdata->gpio_reset, 0);
		msleep(10);
		gpio_set_value(pdata->gpio_reset, 1);
		/* Wait for startup (up to 125ms according to datasheet) */
		msleep(125);

		gpio_export(pdata->gpio_reset, 0);
	} else {
		dev_warn(&i2c->dev, "Could not obtain GPIO for Fusion reset\n");
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
			i2c_adapter_id(gsl1680.client->adapter),	gsl1680.client->addr);

	/* Read out a lot of registers */
	ret = gsl1680_read_u8(gsl1680_VIESION_INFO_LO);
	if (ret < 0) {
		dev_err(&i2c->dev, "query failed: %d\n", ret);
		goto bail1;
	}
	ver_product = (((u8)ret) & 0xc0) >> 6;
	version = (10 + ((((u32)ret)&0x30) >> 4)) * 100000;
	version += (((u32)ret)&0xf) * 1000;
	/* Read out a lot of registers */
	ret = gsl1680_read_u8(gsl1680_VIESION_INFO);
		if (ret < 0) {
		dev_err(&i2c->dev, "query failed: %d\n", ret);
		goto bail1;
	}
	ver_id = ((u8)(ret) & 0x6) >> 1;
	version += ((((u32)ret) & 0xf8) >> 3) * 10;
	version += (((u32)ret) & 0x1) + 1; /* 0 is build 1, 1 is build 2 */
	dev_info(&i2c->dev, "version product %s(%d)\n", g_ver_product[ver_product] ,ver_product);
	dev_info(&i2c->dev, "version id %s(%d)\n", ver_id ? "1.4" : "1.0", ver_id);
	dev_info(&i2c->dev, "version series (%d)\n", version);

	switch(ver_product)
	{
	case fusion_F0710A_VIESION_07: /* 7 inch */
		fusion_F0710A.info.xres = fusion_F0710A07_XMAX;
		fusion_F0710A.info.yres = fusion_F0710A07_YMAX;
		fusion_F0710A.info.xy_reverse = fusion_F0710A07_REV;
		break;
	case fusion_F0710A_VIESION_43: /* 4.3 inch */
		fusion_F0710A.info.xres = fusion_F0710A43_XMAX;
		fusion_F0710A.info.yres = fusion_F0710A43_YMAX;
		fusion_F0710A.info.xy_reverse = fusion_F0710A43_REV;
		break;
	default: /* fusion_F0710A_VIESION_10 10 inch */
		fusion_F0710A.info.xres = fusion_F0710A10_XMAX;
		fusion_F0710A.info.yres = fusion_F0710A10_YMAX;
		fusion_F0710A.info.xy_reverse = fusion_F0710A10_REV;
		break;
	}

	/* Register the input device. */
	ret = fusion_F0710A_register_input();
	if (ret < 0) {
		dev_err(&i2c->dev, "can't register input: %d\n", ret);
		goto bail1;
	}

	/* Create a worker thread */
	fusion_F0710A.workq = create_singlethread_workqueue(DRV_NAME);
	if (fusion_F0710A.workq == NULL) {
		dev_err(&i2c->dev, "can't create work queue\n");
		ret = -ENOMEM;
		goto bail2;
	}


	/* Register for the interrupt and enable it. Our handler will
	*  start getting invoked after this call. */
	ret = request_irq(i2c->irq, fusion_F0710A_interrupt, IRQF_TRIGGER_RISING,
	i2c->name, &fusion_F0710A);
	if (ret < 0) {
		dev_err(&i2c->dev, "can't get irq %d: %d\n", i2c->irq, ret);
		goto bail3;
	}
	/* clear the irq first */
	ret = fusion_F0710A_write_u8(fusion_F0710A_SCAN_COMPLETE, 0);
	if (ret < 0) {
		dev_err(&i2c->dev, "Clear irq failed: %d\n", ret);
		goto bail4;
	}

	return 0;

bail4:
	free_irq(i2c->irq, &fusion_F0710A);

bail3:
	destroy_workqueue(fusion_F0710A.workq);
	fusion_F0710A.workq = NULL;

bail2:
	input_unregister_device(fusion_F0710A.input);
bail1:
	gpio_free(pdata->gpio_reset);
bail0:
	gpio_free(pdata->gpio_int);

	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int fusion_F0710A_suspend(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	disable_irq(i2c->irq);
	flush_workqueue(fusion_F0710A.workq);

	return 0;
}

static int fusion_F0710A_resume(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	enable_irq(i2c->irq);

	return 0;
}
#endif

static int fusion_F0710A_remove(struct i2c_client *i2c)
{
	struct fusion_f0710a_init_data *pdata = i2c->dev.platform_data;

	gpio_free(pdata->gpio_int);
	gpio_free(pdata->gpio_reset);
	destroy_workqueue(fusion_F0710A.workq);
	free_irq(i2c->irq, &fusion_F0710A);
	input_unregister_device(fusion_F0710A.input);
	i2c_set_clientdata(i2c, NULL);

	dev_info(&i2c->dev, "driver removed\n");
	
	return 0;
}

static struct i2c_device_id fusion_F0710A_id[] = {
	{"fusion_F0710A", 0},
	{},
};

static const struct dev_pm_ops fusion_F0710A_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(fusion_F0710A_suspend, fusion_F0710A_resume)
};

static struct i2c_driver fusion_F0710A_i2c_drv = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= DRV_NAME,
		.pm		= &fusion_F0710A_pm_ops,
	},
	.probe          = fusion_F0710A_probe,
	.remove         = fusion_F0710A_remove,
	.id_table       = fusion_F0710A_id,
	.address_list   = normal_i2c,
};

static int __init gsl1680_init( void )
{
	int ret;

	memset(&fusion_F0710A, 0, sizeof(fusion_F0710A));

	/* Probe for gsl1680 on I2C. */
	ret = i2c_add_driver(&fusion_F0710A_i2c_drv);
	if (ret < 0) {
		printk(KERN_WARNING DRV_NAME " can't add i2c driver: %d\n", ret);
	}

	return ret;
}

static void __exit gsl1680_exit( void )
{
	i2c_del_driver(&fusion_F0710A_i2c_drv);
}
module_init(gsl1680_init);
module_exit(gsl1680_exit);

MODULE_DESCRIPTION("GSL1680 Touchscreen Driver");
MODULE_LICENSE("GPL");