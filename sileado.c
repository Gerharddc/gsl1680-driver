/* -------------------------------------------------------------------------
 * Copyright (C) 2015, Gerhard de Clercq trading as de Clercq Systems
 * 
 * Derived from:
 *  silead.c
 *  Copyright (C) 2014-2015, Intel Corporation
 *
 *  Derived from:
 *   gslX68X.c
 *   Copyright (C) 2010-2015, Shanghai Sileadinc Co.Ltd
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 * ------------------------------------------------------------------------- */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/pm.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/input/gsl1680.h>

#define SILEAD_TS_NAME "Silead Touch Screen"
#define DRVNAME "silead_ts"

#define SILEAD_REG_RESET   0xE0
#define SILEAD_REG_DATA        0x80
#define SILEAD_REG_TOUCH_NR    0x80
#define SILEAD_REG_POWER   0xBC
#define SILEAD_REG_CLOCK   0xE4
#define SILEAD_REG_STATUS  0xB0
#define SILEAD_REG_ID      0xFC
#define SILEAD_REG_MEM_CHECK   0xB0

#define SILEAD_STATUS_OK   0x5A5A5A5A
#define SILEAD_TS_DATA_LEN 44
#define SILEAD_CLOCK       0x04

#define SILEAD_CMD_RESET   0x88
#define SILEAD_CMD_START   0x00

#define SILEAD_POINT_DATA_LEN 0x04
#define SILEAD_POINT_Y_OFF      0x00
#define SILEAD_POINT_Y_MSB_OFF 0x01
#define SILEAD_POINT_X_OFF 0x02
#define SILEAD_POINT_X_MSB_OFF 0x03
#define SILEAD_POINT_HSB_MASK  0x0F
#define SILEAD_TOUCH_ID_MASK   0xF0

#define SILEAD_CMD_SLEEP_MIN   10000
#define SILEAD_CMD_SLEEP_MAX   20000
#define SILEAD_POWER_SLEEP     20
#define SILEAD_STARTUP_SLEEP   30

#define FIRMWARE_GSL1680 "gsl1680.bin"
MODULE_FIRMWARE(FIRMWARE_GSL1680);

enum silead_ts_power {
   SILEAD_POWER_ON  = 1,
   SILEAD_POWER_OFF = 0
};

struct silead_ts_data {
   struct i2c_client *client;
   unsigned gpio_power;
   unsigned gpio_irq;
   struct input_dev *input;
   const char *custom_fw_name;
   char fw_name[I2C_NAME_SIZE];
   u16 x_max;
   u16 y_max;
   u8 max_fingers;
   u32 chip_id;
   bool sent_empty_last;

   bool slot_pressed[5];
};

static int silead_ts_request_input_dev(struct silead_ts_data *data)
{
    struct input_dev *dev;
    int ret;

    dev = data->input = input_allocate_device();
    if (dev == NULL) {
        dev_err(&data->client->dev, "Failed to allocate input device\n");
        return -1;
    }

    set_bit(EV_KEY, dev->evbit);
    set_bit(EV_ABS, dev->evbit);
    set_bit(BTN_TOUCH, dev->keybit);
    set_bit(ABS_X, dev->evbit);
    set_bit(ABS_Y, dev->evbit);
    set_bit(ABS_PRESSURE, dev->evbit);

    input_set_abs_params(dev, ABS_MT_POSITION_X, 0, data->x_max, 0, 0);
    input_set_abs_params(dev, ABS_MT_POSITION_Y, 0, data->y_max, 0, 0);
    input_set_abs_params(dev, ABS_X, 0, data->x_max, 0, 0);
    input_set_abs_params(dev, ABS_Y, 0, data->y_max, 0, 0);
    input_set_abs_params(dev, ABS_PRESSURE, 0, 10, 0, 0);
    input_set_abs_params(dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(dev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);

    input_set_drvdata(dev, data);

    input_mt_init_slots(data->input, data->max_fingers);

    data->input->name = SILEAD_TS_NAME;
    data->input->id.bustype = BUS_I2C;

    ret = input_register_device(data->input);
    if (ret) {
        dev_err(&data->client->dev, "Failed to register input device: %d\n", ret);
        return ret;
    }

    return 0;
}

static void silead_ts_report_touch(struct silead_ts_data *data, u16 x, u16 y, u8 id)
{
    input_mt_slot(data->input, id);
    input_mt_report_slot_state(data->input, MT_TOOL_FINGER, true);
    input_report_abs(data->input, ABS_MT_POSITION_X, x);
    input_report_abs(data->input, ABS_MT_POSITION_Y, y);
}

static void silead_ts_set_power(struct i2c_client *client,
               enum silead_ts_power state)
{
   struct silead_ts_data *data = i2c_get_clientdata(client);

   gpio_set_value(data->gpio_power, state);
   msleep(SILEAD_POWER_SLEEP);
}

static void silead_ts_read_data(struct i2c_client *client)
{
   struct silead_ts_data *data = i2c_get_clientdata(client);
   struct device *dev = &client->dev;
   u8 buf[24];
   int x, y, touch_nr, ret, i, id;
   bool slot_pressed[5];
   bool should_sync = false;

   ret = i2c_smbus_read_i2c_block_data(client, SILEAD_REG_DATA, 24, buf);
   if (ret < 0) {
       dev_err(dev, "Data read error %d\n", ret);
       return;
   }

   touch_nr = buf[0];

   if (touch_nr < 0)
       return;

   dev_dbg(dev, "Touch number: %d\n", touch_nr);

   for (i = 0; i < 5; i++) {
    slot_pressed[i] = false;
   }

   for (i = 0; i < touch_nr; i++) {
        x = ( (((u32)buf[(i*4)+5])<<8) | (u32)buf[(i*4)+4] ) & 0x00000FFF;
        y = ( (((u32)buf[(i*4)+7])<<8) | (u32)buf[(i*4)+6] ) & 0x00000FFF;
        id = ((u32)buf[(i*4)+7] >> 4) - 1; // We need to start at 0
        silead_ts_report_touch(data, x, y, id);
        slot_pressed[id] = true;
        should_sync = true;
   }

    for (i = 0; i < 5; i++)
    {
        // Check if any points were releasesd
        if (data->slot_pressed[i] && !slot_pressed[i])
        {
            input_mt_slot(data->input, i);
            input_mt_report_slot_state(data->input, MT_TOOL_FINGER, false);
            should_sync = true;
            //dev_info(dev, "Released %d", i);
        }

        data->slot_pressed[i] = slot_pressed[i];
    }

    if (should_sync)
    {
        input_mt_sync(data->input);

        input_sync(data->input);
    }
}

static int silead_ts_init(struct i2c_client *client)
{
   struct silead_ts_data *data = i2c_get_clientdata(client);
   int ret;

   ret = i2c_smbus_write_byte_data(client, SILEAD_REG_RESET,
                   SILEAD_CMD_RESET);
   if (ret)
       goto i2c_write_err;
   usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

   ret = i2c_smbus_write_byte_data(client, SILEAD_REG_TOUCH_NR,
                   data->max_fingers);
   if (ret)
       goto i2c_write_err;
   usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

   ret = i2c_smbus_write_byte_data(client, SILEAD_REG_CLOCK, SILEAD_CLOCK);
   if (ret)
       goto i2c_write_err;
   usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

   ret = i2c_smbus_write_byte_data(client, SILEAD_REG_RESET,
                   SILEAD_CMD_START);
   if (ret)
       goto i2c_write_err;
   usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

   return 0;

i2c_write_err:
   dev_err(&client->dev, "Registers clear error %d\n", ret);
   return ret;
}

static int silead_ts_reset(struct i2c_client *client)
{
   int ret;

   ret = i2c_smbus_write_byte_data(client, SILEAD_REG_RESET,
                   SILEAD_CMD_RESET);
   if (ret)
       goto i2c_write_err;
   usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

   ret = i2c_smbus_write_byte_data(client, SILEAD_REG_CLOCK, SILEAD_CLOCK);
   if (ret)
       goto i2c_write_err;
   usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

   ret = i2c_smbus_write_byte_data(client, SILEAD_REG_POWER,
                   SILEAD_CMD_START);
   if (ret)
       goto i2c_write_err;
   usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

   return 0;

i2c_write_err:
   dev_err(&client->dev, "Chip reset error %d\n", ret);
   return ret;
}

static int silead_ts_startup(struct i2c_client *client)
{
   int ret;

   ret = i2c_smbus_write_byte_data(client, SILEAD_REG_RESET, 0x00);
   if (ret) {
       dev_err(&client->dev, "Startup error %d\n", ret);
       return ret;
   }
   msleep(SILEAD_STARTUP_SLEEP);

   return 0;
}

static int silead_ts_load_fw(struct i2c_client *client)
{
    int ret;
    const struct firmware *fw;
    u32 i;
    u32 source_len;
    u8 addr;
    u8 Wrbuf[4];
    struct device *dev = &client->dev;
    u32 cur_byte_pos = 0;

    printk(KERN_INFO "GSL1680 firmware being loaded.\n");
    ret = request_firmware(&fw, FIRMWARE_GSL1680, dev);
    if (ret) 
    {
        printk(KERN_ERR "Failed to load %s, %d.\n", FIRMWARE_GSL1680, ret);
        return -EINVAL;
    }

    source_len = fw->size / 5;

    for (i = 0; i < source_len; i++) {
        cur_byte_pos = i * 5;
        addr = fw->data[cur_byte_pos];
        Wrbuf[0] = fw->data[cur_byte_pos + 1];
        Wrbuf[1] = fw->data[cur_byte_pos + 2];
        Wrbuf[2] = fw->data[cur_byte_pos + 3];
        Wrbuf[3] = fw->data[cur_byte_pos + 4];

        ret = i2c_smbus_write_i2c_block_data(client, addr, 4, Wrbuf);

        if (ret) {
            dev_err(dev, "Firmware load error %d\n", ret);
            goto release_fw_err;
        }
    }

    dev_info(dev, " Loaded fw of size:%d \n", i);

    release_firmware(fw);
    return 0;

release_fw_err:
   release_firmware(fw);
   return ret;
}

static u32 silead_ts_get_status(struct i2c_client *client)
{
   int ret;
   u32 status;

   ret = i2c_smbus_read_i2c_block_data(client, SILEAD_REG_STATUS, 4,
                       (u8 *)&status);
   if (ret < 0) {
       dev_err(&client->dev, "Status read error %d\n", ret);
       return ret;
   }

   return status;
}

static int silead_ts_get_id(struct i2c_client *client)
{
   struct silead_ts_data *data = i2c_get_clientdata(client);
   int ret;

   ret = i2c_smbus_read_i2c_block_data(client, SILEAD_REG_ID, 4,
                       (u8 *)&data->chip_id);
   if (ret < 0) {
       dev_err(&client->dev, "Chip ID read error %d\n", ret);
       return ret;
   }

   return 0;
}

static int silead_ts_setup(struct i2c_client *client)
{
   struct silead_ts_data *data = i2c_get_clientdata(client);
   struct device *dev = &client->dev;
   int ret;
   u32 status;

   silead_ts_set_power(client, SILEAD_POWER_OFF);
   silead_ts_set_power(client, SILEAD_POWER_ON);

   ret = silead_ts_get_id(client);
   if (ret)
       return ret;
   dev_dbg(dev, "Chip ID: 0x%8X", data->chip_id);

   ret = silead_ts_init(client);
   if (ret)
       return ret;

   ret = silead_ts_reset(client);
   if (ret)
       return ret;

   ret = silead_ts_load_fw(client);
   if (ret)
       return ret;

   ret = silead_ts_startup(client);
   if (ret)
       return ret;

   status = silead_ts_get_status(client);
   if (status != SILEAD_STATUS_OK) {
       dev_err(dev, "Initialization error, status: 0x%X\n", status);
       return -ENODEV;
   }

   return 0;
}

static irqreturn_t silead_ts_threaded_irq_handler(int irq, void *id)
{
   struct silead_ts_data *data = (struct silead_ts_data *)id;
   struct i2c_client *client = data->client;

   silead_ts_read_data(client);

   return IRQ_HANDLED;
}

static int silead_ts_read_props(struct i2c_client *client)
{
    struct silead_ts_data *data = i2c_get_clientdata(client);
    struct device *dev = &client->dev;
    int ret;

    struct gsl1680_init_data *pdata = dev->platform_data;

    if (pdata == NULL)
    {
        dev_err(dev, "No platform data for Silead driver\n");
        return -ENODEV;
    }

    data->x_max = 800;//pdata->x_max;
    data->y_max = 480;//pdata->y_max;
    data->max_fingers = 5;//pdata->max_fingers;
    sprintf(data->fw_name, "gsl1680.bin");
    data->gpio_power = pdata->gpio_reset;
    data->gpio_irq = pdata->gpio_int;

    dev_dbg(dev, "X max = %d, Y max = %d, max fingers = %d",
        data->x_max, data->y_max, data->max_fingers);


    gpio_free(data->gpio_irq);
    ret = gpio_request(data->gpio_irq, "Silead intrpt");
    if (ret) 
    {
      printk("GPIO int request failure: %d\n", ret );
      return ret;
    }
    gpio_direction_input(data->gpio_irq);
    gpio_export(data->gpio_irq, false);

    gpio_free(data->gpio_power);
    ret = gpio_request(data->gpio_power, "Silead reset");
    if (ret) 
    {
      printk("GPIO rst request failure: %d\n", ret );
      return ret;
    }
    gpio_direction_output(data->gpio_power, 0);
    gpio_export(data->gpio_power, false);

    return 0;
}

static int silead_ts_probe(struct i2c_client *client,
              const struct i2c_device_id *id)
{
   struct silead_ts_data *data;
   struct device *dev = &client->dev;
   int ret;

   printk(KERN_INFO DRVNAME " Probing \n");

   if (!i2c_check_functionality(client->adapter,
                    I2C_FUNC_I2C |
                    I2C_FUNC_SMBUS_READ_I2C_BLOCK |
                    I2C_FUNC_SMBUS_WRITE_I2C_BLOCK)) {
       dev_err(dev, "I2C functionality check failed\n");
       return -ENXIO;
   }

   data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
   if (!data)
       return -ENOMEM;

   i2c_set_clientdata(client, data);
   data->client = client;

   printk(KERN_INFO DRVNAME " Data allocated \n");

    printk(KERN_INFO DRVNAME " Reading props \n");
   ret = silead_ts_read_props(client);
   if (ret)
       return ret;

   printk(KERN_INFO DRVNAME " Setting up \n");
   ret = silead_ts_setup(client);
   if (ret)
       return ret;

   printk(KERN_INFO DRVNAME " Requesting input device \n");
   ret = silead_ts_request_input_dev(data);
   if (ret)
       return ret;

   client->irq = gpio_to_irq(data->gpio_irq);

   // If the IRQ is not filled
    // we can't continue without it
   if (client->irq <= 0) {
       dev_err(dev, "Failed to get IRQ from GPIO\n"); 
       return -ENODEV;
   }

   ret = devm_request_threaded_irq(dev, client->irq, NULL,
                   silead_ts_threaded_irq_handler,
                   IRQF_ONESHOT | IRQ_TYPE_EDGE_RISING,
                   client->name, data);

   if (ret) {
       dev_err(dev, "IRQ request failed %d\n", ret);
       return ret;
   }
   else {
        dev_info(dev, "IRQ %d registered", client->irq);
   }

   // Init the vraible
   data->sent_empty_last = false;
   data->slot_pressed[0] = false;
   data->slot_pressed[1] = false;
   data->slot_pressed[2] = false;
   data->slot_pressed[3] = false;
   data->slot_pressed[4] = false;

   printk(KERN_INFO DRVNAME " Probing succeded \n");
   dev_dbg(dev, "Probing succeded\n");
   return 0;
}

/*static int silead_ts_remove(struct i2c_client *client,
              const struct i2c_device_id *id)
{
    struct silead_ts_data *data = i2c_get_clientdata(client);
    struct device *dev = &client->dev;

    dev_info(dev, "Removing GSL1680");

    return 0;
}*/

#ifdef CONFIG_PM_SLEEP
static int silead_ts_suspend(struct device *dev)
{
   struct i2c_client *client = to_i2c_client(dev);

   silead_ts_set_power(client, SILEAD_POWER_OFF);
   return 0;
}

static int silead_ts_resume(struct device *dev)
{
   struct i2c_client *client = to_i2c_client(dev);
   int ret, status;

   silead_ts_set_power(client, SILEAD_POWER_ON);

   ret = silead_ts_reset(client);
   if (ret)
       return ret;

   ret = silead_ts_startup(client);
   if (ret)
       return ret;

   status = silead_ts_get_status(client);
   if (status != SILEAD_STATUS_OK) {
       dev_err(dev, "Resume error, status: 0x%X\n", status);
       return -ENODEV;
   }

   return 0;
}

static SIMPLE_DEV_PM_OPS(silead_ts_pm, silead_ts_suspend, silead_ts_resume);
#endif

static const struct i2c_device_id silead_ts_id[] = {
   { "gsl1680", 0 },
   { }
};

static struct i2c_driver silead_ts_driver = {
   .probe = silead_ts_probe,
   //.remove = silead_ts_remove,
   .id_table = silead_ts_id,
   .driver = {
       .name = SILEAD_TS_NAME,
       .owner = THIS_MODULE,
#ifdef CONFIG_PM_SLEEP
       .pm = &silead_ts_pm,
#endif
   },
};

static int __init silead_init( void )
{
    int ret;

    ret = i2c_add_driver(&silead_ts_driver);
    if (ret < 0) {
        printk(KERN_WARNING DRVNAME "can't add i2c driver: %d\n", ret);
    }

    return ret;
}

static void __exit silead_exit( void )
{
    i2c_del_driver(&silead_ts_driver);
}
module_init(silead_init);
module_exit(silead_exit);

MODULE_AUTHOR("Gerhard de Clercq <gerhard@declercqsystems.com>");
MODULE_DESCRIPTION("GSL1680 I2C touchscreen driver");
MODULE_LICENSE("GPL");
