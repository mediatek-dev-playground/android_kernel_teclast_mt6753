/*
 *  drivers/switch/switch_gpio.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/timer.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <cust_eint.h>
#include <cust_gpio_usage.h>
#include <mach/mt_gpio.h>
#include <mach/eint.h>
#include <mach/upmu_common.h>
#include <linux/input.h>




#define support_power_save_key

struct mtk_gpio_hall_data {
	struct switch_dev sdev;
	struct input_dev *idev;
	unsigned gpio;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	int irq;
	struct work_struct work;
	unsigned eint_no;
	unsigned trigger_mode;		// 1:level, 0:edge
	unsigned active_level;	// 1:high or rising, 0:low or falling
	unsigned active_value;	// input event value when active on
	unsigned inactive_value;
	unsigned debounce_us;
};

struct mtk_gpio_hall_platform_data {
	const char *name;
	unsigned 	gpio;

	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	
	unsigned eint_no;		// eint number
	unsigned trigger_mode;	// 1:level, 0:edge
	unsigned active_level;	// 1:high or rising, 0:low or falling
	unsigned active_value;	// input event value when active on
	unsigned inactive_value;
	unsigned debounce_us;
};


#ifdef support_power_save_key

static struct mtk_gpio_hall_platform_data power_save_pdata = {
	.name = "hall_gpio",
	.name_on = "hall_on",
	.name_off = "hall_off",
	.state_on = "1",
	.state_off = "0",
	.eint_no = 66,//22,
	.trigger_mode = 1,
	.active_level = 1,
	.active_value = 0xf9, //KEY_DASHBOARD close
	.inactive_value = 0xfa, //open
};

static struct platform_device power_save_device = {
	.name = "mtk-hall-gpio",
	.id = -1,
	.dev = {
		.platform_data = &power_save_pdata,
	}
};
#endif

static void mtk_gpio_hall_work(struct work_struct *work)
{
	int state;
	unsigned key_value;
	
	struct mtk_gpio_hall_data	*data =
		container_of(work, struct mtk_gpio_hall_data, work);

	printk("[HALLDET]mtk_gpio_hall_work\n");

	//disable_irq_nosync(data->irq);

	state = mt_get_gpio_in(data->gpio);
	switch_set_state(&data->sdev, state);

	key_value = (state==data->active_level) ? data->inactive_value : data->active_value;
	printk("[HALLDET]state=%d, key=%d\n",state, key_value);

	input_report_key(data->idev, key_value, 1);
	input_sync(data->idev);
	input_report_key(data->idev, key_value, 0);
	input_sync(data->idev);

	if (state==1) {
		irq_set_irq_type(data->irq,IRQ_TYPE_LEVEL_LOW);
	}
	else {
		irq_set_irq_type(data->irq,IRQ_TYPE_LEVEL_HIGH);
	}

	enable_irq(data->irq);
}

static irqreturn_t mtk_hall_gpio_irq_handler(int irq, void *dev_id)
{
	struct mtk_gpio_hall_data *data =
	    (struct mtk_gpio_hall_data *)dev_id;

	//printk("[HALLDET]mtk_hall_gpio_irq_handler\n");
	disable_irq_nosync(irq);

	schedule_work(&data->work);
	return IRQ_HANDLED;
}

static ssize_t mtk_hall_gpio_print_state(struct switch_dev *sdev, char *buf)
{
	struct mtk_gpio_hall_data	*hall_data =
		container_of(sdev, struct mtk_gpio_hall_data, sdev);
	const char *state;
	if (switch_get_state(sdev))
		state = hall_data->state_on;
	else
		state = hall_data->state_off;

	if (state)
		return sprintf(buf, "%s\n", state);
	return -1;
}


static int mtk_gpio_hall_probe(struct platform_device *pdev)
{
	struct mtk_gpio_hall_platform_data *pdata = pdev->dev.platform_data;
	struct mtk_gpio_hall_data *hall_data;
	struct input_dev *mtk_gpio_hall_input_dev;
	int ret = 0;
	u32 ints[2]={0,0};
	struct device_node *node;

	printk("[HALLDET]%s\n", __func__);

	if (!pdata)
		return -EBUSY;

	hall_data = kzalloc(sizeof(struct mtk_gpio_hall_data), GFP_KERNEL);
	if (!hall_data) {
		printk("[HALLDET]alloc mtk_gpio_hall_data failed\n");
		return -ENOMEM;
	}
	
	hall_data->sdev.name = pdata->name;
	//switch_data->gpio = pdata->gpio;
	hall_data->name_on = pdata->name_on;
	hall_data->name_off = pdata->name_off;
	hall_data->state_on = pdata->state_on;
	hall_data->state_off = pdata->state_off;
	hall_data->eint_no = pdata->eint_no;
	hall_data->trigger_mode = pdata->trigger_mode;
	hall_data->active_level = pdata->active_level;
	hall_data->active_value = pdata->active_value;
	hall_data->inactive_value = pdata->inactive_value;
	//switch_data->debounce_us = pdata->debounce_us;
	hall_data->sdev.print_state = mtk_hall_gpio_print_state;

	// register switch
	ret = switch_dev_register(&hall_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	// input
	mtk_gpio_hall_input_dev = input_allocate_device();
	if (!mtk_gpio_hall_input_dev) {
		printk("[HALLDET]mtk_gpio_hall_input_dev : fail!\n");
		goto input_alloc_failed;
	}

	__set_bit(EV_KEY, mtk_gpio_hall_input_dev->evbit);
	__set_bit(hall_data->active_value, mtk_gpio_hall_input_dev->keybit);
	__set_bit(hall_data->inactive_value, mtk_gpio_hall_input_dev->keybit);

	mtk_gpio_hall_input_dev->id.bustype = BUS_HOST;
	mtk_gpio_hall_input_dev->name = hall_data->sdev.name;
	ret = input_register_device(mtk_gpio_hall_input_dev); 
	if (ret>0) {
		printk("[HALLDET]mtk_gpio_hall_input_dev register : fail!\n");
		goto input_register_failed;
	} 

	hall_data->idev = mtk_gpio_hall_input_dev;
	
	INIT_WORK(&hall_data->work, mtk_gpio_hall_work);
		

	// eint and irq
	node = of_find_compatible_node(NULL,NULL,"mediatek, HALL_1-eint");
	if(node) {
		of_property_read_u32_array(node,"debounce",ints,ARRAY_SIZE(ints));
		hall_data->gpio = ints[0] | 0x80000000; // avoid hardcode check
		hall_data->debounce_us = ints[1];
		printk("[HALLDET]gpio=%d,debounce=%d\n", ints[0],ints[1]);
		
		mt_set_gpio_mode(hall_data->gpio, GPIO_MODE_04); // in mt6732, all eint mode is mode_04
		mt_set_gpio_dir(hall_data->gpio, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(hall_data->gpio, GPIO_PULL_ENABLE); //To disable GPIO PULL.
		mt_set_gpio_pull_select(hall_data->gpio, GPIO_PULL_UP);
		
		mt_gpio_set_debounce(hall_data->eint_no,hall_data->debounce_us);
		hall_data->irq = irq_of_parse_and_map(node,0);
		ret = request_irq(hall_data->irq,mtk_hall_gpio_irq_handler,IRQF_TRIGGER_NONE,"swgpio-eint",hall_data);
		disable_irq_nosync(hall_data->irq);
		if(ret>0){
			printk("[HALLDET]eint irq line not avaliable\n");
			goto err_request_irq;
		}
	}
	else {
		printk("[HALLDET]%s can't find compatible node\n", __func__);
		goto err_request_irq;
	}

	/* Perform initial detection */
	mtk_gpio_hall_work(&hall_data->work);

	return 0;

input_register_failed:
	input_free_device(mtk_gpio_hall_input_dev);
input_alloc_failed:
	switch_dev_unregister(&hall_data->sdev);
err_switch_dev_register:
	free_irq(hall_data->irq,NULL);
err_request_irq:
	kfree(hall_data);
	
	printk("[HALLDET]%s error probe!\n",__func__);

	return ret;
}

static int mtk_gpio_hall_remove(struct platform_device *pdev)
{
	struct mtk_gpio_hall_data *hall_data = platform_get_drvdata(pdev);

	cancel_work_sync(&hall_data->work);
	//gpio_free(switch_data->gpio);
	free_irq(hall_data->irq,NULL);
	switch_dev_unregister(&hall_data->sdev);
	kfree(hall_data);

	return 0;
}

static struct platform_driver mtk_gpio_hall_driver = {
	.probe		= mtk_gpio_hall_probe,
	.remove		= mtk_gpio_hall_remove,
	.driver		= {
		.name	= "mtk-hall-gpio",
		.owner	= THIS_MODULE,
	},
};

static int __init mtk_gpio_hall_init(void)
{
	int ret = 0;

	#ifdef support_power_save_key
	ret = platform_device_register(&power_save_device);
	if (ret) {
		printk("[HALLDET]gpio_hall_init register power_save_device failed:%d\n", ret);
		return ret;
	}
	#endif

	return platform_driver_register(&mtk_gpio_hall_driver);
}

static void __exit mtk_gpio_hall_exit(void)
{
	platform_driver_unregister(&mtk_gpio_hall_driver);
}

module_init(mtk_gpio_hall_init);
module_exit(mtk_gpio_hall_exit);

MODULE_AUTHOR("zengzhihua <zengzhihua@malatamobile.com>");
MODULE_DESCRIPTION("MTK GPIO Hall driver");
MODULE_LICENSE("GPL");
