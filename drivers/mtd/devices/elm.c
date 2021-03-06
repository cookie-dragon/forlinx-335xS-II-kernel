/*
 * elm.c
 * Copyright (C) 2011 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/completion.h>
#include <linux/err.h>

#include <plat/elm.h>

#define	DRIVER_NAME	"omap2_elm"

static void  __iomem *elm_base;
static struct completion elm_completion;

static void elm_write_reg(int idx, u32 val)
{
	__raw_writel(val, elm_base + idx);
}

static u32 elm_read_reg(int idx)
{
	return __raw_readl(elm_base + idx);
}

void elm_reset(void)
{
	u32 reg_val;

	reg_val = elm_read_reg(ELM_SYSCONFIG);
	reg_val |= ELM_SYSCONFIG_SOFTRESET;
	elm_write_reg(ELM_SYSCONFIG, reg_val);

	while ((elm_read_reg(ELM_SYSSTATUS) & ELM_SYSSTATUS_RESETDONE)
			!= ELM_SYSSTATUS_RESETDONE)
		;

	reg_val = elm_read_reg(ELM_SYSCONFIG);
	reg_val &= ~ELM_SYSCONFIG_SIDLE_MASK;
	reg_val |= ELM_SYSCONFIG_SMART_IDLE;
	elm_write_reg(ELM_SYSCONFIG, reg_val);
}

void elm_config(int bch_type)
{
		u32 reg_val;
		u32 buffer_size;
		elm_reset();
		buffer_size = 0x7ff;//bch8
		reg_val = (bch_type & ECC_BCH_LEVEL_MASK) | (buffer_size << 16);
		elm_write_reg(ELM_LOCATION_CONFIG, reg_val);

		/* clearing interrupts */
		reg_val = elm_read_reg(ELM_IRQSTATUS);
		elm_write_reg(ELM_IRQSTATUS, reg_val & INTR_STATUS_LOC_VALID_0);
		elm_write_reg(ELM_IRQSTATUS, INTR_STATUS_LOC_VALID_0);

		/* enable in interupt mode */
		reg_val = elm_read_reg(ELM_IRQENABLE);
		reg_val |= INTR_EN_LOCATION_MASK_0;
		elm_write_reg(ELM_IRQENABLE, reg_val);

		/* config in Continuous mode */
		reg_val = elm_read_reg(ELM_PAGE_CTRL);
		reg_val = 0;
		elm_write_reg(ELM_PAGE_CTRL, reg_val);
}
EXPORT_SYMBOL(elm_config);
void elm_load_syndrome(unsigned int ecc_bytes, char *syndrome)
{
	int reg_val;
	int i;

	for (i = 0; i < ecc_bytes; i += 4) {
		reg_val = syndrome[0] | syndrome[1] << 8 |
			syndrome[2] << 16 | syndrome[3] << 24;
		elm_write_reg(ELM_SYNDROME_FRAGMENT_0 + i, reg_val);
		syndrome += 4;
	}
}

void rotate_ecc_bytes(unsigned int ecc_bytes, u8 *src, u8 *dst)
{
	int i;

	for (i = 0; i < ecc_bytes; i++)
		dst[ecc_bytes - 1 - i] = src[i];
}

int elm_decode_bch_error(int bch_type,
				char *ecc_calc, unsigned int *err_loc)
{
	u8 ecc_data[28] = {0};
	u32 reg_val;
	int i, err_no;
	/* input calculated ECC syndrome to ELM engine */
	INIT_COMPLETION(elm_completion);
	switch (bch_type) {
	case ECC_TYPE_BCH16:
		rotate_ecc_bytes(ECC_BYTES_BCH16, ecc_calc, ecc_data);
		elm_load_syndrome(ECC_BYTES_BCH16, ecc_data);
		printk("%s %d\n",__FUNCTION__,__LINE__);
		break;
	case ECC_TYPE_BCH8:
		printk("%s %d\n",__FUNCTION__,__LINE__);
		rotate_ecc_bytes(ECC_BYTES_BCH8, ecc_calc, ecc_data);
		elm_load_syndrome(ECC_BYTES_BCH8, ecc_data);
		break;
	case ECC_TYPE_BCH4:
		rotate_ecc_bytes(ECC_BYTES_BCH4, ecc_calc, ecc_data);
		elm_load_syndrome(ECC_BYTES_BCH4, ecc_data);
		printk("%s %d\n",__FUNCTION__,__LINE__);
		break;
	default:
		printk("%s %d\n",__FUNCTION__,__LINE__);
		return -EINVAL;
	}
	

	/* start elm processing */
	reg_val = elm_read_reg(ELM_SYNDROME_FRAGMENT_6);
	reg_val |= ELM_SYNDROME_VALID;
	elm_write_reg(ELM_SYNDROME_FRAGMENT_6, reg_val);

	wait_for_completion(&elm_completion);
	reg_val = elm_read_reg(ELM_IRQSTATUS);
	reg_val = elm_read_reg(ELM_LOCATION_CONFIG);
	reg_val = elm_read_reg(ELM_LOCATION_STATUS);
	printk("reg_val %x\n",reg_val);

	if (reg_val & ECC_CORRECTABLE_MASK) {
		err_no = reg_val & ECC_NB_ERRORS_MASK;

		for (i = 0; i < err_no; i++) {
			reg_val = elm_read_reg(ELM_ERROR_LOCATION_0 + i * 4);
			err_loc[i] = reg_val;
		}

		printk("%s %d\n",__FUNCTION__,__LINE__);
		return err_no;
	}

		printk("%s %d\n",__FUNCTION__,__LINE__);
	return -EINVAL;
}
EXPORT_SYMBOL(elm_decode_bch_error);

static irqreturn_t elm_isr(int this_irq, void *dev_id)
{
	u32 reg_val;

	reg_val = elm_read_reg(ELM_IRQSTATUS);
	if (reg_val & INTR_STATUS_LOC_VALID_0) {
		elm_write_reg(ELM_IRQSTATUS, reg_val | INTR_STATUS_LOC_VALID_0);
		complete(&elm_completion);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static int __devinit omap_elm_probe(struct platform_device *pdev)
{
	struct resource *res = NULL, *irq = NULL;
	int             ret_status = 0;
	static struct clk *elm_clk;
	struct omap_elm_platform_data *platform_data = pdev->dev.platform_data;

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	if (irq == NULL) {
		printk("----------%s--------- no irq\n",__FUNCTION__);
		ret_status = -EBUSY;
		goto err_resource;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (res == NULL) {
	printk("----------%s--------- no reg\n",__FUNCTION__);
		ret_status = -EBUSY;
		goto err_resource;
	}

	if (!request_mem_region(res->start, resource_size(res),
				dev_name(&pdev->dev))) {
		ret_status = -EBUSY;
		goto err_resource;
	}

	elm_base = ioremap(res->start, resource_size(res));

	if (!elm_base) {
		dev_dbg(&pdev->dev, "can't ioremap ELM\n");
		ret_status = -ENOMEM;
		goto err_remap;
	}

	elm_clk = clk_get(&pdev->dev, "elm_fck");
	if (IS_ERR(elm_clk)) {
		printk(KERN_ERR "Could not get ELM clock %s\n", "elm_fck");
		ret_status = -ENOMEM;
		goto err_clk;
	}

	ret_status = request_irq(irq->start, elm_isr, 0, pdev->name,
			&pdev->dev);

	if (ret_status) {
		printk(KERN_ERR "failure requesting irq %i\n", irq->start);
		goto err_put_clock;
	}

	clk_enable(elm_clk);
	elm_reset();
	/* configure ELM module for handling various BCHx ECC schemes */
	if (platform_data == NULL) {
		printk(KERN_NOTICE  "%s: Error: missing platform data"
				"cannot initialize ELM", DRIVER_NAME);
		return -EINVAL;
	}
	if (platform_data->ecc_opt == OMAP_ECC_BCH16_CODE_HW)
		elm_config(ECC_TYPE_BCH16);
	else if (platform_data->ecc_opt == OMAP_ECC_BCH8_CODE_HW)
		elm_config(ECC_TYPE_BCH8);
	else
		elm_config(ECC_TYPE_BCH4);

	init_completion(&elm_completion);
	return ret_status;

err_put_clock:
	clk_put(elm_clk);
err_clk:
	iounmap(elm_base);
err_remap:
	release_mem_region(res->start, resource_size(res));
err_resource:
	return ret_status;
}

static int omap_elm_remove(struct platform_device *pdev)
{
	struct resource *res = NULL;
	int             ret_status = 0;

	iounmap(elm_base);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));
	return ret_status;
}

static struct platform_driver omap_elm_driver = {
	.probe		= omap_elm_probe,
	.remove		= omap_elm_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init omap_elm_init(void)
{

	return platform_driver_register(&omap_elm_driver);
}

static void __exit omap_elm_exit(void)
{
	platform_driver_unregister(&omap_elm_driver);
}

module_init(omap_elm_init);
module_exit(omap_elm_exit);

MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Module for Handling BCH error correction on TI OMAP boards");
