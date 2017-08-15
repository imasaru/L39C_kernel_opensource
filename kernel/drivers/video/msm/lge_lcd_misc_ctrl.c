#include <linux/sched.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <mach/board.h>

#include "mdp.h"

#define IOCTL_READ_REG _IOW('a', 0, int)
#define IOCTL_WRITE_REG _IOW('a', 1, int)
#define IOCTL_READ_PORCH _IOW('a', 2, int)
#define IOCTL_WRITE_PORCH _IOW('a', 3, int)
#define IOCTL_READ_LUT _IOW('a', 4, int)
#define IOCTL_WRITE_LUT _IOW('a', 5, int)
#define IOCTL_READ_NEGATIVE_GAMMA _IOW('a', 6, int)
#define IOCTL_WRITE_NEGATIVE_GAMMA _IOW('a', 7, int)

#define LUT_SIZE 256
#if 0//def CONFIG_LGE_LCD_TUNING
#define GAMMNA_SIZE 	16
#define TUNING_REGSIZE  100
#define TUNING_REGNUM 16
struct tuning_buff{
	char buf[TUNING_REGNUM][TUNING_REGSIZE];
	int size;
	int idx;
};
#endif
int mdp_lut_init_update_lcdc(void);
//extern int lge_set_qlut(void);
//extern unsigned int p_lg_qc_lcdc_lut[];
extern uint32 lge_gc_lut[];
extern int tuning_read_regset(unsigned long tmp);
extern int tuning_write_regset(unsigned long tmp);
//extern struct dsi_cmd_desc ili9488_init_on_cmds[];

#if 0
static int tuning_read_regset(unsigned long tmp)
{
	int i;
	int size;
	struct tuning_buff *rbuf = (struct tuning_buff *)tmp;

	size = ARRAY_SIZE(ili9488_init_on_cmds);

	for (i = 0; i < size; i++) {
		if (copy_to_user(rbuf->buf[i], ili9488_init_on_cmds[i].payload,
					ili9488_init_on_cmds[i].dlen))
		{
			printk(KERN_ERR "read_file : error of copy_to_user_buff\n");
			return -EFAULT;
		}
	}
	if (put_user(size, &(rbuf->size))) {
	return 0;
}
#endif
static int tuning_read_lut(unsigned long tmp)
{
	int size = LUT_SIZE*4;
	printk(KERN_INFO "read_lut_table in misc driver\n");

	if (copy_to_user((unsigned int *)tmp, lge_gc_lut,
				size)) {
		printk(KERN_ERR "read_file : error of copy_to_user_buff\n");
		return -EFAULT;
	}

	return 0;
}

static int tuning_write_lut(unsigned long tmp)
{
	u32 *buf;
	int size = LUT_SIZE*4;

	printk(KERN_INFO "write lut file\n");

	buf = kmalloc(size, GFP_KERNEL);
	if (copy_from_user(buf, (unsigned int *)tmp, size)) {
		printk(KERN_ERR "write_file : error of copy_from_user\n");
		return -EFAULT;
	}

	memcpy(lge_gc_lut, buf, size);
	kfree(buf);
	return 0;
}
long device_ioctl(struct file *file, unsigned int ioctl_num,
		unsigned long ioctl_param)
{
	switch (ioctl_num) {
		case IOCTL_READ_LUT:
			printk(KERN_INFO "IOCTL_READ_LUT\n");
			tuning_read_lut(ioctl_param);
			break;
		case IOCTL_WRITE_LUT:
			printk(KERN_INFO "IOCTL_WRITE_LUT\n");
			tuning_write_lut(ioctl_param);
			mdp_lut_init_update_lcdc();
			break;
		case IOCTL_READ_REG:
			printk(KERN_INFO "IOCTL_READ_REG\n");
			tuning_read_regset(ioctl_param);
			break;
		case IOCTL_WRITE_REG:
			printk(KERN_INFO "IOCTL_WRITE_REG\n");
			tuning_write_regset(ioctl_param);
			break;
		default:
			printk(KERN_ERR "IOCTL WRONG CMD\n");
			return -1;
			break;
	}
	return 0;
}

static const struct file_operations lcd_misc_fops = {
	.owner= THIS_MODULE,
	.unlocked_ioctl = device_ioctl
};

struct miscdevice lcd_misc_dev = {
	.minor= MISC_DYNAMIC_MINOR,
	.name= "lcd_misc",
	.fops= &lcd_misc_fops
};

static int lcd_misc_probe(struct platform_device *pdev)
{
	return misc_register(&lcd_misc_dev);
}
static struct platform_driver this_driver = {
	.probe  = lcd_misc_probe,
	.driver = {
		.name   = "lcd_misc_msm",
	},
};

int __init lcd_misc_init(void)
{
	printk(KERN_INFO "lcd_misc_init \n");
	return platform_driver_register(&this_driver);
}

device_initcall(lcd_misc_init);

MODULE_DESCRIPTION("MSM MISC driver");
MODULE_LICENSE("GPL v2");
