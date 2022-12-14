/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>       /* min() */
#include <linux/uaccess.h>      /* copy_to_user() */
#include <linux/sched/clock.h> /* local_clock() */
#include <linux/sched/signal.h> /* TASK_INTERRUPTIBLE/signal_pending/schedule */
#include <linux/poll.h>
#include <linux/io.h>           /* ioremap() */
#include <linux/of_fdt.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/seq_file.h>
#include <asm/setup.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/atomic.h>
#include <linux/irq.h>
#include <linux/syscore_ops.h>
#ifdef CONFIG_MTK_AEE_AED
#include <mt-plat/aee.h>
#endif
#include <mt-plat/mtk_secure_api.h>

/*#define ATF_LOGGER_DEBUG*/

#define ATF_LOG_CTRL_BUF_SIZE 512
#define ATF_CRASH_MAGIC_NO	0xdead1abf
#define ATF_LAST_MAGIC_NO	0x41544641

static struct mutex atf_mutex; /* shared between the threads */
static wait_queue_head_t    atf_log_wq;
static const struct of_device_id atf_logger_of_ids[] = {
	{ .compatible = "mediatek,atf_logger", },
	{}
};

union atf_log_ctl_t {
	struct {
		unsigned int atf_buf_addr;          /*  0x00 */
		unsigned int atf_buf_size;
		unsigned int atf_write_pos;
		unsigned int atf_read_pos;
		/* atf_spinlock_t atf_buf_lock; */
		unsigned int atf_buf_lock;          /*  0x10 */
		unsigned int atf_buf_unread_size;
		unsigned int atf_irq_count;
		unsigned int atf_reader_alive;
		unsigned long long atf_write_seq;   /*  0x20 */
		/* useless both in ATF and atf_logger */
		unsigned long long atf_read_seq;
		unsigned int atf_aee_dbg_buf_addr;  /*  0x30 */
		unsigned int atf_aee_dbg_buf_size;
		unsigned int atf_crash_log_addr;
		unsigned int atf_crash_log_size;
		unsigned int atf_crash_flag;        /*  0x40 */
		/* for FIQ/IRQ footprint, print in crash log*/
		unsigned int atf_crash_write_pos;
#ifdef CONFIG_MTK_AEE_AED
		unsigned long long except_write_pos_per_cpu[AEE_MTK_CPU_NUMS];
		unsigned long long fiq_irq_enter_timestamp[AEE_MTK_CPU_NUMS];
		unsigned long long fiq_irq_quit_timestamp[AEE_MTK_CPU_NUMS];
		unsigned int irq_num[AEE_MTK_CPU_NUMS];
#endif
	} info;
	unsigned char data[ATF_LOG_CTRL_BUF_SIZE];
};

static union atf_log_ctl_t *atf_buf_vir_ctl;
static unsigned long atf_buf_phy_ctl;
static unsigned int atf_buf_len;
static unsigned char *atf_log_vir_addr;
static unsigned int atf_log_len;


static ssize_t atf_log_write(struct file *file,
	const char __user *buf, size_t count, loff_t *pos)
{
	unsigned long ret;
	unsigned long param;

	ret = -1;
	param = -1;

	if (count < 12) {
		/* for coverity check */
		/* use kstrtoul_from_user() instead of */
		/* copy_from_user() and kstrtoul() */
		ret = kstrtoul_from_user(buf, count, 16, &param);
	}

	pr_notice("[%s]param:0x%lx, count:%zu, ret:%ld\n",
		__func__, param, count, ret);
	if (ret == 0x0) {
		mt_secure_call(MTK_SIP_KERNEL_ATF_DEBUG,
			param, 0, 0, 0);
	} else {
		wake_up_interruptible(&atf_log_wq);
	}
	*pos += count;
	return count;
}

static ssize_t do_read_log_to_usr(char __user *buf, size_t count)
{
	size_t copy_len = 0;
	size_t right = 0;

	unsigned int local_write_index = 0;
	unsigned int local_read_index = 0;

	local_write_index = atf_buf_vir_ctl->info.atf_write_pos;
	local_read_index = atf_buf_vir_ctl->info.atf_read_pos;

	/* check copy length */
	copy_len = (local_write_index +
		atf_log_len - local_read_index) % atf_log_len;

	/* if copy length < count, just copy the "copy length" */
	if (count > copy_len)
		count = copy_len;

	if (local_write_index > local_read_index) {
		/* write (right) - read (left) */
		/* --------R-------W-----------*/
		if (copy_to_user(buf,
			atf_log_vir_addr + local_read_index, count))
			return -EFAULT;
	} else {
		/* turn around to the head */
		/* --------W-------R-----------*/
		right = atf_log_len - local_read_index;

		/* check buf space is enough to copy */
		if (count > right) {
			/* if space is enough to copy */
			if (copy_to_user(buf,
				atf_log_vir_addr + local_read_index, right))
				return -EFAULT;
			if (copy_to_user((buf + right),
				atf_log_vir_addr, count - right))
				return -EFAULT;
		} else {
			/* if count is only enough to copy right or count,
			 * just copy right or count.
			 */
			if (copy_to_user(buf,
				atf_log_vir_addr + local_read_index, count))
				return -EFAULT;
		}
	}

	/* update the read pos */
	local_read_index = (local_read_index + count) % atf_log_len;
	atf_buf_vir_ctl->info.atf_read_pos = local_read_index;
	return count;
}

static int atf_log_open(struct inode *inode, struct file *file)
{
	int ret;

	ret = nonseekable_open(inode, file);
	if (unlikely(ret))
		return ret;
	file->private_data = NULL;

	atf_buf_vir_ctl->info.atf_reader_alive++;

	return 0;
}

static int atf_log_release(struct inode *ignored, struct file *file)
{
	atf_buf_vir_ctl->info.atf_reader_alive--;
	return 0;
}

static ssize_t atf_log_read(struct file *file,
	char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t readback_bytes;
	int rc;
	unsigned int write_pos;
	unsigned int read_pos;

	readback_bytes = 0;
	rc = 0;

	/* inside a thread */
	mutex_lock(&atf_mutex);
start:
	while (1) {
		/* pr_notice("atf_log_read: wait in wq\n"); */
		wait_event_interruptible_timeout(atf_log_wq,
			(atf_buf_vir_ctl->info.atf_write_pos !=
				atf_buf_vir_ctl->info.atf_read_pos), HZ*10);

		write_pos = atf_buf_vir_ctl->info.atf_write_pos;
		read_pos = atf_buf_vir_ctl->info.atf_read_pos;

		/* pr_notice("w_pos=%d r_pos=%d\n", write_pos, read_pos); */
		if (write_pos != read_pos) {
			break;
		} else if (file->f_flags & O_NONBLOCK) {
			rc = -EAGAIN;
			break;
		}
	}
	if (rc) {
		pr_notice("%s: rc=%d\n", __func__, rc);
		/* do the work with the data you're protecting */
		mutex_unlock(&atf_mutex);
		return rc;
	}

	if (unlikely(write_pos == read_pos))
		goto start;

	readback_bytes = do_read_log_to_usr(buf, count);
	/* pr_notice("read count=%d\n", readback_bytes); */
	/* update the file pos */
	*f_pos += readback_bytes;

	/* do the work with the data you're protecting */
	mutex_unlock(&atf_mutex);

	return readback_bytes;
}

static unsigned int atf_log_poll(struct file *file, poll_table *wait)
{
	unsigned int ret = 0;

	if (!(file->f_mode & FMODE_READ))
		return ret;

	poll_wait(file, &atf_log_wq, wait);

	if (atf_buf_vir_ctl->info.atf_write_pos !=
		atf_buf_vir_ctl->info.atf_read_pos)
		ret |= POLLIN | POLLRDNORM;

	return ret;
}

static long atf_log_ioctl(struct file *flip,
	unsigned int cmd, unsigned long arg)
{
	return 0;
}

static const struct file_operations atf_log_fops = {
	.owner      = THIS_MODULE,
	.unlocked_ioctl = atf_log_ioctl,
	.compat_ioctl = atf_log_ioctl,
	.poll       = atf_log_poll,
	.open       = atf_log_open,
	.release    = atf_log_release,
	.read       = atf_log_read,
	.write      = atf_log_write,
};

static struct miscdevice atf_log_dev = {
	.minor      = MISC_DYNAMIC_MINOR,
	.name       = "atf_log",
	.fops       = &atf_log_fops,
	.mode       = 0640,
};
#ifdef CONFIG_MTK_RAM_CONSOLE
static int __init atf_log_dt_scan_memory(unsigned long node,
	const char *uname, int depth, void *data)
{
	char *type = (char *)of_get_flat_dt_prop(node, "device_type", NULL);
	__be32 *reg, *endp;
	int l;

	/* We are scanning "memory" nodes only */
	if (type == NULL) {
		/*
		 * The longtrail doesn't have a device_type on the
		 * /memory node, so look for the node called /memory@0.
		 */
		if (depth != 1 || strcmp(uname, "memory@0") != 0)
			return 0;
	} else if (strcmp(type, "memory") != 0)
		return 0;

	reg = (__be32 *)of_get_flat_dt_prop(node, "reg", (int *)&l);
	if (reg == NULL)
		return 0;

	endp = reg + (l / sizeof(__be32));

	while ((endp - reg) >= (dt_root_addr_cells + dt_root_size_cells)) {
		u64 base, size;

		base = dt_mem_next_cell(dt_root_addr_cells,
			(const __be32 **)&reg);
		size = dt_mem_next_cell(dt_root_size_cells,
			(const __be32 **)&reg);

		if (size == 0)
			continue;
	}
	*(unsigned long *)data = node;
	return node;
}
#endif

static unsigned long long __init atf_log_get_from_dt(
	unsigned long *phy_addr, unsigned int *len)
{
#ifdef CONFIG_MTK_RAM_CONSOLE
	unsigned long node = 0;
	struct mem_desc *mem_desc = NULL;

	if (of_scan_flat_dt(atf_log_dt_scan_memory, &node)) {
		mem_desc = (struct mem_desc *)of_get_flat_dt_prop(node,
			"tee_reserved_mem", NULL);
		if (mem_desc && mem_desc->size) {
			pr_notice("ATF reserved memory: 0x%08llx - 0x%08llx (0x%llx)\n",
				mem_desc->start,
				mem_desc->start+mem_desc->size - 1,
				mem_desc->size);
		}
	}
	if (mem_desc) {
		*phy_addr = mem_desc->start;
		*len = mem_desc->size;
	}
#endif
	return 0;
}


#ifdef ATF_LOGGER_DEBUG
void show_atf_log_ctl(void)
{
	pr_notice("atf_buf_addr(%p) = 0x%x\n",
			&(atf_buf_vir_ctl->info.atf_buf_addr),
			atf_buf_vir_ctl->info.atf_buf_addr);
	pr_notice("atf_buf_size(%p) = 0x%x\n",
			&(atf_buf_vir_ctl->info.atf_buf_size),
			atf_buf_vir_ctl->info.atf_buf_size);
	pr_notice("atf_write_pos(%p) = %u\n",
			&(atf_buf_vir_ctl->info.atf_write_pos),
			atf_buf_vir_ctl->info.atf_write_pos);
	pr_notice("atf_read_pos(%p) = %u\n",
			&(atf_buf_vir_ctl->info.atf_read_pos),
			atf_buf_vir_ctl->info.atf_read_pos);
	pr_notice("atf_buf_lock(%p) = %u\n",
			&(atf_buf_vir_ctl->info.atf_buf_lock),
			atf_buf_vir_ctl->info.atf_buf_lock);
	pr_notice("atf_buf_unread_size(%p) = %u\n",
			&(atf_buf_vir_ctl->info.atf_buf_unread_size),
			atf_buf_vir_ctl->info.atf_buf_unread_size);
	pr_notice("atf_irq_count(%p) = %u\n",
			&(atf_buf_vir_ctl->info.atf_irq_count),
			atf_buf_vir_ctl->info.atf_irq_count);
	pr_notice("atf_reader_alive(%p) = %u\n",
			&(atf_buf_vir_ctl->info.atf_reader_alive),
			atf_buf_vir_ctl->info.atf_reader_alive);
	pr_notice("atf_write_seq(%p) = %llu\n",
			&(atf_buf_vir_ctl->info.atf_write_seq),
			atf_buf_vir_ctl->info.atf_write_seq);
}
#if 0
static void show_data(unsigned long addr,
	int nbytes, const char *name)
{
	int	i, j;
	int	nlines;
	u32	*p;

	/*
	 * don't attempt to dump non-kernel addresses or
	 * values that are probably just small negative numbers
	 */
	if (addr < PAGE_OFFSET || addr > -256UL)
		return;

	pr_debug("\n%s: %#lx:\n", name, addr);

	/*
	 * round address down to a 32 bit boundary
	 * and always dump a multiple of 32 bytes
	 */
	p = (u32 *)(addr & ~(sizeof(u32) - 1));
	nbytes += (addr & (sizeof(u32) - 1));
	nlines = (nbytes + 31) / 32;


	for (i = 0; i < nlines; i++) {
		/*
		 * just display low 16 bits of address to keep
		 * each line of the dump < 80 characters
		 */
		pr_debug("%04lx ", (unsigned long)p & 0xffff);
		for (j = 0; j < 8; j++) {
			u32	data;

			if (probe_kernel_address(p, data))
				pr_debug(" ********");
			else
				pr_debug(" %08x", data);
			++p;
		}
		pr_debug("\n");
	}
}
#endif
#endif

static irqreturn_t ATF_log_irq_handler(int irq, void *dev_id)
{
	if (!atf_buf_vir_ctl->info.atf_reader_alive)
		pr_info("No alive reader, but still receive irq\n");
	wake_up_interruptible(&atf_log_wq);
	return IRQ_HANDLED;
}

static void atf_time_sync_resume(void)
{
	/* Get local_clock and sync to ATF */
	u64 time_to_sync = local_clock();

#ifdef CONFIG_ARM64
	mt_secure_call(MTK_SIP_KERNEL_TIME_SYNC, time_to_sync, 0, 0, 0);
#else
	mt_secure_call(MTK_SIP_KERNEL_TIME_SYNC,
		(u32)time_to_sync, (u32)(time_to_sync >> 32), 0, 0);
#endif
	pr_notice("atf_time_sync: resume sync");
}

static struct syscore_ops atf_time_sync_syscore_ops = {
	.resume = atf_time_sync_resume,
};

static int __init atf_logger_probe(struct platform_device *pdev)
{
	/* register module driver */
	int err;
	struct proc_dir_entry *atf_log_proc_dir;
	struct proc_dir_entry *atf_log_proc_file;
	int irq_num;
	/* register module driver */
	u64 time_to_sync;

	err = misc_register(&atf_log_dev);
	if (unlikely(err)) {
		pr_info("atf_log: failed to register device");
		return -1;
	}
	pr_notice("atf_log: inited");
	/* get atf reserved memory(atf_buf_phy_ctl) from device tree */
	/* pass from preloader to LK, then create the dt node in LK */
	atf_log_get_from_dt(&atf_buf_phy_ctl, &atf_buf_len);    /* TODO */
	if (atf_buf_len == 0) {
		pr_info("No atf_log_buffer!\n");
		return -1;
	}
	/* map control header */
	atf_buf_vir_ctl = ioremap_wc(atf_buf_phy_ctl, ATF_LOG_CTRL_BUF_SIZE);
	atf_log_len = atf_buf_vir_ctl->info.atf_buf_size;
	/* map log buffer */
	atf_log_vir_addr = ioremap_wc(atf_buf_phy_ctl +
		ATF_LOG_CTRL_BUF_SIZE, atf_log_len);
	pr_notice("atf_buf_phy_ctl: 0x%lx\n", atf_buf_phy_ctl);
	pr_notice("atf_buf_len: %u\n", atf_buf_len);
	pr_notice("atf_buf_vir_ctl: %p\n", atf_buf_vir_ctl);
	pr_notice("atf_log_vir_addr: %p\n", atf_log_vir_addr);
	pr_notice("atf_log_len: %u\n", atf_log_len);
#ifdef ATF_LOGGER_DEBUG
	/* show_atf_log_ctl(); */
	/* show_data(atf_buf_vir_ctl, 512, "atf_buf"); */
#endif
	atf_buf_vir_ctl->info.atf_reader_alive = 0;
	/* initial wait queue */
	init_waitqueue_head(&atf_log_wq);

	irq_num = platform_get_irq(pdev, 0);
	if (irq_num == -ENXIO) {
		pr_info("Fail to get atf_logger irq number from device tree\n");
		WARN_ON(irq_num == -ENXIO);
		return -EINVAL;
	}
	pr_notice("atf irq num %d.\n", irq_num);
	if (request_irq(irq_num, (irq_handler_t)ATF_log_irq_handler,
			IRQF_TRIGGER_NONE, "ATF_irq", NULL) != 0) {
		pr_info("Fail to request ATF_log_irq interrupt!\n");
		return -1;
	}

	/* create /proc/atf_log */
	atf_log_proc_dir = proc_mkdir("atf_log", NULL);
	if (atf_log_proc_dir == NULL) {
		pr_info("atf_log proc_mkdir failed\n");
		return -ENOMEM;
	}

	/* create /proc/atf_log/atf_log */
	atf_log_proc_file = proc_create("atf_log", 0440,
		atf_log_proc_dir, &atf_log_fops);
	if (atf_log_proc_file == NULL) {
		pr_info("atf_log proc_create failed at atf_log\n");
		return -ENOMEM;
	}
	register_syscore_ops(&atf_time_sync_syscore_ops);

	/* Get local_clock and sync to ATF */
	time_to_sync = local_clock();

#ifdef CONFIG_ARM64
	mt_secure_call(MTK_SIP_KERNEL_TIME_SYNC, time_to_sync, 0, 0, 0);
#else
	mt_secure_call(MTK_SIP_KERNEL_TIME_SYNC,
		(u32)time_to_sync, (u32)(time_to_sync >> 32), 0, 0);
#endif
	pr_notice("atf_time_sync: inited");

	return 0;
}

static void __exit atf_log_exit(void)
{
	misc_deregister(&atf_log_dev);
	pr_notice("atf_log: exited");
}
static int atf_logger_remove(struct platform_device *dev)
{
	return 0;
}

/* variable with __init* or __refdata (see linux/init.h) or */
/* name the variable *_template, *_timer, *_sht, *_ops, *_probe, */
/* *_probe_one, *_console */
static struct platform_driver atf_logger_driver_probe = {
	.probe = atf_logger_probe,
	.remove = atf_logger_remove,
	.driver = {
		.name = "atf_logger",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = atf_logger_of_ids,
#endif
	},
};

static int __init atf_log_init(void)
{
	int ret = 0;

	mutex_init(&atf_mutex); /* called only ONCE */

	ret = platform_driver_register(&atf_logger_driver_probe);
	if (ret)
		pr_info("atf logger init FAIL, ret 0x%x!!!\n", ret);
	return ret;
}


module_init(atf_log_init);
module_exit(atf_log_exit);

MODULE_DESCRIPTION("MEDIATEK Module ATF Logging Driver");
MODULE_AUTHOR("Chun Fan<chun.fan@mediatek.com>");

