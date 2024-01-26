/*
    rl78_irq driver
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/gpio.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include "rl78_drv.h"

/* Wait queue for RL78 request signalisation via IRQ lines */
DECLARE_WAIT_QUEUE_HEAD(rl78_irq_queue);

/* irq flags */
#define RL78_IRQ      0x1

static irqreturn_t rl78_irq_handler(int, void *);
static int32_t rl78_req_line_read(void);
static int rl78_irq_open(struct inode *inode, struct file *file);
static unsigned int rl78_irq_poll(struct file *file, struct poll_table_struct *poll_table);
static ssize_t rl78_irq_read(struct file *file, char __user *buf, size_t count, loff_t *offset);
static int rl78_irq_release(struct inode *inode, struct file *file);
static int __exit rl78_irq_remove(struct platform_device *pdev);
static int __init rl78_irq_probe(struct platform_device *pdev);
static void rl78_irq_cleanup(void);

static char rl78_irq_flags;
static struct class *rl78_irq_class;
static int gpio_count;
static int *gpio_numbers;
static int *irq_numbers;
spinlock_t flag_lock;

/* The file operations struct */
static const struct file_operations rl78_irq_fops = {
	.owner = THIS_MODULE,
	.llseek       = no_llseek,
	.open = rl78_irq_open,
	.release = rl78_irq_release,
	.read = rl78_irq_read,
	.poll = rl78_irq_poll,
};

/* The of_device_id struct */
static struct of_device_id rl78_irq_match_table[] = {
     {
             .compatible = "hach,rl78_irq",
     },
     {0}
};
MODULE_DEVICE_TABLE(of, rl78_irq_match_table);

/* The driver struct */
static struct platform_driver rl78_irq_driver = {
	.probe = rl78_irq_probe,
	.remove = rl78_irq_remove,
	.driver = {
		.name = "hach,rl78_irq",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rl78_irq_match_table),
	},
};
module_platform_driver(rl78_irq_driver);


/******************************************************************************
 * rl78_irq_open
 *   Opens the rl78 interrupt device
 *
 * Parameters:
 *  inode - pointer to inode structure related to rl78 interrupt.
 *  file  - pointer to file structure directly linked to rl78 interrupt
 *          device file.
 *
 * Return Values:
 *   0 - always
 *
 *****************************************************************************/
static int rl78_irq_open(struct inode *inode, struct file *file)
{
	unsigned long flags;

	PDEBUG(KERN_DEBUG, "Open file\n");
	spin_lock_irqsave(&flag_lock, flags);
	rl78_irq_flags = 0;
	spin_unlock_irqrestore(&flag_lock, flags);
	return 0;
}


/******************************************************************************
 * rl78_irq_release
 *   Closes the rl78 interrupt device.
 *
 * Parameters:
 *  inode - pointer to inode structure related to rl78 interrupt.
 *  file - pointer to file structure directly linked to rl78 interrupt
 *         device file.
 *
 * Return Values:
 *   0 - always.
 *
 *****************************************************************************/
static int rl78_irq_release(struct inode *inode, struct file *file)
{
	PDEBUG(KERN_DEBUG, "Release file\n");
	return 0;
}


/******************************************************************************
 * rl78_irq_read
 * Copies to the user space the status of the RL78s interrupt request lines
 * and copies it to the user space
 *
 * Parameters:
 *   file   - pointer to file structure related to rl78 device.
 *   buf    - pointer to received data .
 *   count  - amount of bytes to be read.
 *   offset - unused.
 *
 * Return Values:
 *   <0  - in case of error,
 *   >0  - number of bytes received, in case of success.
 * 
 *****************************************************************************/
static ssize_t rl78_irq_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	ssize_t ret = 0;
	int32_t req_lines = 0;
	unsigned long flags;

	spin_lock_irqsave(&flag_lock, flags);

	req_lines = rl78_req_line_read();

	/* clear rl78 interrupt flag */
	rl78_irq_flags = rl78_irq_flags & ~RL78_IRQ;

	spin_unlock_irqrestore(&flag_lock, flags);

	if (count > sizeof(req_lines))
		count = sizeof(req_lines);

	ret = copy_to_user(buf, &req_lines, count) ? -EFAULT : ret;

	PDEBUG(KERN_DEBUG, "Request lines: 0x%08X\n", req_lines);

	return count;
}


/******************************************************************************
 * rl78_req_line_read
 * Reads the status of the RL78s interrupt request lines.
 *
 * Parameters:
 *
 * Return Values:
 *   A 32 bit Value 
 *   each bit from 0 to 18 represents a line (b0:IRQ1, b1:IRQ1 ... b18:IRQ19)
 *
 *****************************************************************************/
static int32_t rl78_req_line_read(void)
{
	int32_t retval = 0;
	unsigned int gpio_index;

	for (gpio_index = 0; gpio_index < gpio_count; gpio_index++)
		retval |= (gpio_get_value(gpio_numbers[gpio_index]) ^ 0x01) << gpio_index;

	PDEBUG(KERN_DEBUG, "Request lines read: all GPIO: %i\n", retval);
	return retval;
}


/******************************************************************************
 * rl78_irq_poll
 * Polls for rl78 interrupt occurence.
 *
 * Parameters:
 *  file - pointer to file structure directly linked to openened
 *         rl78 interrupt device file.
 *  wait - pointer to poll_table structure.
 *
 * Return Values:
 *   >0     - if i2c device file is ready for read,
 *   0      - if i2c device is not ready for read.
 *
 *****************************************************************************/
static unsigned int rl78_irq_poll(struct file *file, struct poll_table_struct *wait)
{
	unsigned int mask = 0;
	int32_t req_lines = 0;
	char flag_cpy;
	unsigned long flags;

	poll_wait(file, &rl78_irq_queue, wait);

	/*
	 * check if RL78_IRQ has occurred and
	 * at least one request line is active ->
	 * device is ready to be read from
	 */

	spin_lock_irqsave(&flag_lock, flags);
	flag_cpy = rl78_irq_flags;
	spin_unlock_irqrestore(&flag_lock, flags);

	if ((flag_cpy & RL78_IRQ) != 0) {
		req_lines = rl78_req_line_read();
		if (req_lines > 0) {
			/* indicate data are available to be read on i2c */
			mask |= POLLIN | POLLRDNORM;
		} else {
			/*
			 * plausibility check,
			 * interrupt without at least one active request
			 * line should never occur
			 */
			if (req_lines == 0)
				printk(KERN_DEBUG "rl78_drv: ERR - No RL78 request(s) detected\n");
		}
	}

	PDEBUG(KERN_DEBUG, "Poll returns %04X\n", mask);
	return mask;
}


/******************************************************************************
 * rl78_irq_cleanup
 * Cleanup the collective RL78 requested interrupts, gpios
 * and the kmalloc allocated memory
 *
 * Parameters:
 *  None
 *
 * Return Values:
 *  None
 *
 *****************************************************************************/
void rl78_irq_cleanup(void)
{
	unsigned long flags;
	unsigned int gpio_index;
	
	// Free GPIOs and IRQs allocated resources
	PDEBUG(KERN_DEBUG, "Free GPIOs, IRQs and memory allocated by kmalloc\n");
	for (gpio_index = 0; gpio_index < gpio_count; gpio_index++){
		free_irq(irq_numbers[gpio_index], (void *)(1 << gpio_index));
		gpio_free(gpio_numbers[gpio_index]);
	}

	// Free memory allocated by kmalloc
	kfree(gpio_numbers);
	kfree(irq_numbers);

	spin_lock_irqsave(&flag_lock, flags);
	rl78_irq_flags = 0;
	spin_unlock_irqrestore(&flag_lock, flags);
}


/******************************************************************************
 * rl78_irq_handler
 *   Handler for RL78 request interrupt.
 *
 * Parameters:
 *   irq - interrupt number.
 *   ptr  - .
 *
 * Return Values:
 *   IRQ_HANDLED      - always.
 *
 * Remarks:
 *
 *****************************************************************************/
static irqreturn_t rl78_irq_handler(int irq, void *ptr)
{
	unsigned long flags;
	unsigned long flagsirq;

	local_irq_save(flags);

	PDEBUG(KERN_DEBUG, "IRQ handler called\n");

	spin_lock_irqsave(&flag_lock, flagsirq);

	/* set flag to indicate rl78 interrupt occurence */
	rl78_irq_flags |= RL78_IRQ;

	spin_unlock_irqrestore(&flag_lock, flagsirq);

	/* wake up interrupt processing function and return from interrupt */
	wake_up_interruptible(&rl78_irq_queue);
	local_irq_restore(flags);

	return IRQ_HANDLED;
}


/******************************************************************************
 * rl78_irq_remove
 *   Handler for RL78 request interrupt.
 *
 * Parameters:
 *   *pdev - pointer to struct platform device.
 *
 * Return Values:
 *   0 - always.
 *
 * Remarks:
 *
 *****************************************************************************/
static int rl78_irq_remove(struct platform_device *pdev){
	printk(KERN_DEBUG "rl78_drv: Remove RL78 IRQ Driver\n");
	device_destroy(rl78_irq_class, MKDEV(RL78_IRQ_MAJOR,0));
	class_destroy(rl78_irq_class);
	unregister_chrdev(RL78_IRQ_MAJOR, "rl78_irq");
	rl78_irq_cleanup();		
	return 0;
}


/******************************************************************************
 * rl78_irq_probe
 *   Handler for RL78 request interrupt.
 *
 * Parameters:
 *   *pdev - pointer to struct platform device.
 *
 * Return Values:
 *   0  - on succes
 *   <0 - on error
 *
 * Remarks:
 *
 *****************************************************************************/
static int rl78_irq_probe(struct platform_device *pdev){
	int gpio_index,irq_number,ret;
	char name[11];
	struct device_node *np = pdev->dev.of_node;

	printk(KERN_DEBUG "rl78_drv: Initialize RL78 IRQ Driver\n");
	spin_lock_init(&flag_lock);
	PDEBUG(KERN_DEBUG, "Request GPIOs and IRQs\n");
	gpio_count = of_gpio_count(np);
	gpio_numbers = (int *) kmalloc(gpio_count*sizeof(int),GFP_KERNEL);
	irq_numbers = (int *) kmalloc(gpio_count*sizeof(int),GFP_KERNEL);
	if (gpio_numbers == NULL || irq_numbers == NULL){
		printk(KERN_DEBUG "rl78_drv: ERR - Unable to allocate memory for gpio_numbers and/or irq_numbers array(s) with kmalloc\n");
		return -1;
	}	
	for (gpio_index = 0; gpio_index < gpio_count; gpio_index++){		
		/* Get GPIOs */
		gpio_numbers[gpio_index] = of_get_gpio(np, gpio_index);
		sprintf(name,"RL78_IRQ%d",(gpio_index+1));		
		
		/* request IRQs */
		irq_numbers[gpio_index] = gpio_to_irq(gpio_numbers[gpio_index]);
		ret = request_irq(irq_numbers[gpio_index], rl78_irq_handler, IRQF_SHARED | IRQF_TRIGGER_FALLING, name, (void *)(1 << gpio_index));
		if (ret < 0) {			
			printk(KERN_DEBUG, "rl78_drv: ERR - Request irq failed for %s (GPIO#:%d, IRQ#:%d)\n",name,gpio_numbers[gpio_index],irq_numbers[gpio_index]);
			return -EIO;
		}
		PDEBUG(KERN_DEBUG, "%s: GPIO#=%d, IRQ#=%d\n",name,gpio_numbers[gpio_index],irq_numbers[gpio_index]);
	}				

	/* Register the char device */
	PDEBUG(KERN_DEBUG, "Register RL78 IRQ device\n");
	if (register_chrdev(RL78_IRQ_MAJOR, "rl78_irq", &rl78_irq_fops)) {
		printk(KERN_DEBUG "rl78_drv: ERR - Unable to get major %d for RL78_IRQ\n", RL78_IRQ_MAJOR);
		rl78_irq_cleanup();
		return -1;
	}

	/* Create the virtual device file*/
	PDEBUG(KERN_DEBUG, "Create virtual device file\n");
	rl78_irq_class = class_create(THIS_MODULE, "rl78_irq_class");
	if (IS_ERR(rl78_irq_class)) {
		printk(KERN_DEBUG "rl78_drv: ERR - Unable to create virtual device file. class_create error\n");		
		unregister_chrdev(RL78_IRQ_MAJOR, "rl78_irq");
		rl78_irq_cleanup();		
		return PTR_ERR(rl78_irq_class);	
	}
	if (device_create(rl78_irq_class,NULL,MKDEV(RL78_IRQ_MAJOR,0),NULL,"rl78_irq") == NULL){
		printk(KERN_DEBUG "rl78_drv: ERR - Unable to create virtual device file. device_create error\n");	
		class_destroy(rl78_irq_class);
		unregister_chrdev(RL78_IRQ_MAJOR, "rl78_irq");
		rl78_irq_cleanup();		
		return -1;		
	}

	return 0;
}


MODULE_VERSION("1.2");
MODULE_AUTHOR("Guillaume Deroire <guillaume.deroire@hach.com>");
MODULE_DESCRIPTION("rl78 irq device driver");
MODULE_LICENSE("GPL");

