/*******************************************************************************
 *
 * (C) Copyright 2014 Hach-Lange
 *
 * Project: Fusion
 * Purpose: Contains main defines for rl78 irq device driver
 *
 * File:  rl78_drv.h
 * Desc:  
 *
 * Revision History:
 * 2014-07-17 created.
 * 2019-03-13 module version 1.2
 * 	- Use Device Tree for definition and configuration of GPIOs and IRQs 
 ********************************************************************************/

#ifndef _RL78_DRV_H
#define _RL78_DRV_H

#undef DEBUG
/* Uncomment to enable debug logs */
/*#define DEBUG*/

/* To test driver, you can use the following command to poll the rl78_irq content */
/* while [ true ] ; do hexdump -v -C -n 4 /dev/rl78_irq; sleep 1; done; */

#ifdef __KERNEL__
#define RL78_IRQ_MAJOR  245    /* RL78 irq device major number */
#endif

#ifdef DEBUG
	#define PDEBUG(level, fmt, args...) printk(level "rl78_drv: " fmt, ## args)
#else
	#define PDEBUG(level, fmt, args...) /* not debugging: nothing */
#endif

#endif
