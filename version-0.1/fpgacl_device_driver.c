/* File: fpgacl_device_driver.c
 *
 Copyright (c) [2016] [Mohammad Hosseinabady (mohammad@hosseinabady.com)]
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
===============================================================================
* This file has been written at University of Bristol
* for the ENPOWER project funded by EPSRC
*
* File name : fpgacl_device_driver.c
* author    : Mohammad hosseinabady mohammad@hosseinabady.com
* date      : 1 October 2016
* blog: https://highlevel-synthesis.com/
*/

#define __CACHEABLE__
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
//#include <linux/device.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <asm/page.h>
#include <linux/dma-mapping.h>
#include <asm/siginfo.h>
#include <linux/rcupdate.h>
/* Needed for IO Read/Write Functions */
#include <linux/proc_fs.h> /* Needed for Proc File System Functions */
#include <linux/seq_file.h> /* Needed for Sequence File Operations */
//#include <linux/platform_device.h> /* Needed for Platform Driver Functions */
#include "fpgacl_device_driver.h"
#include <linux/of.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/time.h>
#include <linux/cdev.h>

#define DMA_ALLOC_COHERENT

//===================================================================================
//     memory allocation
//===================================================================================
u32 total_available_CMA_pages;
void fpga_memfree(int user_ref);

struct allocated_memory_struct {
	u32   physical_address;
	void* kernel_address;
	u32   size;
	int   vm_start;
	long unsigned int direction;
	struct allocated_memory_struct* next;
};
typedef struct allocated_memory_struct allocated_memory_type;
allocated_memory_type *allocated_memory_head;
allocated_memory_type *allocated_memory_tail;


//===================================================================================

//static struct platform_device *pdev;
static struct class *enpower_opencl_mohammad_cl;

dev_t        dev_numbers;
struct cdev* enpower_opencl_mohammad_cdev;
static int   enpower_opencl_mohammad_device_open = 0;
struct device *device_fpga;

u32 user_process_pid;
struct task_struct *user_process_struct;
struct siginfo info;

void  *device_kernel_address;
//=============================================================
struct timespec ts_start;
struct timespec ts_end;
struct timespec ts_duration;



//==========================================================

static int register_userspce_process_for_sending_signal(u32 user_process_pid);
static ssize_t  enpower_opencl_mohammad_read(struct file * file, char __user * buffer,	size_t        length,  loff_t *  offset);
static ssize_t enpower_opencl_mohammad_write(	struct file *file, const char __user * buffer, size_t length, loff_t * offset) ;
long  enpower_opencl_mohammad_ioctl(struct        file *file, unsigned int  ioctl_num,	unsigned long ioctl_param);
static int enpower_opencl_mohammad_open(struct inode *inode, struct file *file);
static int enpower_opencl_mohammad_release(struct inode *inode, struct file *file);
static int enpower_register_irq(u32 device_physical_base_address, void  *device_kernel_address);
static void  *enpower_ioremap(long unsigned int device_physical_base_address);
static int enpower_opencl_mohammad_init(void);
static void enpower_opencl_mohammad_exit(void);
static void fpga_dma_sync_single_for_cpu(void);
static void fpga_dma_sync_single_for_device(void);


//===========================================================

static irqreturn_t isr(int irq,void* dev_id)
{
	u32 int_en;
	u32 state;
	volatile u32 data_cntrl;

	void* device_kernel_base_address = (void*)dev_id;



//	getnstimeofday(&ts_end);
//	ts_duration = timespec_sub(ts_end,ts_start);
//	printk(KERN_ERR "isr = started at %lu\n", ts_duration.tv_nsec);

#ifdef __CACHEABLE__
	fpga_dma_sync_single_for_cpu();
#endif
    state = send_sig_info(SIGUSR1, &info, user_process_struct);    //send the signal
	if (state < 0) {
		printk(KERN_ERR "error sending signal\n\r");
		return -1;
	}

	data_cntrl = ioread32(device_kernel_base_address+0x0);

    int_en = ioread32(device_kernel_base_address+0x4);
    iowrite32(int_en & 0x00, device_kernel_base_address+0x4);

    int_en = ioread32(device_kernel_base_address+0x08);
    iowrite32(int_en & 0x00, device_kernel_base_address+0x08);

    iowrite32(0x01, device_kernel_base_address+0x0C); // important to clear interrupt



    return IRQ_HANDLED;
}

//==========================================================

static void fpga_dma_sync_single_for_cpu(void) {



	allocated_memory_type *current_allocated_mem = allocated_memory_head;

	while (current_allocated_mem != NULL) {
		if (current_allocated_mem->direction == 2 || current_allocated_mem->direction == 3 ) {
			dma_sync_single_for_cpu(
					NULL,
					current_allocated_mem->physical_address,
					current_allocated_mem->size,
					DMA_FROM_DEVICE
				);
		}
		current_allocated_mem = current_allocated_mem->next;
	}
}


static void fpga_dma_sync_single_for_device(void) {



	allocated_memory_type *current_allocated_mem = allocated_memory_head;



	while (current_allocated_mem != NULL) {

		if (current_allocated_mem->direction == 1 || current_allocated_mem->direction == 3 ) {

			dma_sync_single_for_device(
					NULL,
					current_allocated_mem->physical_address,
					current_allocated_mem->size,
					DMA_TO_DEVICE
				);
		}

/*
	       struct dma_map_ops *ops = get_dma_ops(NULL);

	       if (ops->sync_single_for_device) {
	    	   printk(KERN_ERR "Hello from sync_single_for_device \n");
	    	   ops->sync_single_for_device(				NULL,
	   				current_allocated_mem->physical_address,
	   				current_allocated_mem->size,
	   				DMA_TO_DEVICE);
	       }
*/

		current_allocated_mem = current_allocated_mem->next;
	}

}



static ssize_t  enpower_opencl_mohammad_read(struct file * file,	// see include/linux/fs.h
                                                 char __user * buffer,	// buffer to be filled with data 
                                                 size_t        length,	        // length of the buffer
                                                 loff_t *      offset) {


    return 0;
}



static ssize_t enpower_opencl_mohammad_write(	struct file *file,
												const char __user * buffer, 
												size_t length, 
												loff_t * offset) {

    return 0;
}



u32 get_physical_address(u32 vm_start, u32 direction) {
	allocated_memory_type *current_allocated_mem = allocated_memory_head;



	if (current_allocated_mem != NULL) {
		if (current_allocated_mem->vm_start == vm_start) {
			current_allocated_mem->direction = direction;
			return current_allocated_mem->physical_address;
		} else {
			while (current_allocated_mem->next != NULL) {
				if (current_allocated_mem->next->vm_start == vm_start) {
					current_allocated_mem->next->direction = direction;
					return current_allocated_mem->next->physical_address;
				}
				current_allocated_mem = current_allocated_mem->next;
			}
			if (current_allocated_mem->next == NULL) {
				printk(KERN_ALERT "get_physical_address: Error free unallocated memory 01! \n");
			}
		}
	} else {
		printk(KERN_ALERT "get_physical_address: Error free unallocated memory 02! \n");
	}

	return 0;
}


long  enpower_opencl_mohammad_ioctl(struct        file *file,	      // ditto
		 	                        unsigned int  ioctl_num,	  // number and param for ioctl
		                            unsigned long ioctl_param) {  //ioctl_param is INLS
	u32 int_en = 0;
	u32 data_cntrl = 0;
	u32 return_value = 0;
	u32 physical_address = 0;
	int register_value = 0;
	register_control_status_command_type *arg_param = kmalloc(sizeof(register_control_status_command_type), GFP_KERNEL);
	copy_from_user(arg_param,  (register_control_status_command_type *)ioctl_param, sizeof(register_control_status_command_type));



    switch(ioctl_num) {
    case FPGACL_DEVICE_IOWRITE:
    	if (arg_param->pointer == 1) {
    		physical_address = get_physical_address(arg_param->value, arg_param->direction);
    		if (physical_address != 0) {
    			register_value = physical_address/arg_param->size_type;
    		}
    		else
    			printk(KERN_ERR "error FPGACL_DEVICE_IOWRITE for address vm_start 0x%x direction = 0x%x\n", arg_param->value, arg_param->direction);
    	} else {
    		register_value = arg_param->value;
    	}

    	iowrite32(register_value, arg_param->device_kernel_base_address+arg_param->register_physical_offset_address);
    	break;

    case FPGACL_DEVICE_IOREAD:

    	return_value = ioread32(arg_param->device_kernel_base_address+arg_param->register_physical_offset_address);

    	break;
	case FPGACL_START:
		//enable accelerator
#ifdef __CACHEABLE__
		fpga_dma_sync_single_for_device();
#endif
//		getnstimeofday(&ts_start);
		data_cntrl = ioread32(arg_param->device_kernel_base_address+arg_param->register_physical_offset_address) & 0x80;
        iowrite32(data_cntrl | 0x01, arg_param->device_kernel_base_address+arg_param->register_physical_offset_address);

        break;
            
	case  FPGACL_CTRL_DONE:
		data_cntrl = ioread32(arg_param->device_kernel_base_address+arg_param->register_physical_offset_address);
	    return_value = (data_cntrl >> 1)&0x1;
		break;

	case  FPGACL_CTRL_READY:
		data_cntrl = ioread32(arg_param->device_kernel_base_address+arg_param->register_physical_offset_address);
	    return_value = (data_cntrl >> 3)&0x1;
		break;

	case FPGACL_INTERRUPT_ENABLE:

        int_en = ioread32(arg_param->device_kernel_base_address+arg_param->register_physical_offset_address);
        iowrite32(int_en | 0x01, arg_param->device_kernel_base_address+arg_param->register_physical_offset_address);

        int_en = ioread32(arg_param->device_kernel_base_address+arg_param->register_physical_offset_address+0x04);
        iowrite32( int_en | 0x01, arg_param->device_kernel_base_address+arg_param->register_physical_offset_address+0x04);
        break;

	case FPGACL_REGISTER_DEVICE:
		/*
		* Register Device
		*/

		device_kernel_address = enpower_ioremap(arg_param->device_physical_base_address);
		enpower_register_irq(arg_param->irq_num, device_kernel_address);

		printk(KERN_ERR "enpower_opencl_mohammad_ioctl: process_id = %d\n", arg_param->process_id);

		register_userspce_process_for_sending_signal(arg_param->process_id);


		return_value = (u32)device_kernel_address;
		break;

    case FPGACL_MEMFREE:
    	printk(KERN_ALERT "free memory at physical_address 0x%x \n",(u32)ioctl_param );
    	fpga_memfree((int)ioctl_param);
		break;

	default:
	
		break;

    }
    


	return return_value;

}


static int register_userspce_process_for_sending_signal(u32 user_process_pid) {

 	rcu_read_lock();
	user_process_struct = pid_task(find_pid_ns(user_process_pid, &init_pid_ns), PIDTYPE_PID);
	if(user_process_struct == NULL){
		printk(KERN_ALERT "no such pid\n");
		rcu_read_unlock();
		return -ENODEV;
	}
	rcu_read_unlock();

	memset(&info, 0, sizeof(struct siginfo));
	info.si_signo = SIGUSR1;
	info.si_code = SI_QUEUE;
	info.si_int = 1234;



	return 0;
}


static int enpower_opencl_mohammad_open(struct inode *inode, struct file *file) {

	u32 return_value = 0;


    
    if(enpower_opencl_mohammad_device_open)
    	return_value=-EBUSY;
     
    enpower_opencl_mohammad_device_open++;




    return return_value;
}



static int enpower_opencl_mohammad_release(struct inode *inode, struct file *file){

    enpower_opencl_mohammad_device_open--;


    return 0;
}


static int data_transfer_mohammad_mmap(struct file *file_p, struct vm_area_struct *vma)
{


    int ret=0;
	void* linux_kernel_memory;
	u32 data_phys_add;
	allocated_memory_type *mem;
	long length = vma->vm_end - vma->vm_start;

	//check maximum nuber of available CMA memory
	u32 number_of_required_page = length/PAGE_SIZE;
	number_of_required_page++;  //for the remaining memory in the last page
    if (number_of_required_page > total_available_CMA_pages)
    	return -EIO;
    length = (number_of_required_page+2)*PAGE_SIZE;
    total_available_CMA_pages = total_available_CMA_pages - number_of_required_page;
    printk(KERN_ALERT "total_available_CMA_pages = %d \n", total_available_CMA_pages);

#ifdef DMA_ALLOC_COHERENT
    printk(KERN_ALERT "data length = %d \n", length);
    linux_kernel_memory = dma_alloc_coherent( NULL, length, &data_phys_add, GFP_KERNEL );
    //linux_kernel_memory = dma_alloc_coherent( NULL, length, &data_phys_add, GFP_KERNEL | GFP_DMA | __GFP_REPEAT );
    //linux_kernel_memory = dma_alloc_noncoherent( NULL, length, &data_phys_add, GFP_KERNEL | GFP_DMA | __GFP_REPEAT );

    printk(KERN_ALERT "Using dma_alloc_coherent\n");
#else
    linux_kernel_memory = kmalloc(length, GFP_KERNEL);
    printk(KERN_ALERT "Using kmalloc\n");
#endif
    data_phys_add = virt_to_phys(linux_kernel_memory);
    if (!linux_kernel_memory) {
    	printk(KERN_ALERT "kernel memory allocation failed for argument %d\n" );
    	return -ENODEV;
    } else {
    	printk(KERN_ALERT "%ld kernel memory allocation success for mem  at address 0x%x\n", length, (u32)data_phys_add);

    }


/* #ifdef ARCH_HAS_DMA_MMAP_COHERENT */
    if (vma->vm_pgoff == 0) {
    	printk(KERN_ALERT "Using dma_mmap_coherent\n");
#ifndef __CACHEABLE__
    	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
#endif
    	//ret = dma_mmap_coherent(NULL,        vma, linux_kernel_memory,   data_phys_add, length);
		ret = remap_pfn_range(vma, vma->vm_start,
				data_phys_add >> PAGE_SHIFT,
				vma->vm_end - vma->vm_start,
				vma->vm_page_prot);

    } else {

    	printk(KERN_ALERT "Using remap_pfn_range\n");
    	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

    	vma->vm_flags |= VM_IO;
    	printk(KERN_INFO "off=%lu\n", vma->vm_pgoff);
        ret = remap_pfn_range(vma, vma->vm_start,
		      PFN_DOWN(data_phys_add) +
		      vma->vm_pgoff, length, vma->vm_page_prot);
    }

    /* map the whole physically contiguous area in one piece */
    if (ret < 0) {
    	printk(KERN_ERR "mmap_alloc: remap failed (%d)\n", ret);
    	return ret;
    }

	mem = (allocated_memory_type *)kmalloc(sizeof(allocated_memory_type), GFP_KERNEL);

	mem->physical_address     = data_phys_add;
	mem->kernel_address       = linux_kernel_memory;
	mem->size                 = length;
	mem->next                 = NULL;
	mem->vm_start             = vma->vm_start;
	mem->direction            = 0;

	if (allocated_memory_head == NULL) {
		allocated_memory_head = mem;
		allocated_memory_tail = mem;
	} else {
		allocated_memory_tail->next = mem;
		allocated_memory_tail       = mem;
	}



    return ret;
}


void fpga_memfree(int vm_start) {

	allocated_memory_type *current_allocated_mem = allocated_memory_head;
	allocated_memory_type *tmp;


	if (current_allocated_mem != NULL) {
		if (current_allocated_mem->vm_start == vm_start) {
			dma_free_coherent(NULL, current_allocated_mem->size, current_allocated_mem->kernel_address, current_allocated_mem->physical_address);
			allocated_memory_head=current_allocated_mem->next;
			if (allocated_memory_head==NULL)
				allocated_memory_tail=NULL;
			kfree(current_allocated_mem);
		} else {
			while (current_allocated_mem->next != NULL) {
				if (current_allocated_mem->next->vm_start == vm_start) {
					dma_free_coherent(NULL, current_allocated_mem->next->size, current_allocated_mem->next->kernel_address, current_allocated_mem->next->physical_address);
					break;
				}
				current_allocated_mem = current_allocated_mem->next;
			}
			if (current_allocated_mem->next == NULL) {
				printk(KERN_ALERT "fpga_memfree: Error free unallocated memory! \n");
			} else {
				tmp = current_allocated_mem->next;
				current_allocated_mem->next = current_allocated_mem->next->next;
				kfree(tmp);
			}
		}
	} else {
		printk(KERN_ALERT "fpga_memfree: Error free unallocated memory! \n");
	}
}


struct file_operations enpower_opencl_mohammad_fops = {
	.owner = THIS_MODULE,
	.read = enpower_opencl_mohammad_read,
	.write = enpower_opencl_mohammad_write,
	.unlocked_ioctl = enpower_opencl_mohammad_ioctl,
	.open    = enpower_opencl_mohammad_open,
	.release = enpower_opencl_mohammad_release,
	.mmap	 = data_transfer_mohammad_mmap
};

static struct device_node *gic_node;

static struct of_device_id gic_match[] = {
	{ .compatible = "arm,cortex-a9-gic", },
	{ .compatible = "arm,cortex-a15-gic", },
	{ },
};

unsigned int enpower_kernel_irq_num(unsigned int hwirq)
{

	struct of_phandle_args irq_data;
	unsigned int irq;



	if (!gic_node)
		gic_node = of_find_matching_node(NULL, gic_match);

	if (WARN_ON(!gic_node))
		return hwirq;

	irq_data.np = gic_node;
	irq_data.args_count = 3;
	irq_data.args[0] = 0;
	irq_data.args[1] = hwirq - 32; /* GIC SPI offset */
	irq_data.args[2] = IRQ_TYPE_EDGE_RISING;//IRQ_TYPE_LEVEL_HIGH;

	irq = irq_create_of_mapping(&irq_data);
	if (WARN_ON(!irq))
		irq = hwirq;

	pr_err("%s: hwirq %d, irq %d\n", __func__, hwirq, irq);

	return irq;
}


static int enpower_register_irq(u32 irq_num, void  *device_kernel_address) {

	int err;
	int irq_hardware;
	int irq;



	irq_hardware = irq_num;

	irq = enpower_kernel_irq_num(irq_hardware);
	pr_err("get_resource for IRQ for dev %d \n", irq);
	if (irq <= 0) {
		pr_err("get_resource for IRQ for dev %d failed\n", irq);
		return -ENODEV;
	}


	err = devm_request_irq(device_fpga, irq, isr, IRQF_SHARED, "axi_timer_dev", device_kernel_address);
	if (err) {
		pr_err("devm_request_irq failed\n");
		return err;
	}


	return err;
}


static void  *enpower_ioremap(long unsigned int device_physical_base_address) {

	void *enpower_device_kernel_base_address;


	enpower_device_kernel_base_address = ioremap(device_physical_base_address, FPGACL_DEVICE_NO_PORT);

	return enpower_device_kernel_base_address;
}



static int enpower_opencl_mohammad_init(void) {

    int   state;


    char  device_name[]=DEVICE_NAME;

    state = alloc_chrdev_region(&dev_numbers, 0, 1, device_name);
    if(state!=0) {
        printk(KERN_ERR "failed to register a region dynamically\n\r");
    } else {
        printk(KERN_ERR "major number = %d\n\r", MAJOR(dev_numbers));
    }

    if ((enpower_opencl_mohammad_cl = class_create(THIS_MODULE, "chardrv")) == NULL) {
    	unregister_chrdev_region(dev_numbers, 1);
     return -1;
    }
    device_fpga = device_create(enpower_opencl_mohammad_cl, NULL, dev_numbers, NULL, "fpga_dev");
    if (device_fpga == NULL) {
    	class_destroy(enpower_opencl_mohammad_cl);
     unregister_chrdev_region(dev_numbers, 1);
     return -1;
     }



    enpower_opencl_mohammad_cdev        = cdev_alloc();
    enpower_opencl_mohammad_cdev->ops   = &enpower_opencl_mohammad_fops;
    enpower_opencl_mohammad_cdev->owner = THIS_MODULE;

    cdev_init(enpower_opencl_mohammad_cdev, &enpower_opencl_mohammad_fops);
    state = cdev_add(enpower_opencl_mohammad_cdev, dev_numbers , 1);
    if(state < 0) {
        device_destroy(enpower_opencl_mohammad_cl, dev_numbers);
        class_destroy(enpower_opencl_mohammad_cl);
        unregister_chrdev_region(dev_numbers, 1);
        printk(KERN_ERR "device failed to be added\n\r");
    }




    total_available_CMA_pages = NPAGES;

    allocated_memory_head = NULL;
    allocated_memory_tail = NULL;


    return 0;
}


static void enpower_opencl_mohammad_exit(void) {

	int i = 0;
	allocated_memory_type *tmp;
	allocated_memory_type *current_allocated_mem = allocated_memory_head;
//	printk(KERN_ERR "current_allocated_mem = 0x%x\n\r", current_allocated_mem);
	while(current_allocated_mem != NULL) {
		printk(KERN_ERR "%d\n\r", i++);
		tmp = current_allocated_mem;
		current_allocated_mem = current_allocated_mem->next;
#ifdef DMA_ALLOC_COHERENT
		dma_free_coherent(NULL, tmp->size, tmp->kernel_address, tmp->physical_address);
#else
		kfree(tmp->kernel_address);
#endif
		kfree(tmp);
	}

//	free_irq(IRQ_NUM, NULL);
	//platform_device_unregister(pdev);
    cdev_del(enpower_opencl_mohammad_cdev);
    device_destroy(enpower_opencl_mohammad_cl, dev_numbers);
	class_destroy(enpower_opencl_mohammad_cl);
    unregister_chrdev_region(dev_numbers, 1);

    printk(KERN_ALERT "Bye fpga device driver!\n\r");

}


module_init(enpower_opencl_mohammad_init);
module_exit(enpower_opencl_mohammad_exit);
MODULE_AUTHOR ("Mohammad Hosseinabady");
MODULE_DESCRIPTION("FPGA OpenCL driver.");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("custom:fpgacl_device_driver");
