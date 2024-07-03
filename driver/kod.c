//** Kernel headers **//

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/kdev_t.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/dma-mapping.h>  
#include <linux/mm.h>

//** Driver Information **//
MODULE_AUTHOR("Petar Ubavic, Jovan Ikic");
MODULE_DESCRIPTION("FPU IP core driver");
MODULE_LICENSE("Dual BSD/GPL");

#define      DRIVER_NAME     "fpu_driver" 
#define      BUFF_SIZE 	    200
#define      ARRAY_SIZE  	5

//** Global variables **//
int endRead = 0;
int cntr = 0;
int cntrIn = 0;
int ctnrOut = 0;
int posIn = 0;
int posOut = 0;


//** DMA defines **//



/*This function is called when module is loadaed in kernel */
static int __init ModuleInit(void) {
    printk("Hello, Kernel\n");           // printk() prints to kernel logs
    return 0;                            // 0 means that module loaded succesfully
}

/*This function is called when module is removed from kernel*/
static void __exit ModuleExit(void) {
    printk("Goodbye, Kernel\n");
}

module_init(ModuleInit);
module_exit(ModuleExit);