//** Kernel headers **//		// 192.168.1.107 Port 22 - 555333

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
MODULE_DESCRIPTION("FPU Exp Driver");
MODULE_LICENSE("Dual BSD/GPL");

#define      DRIVER_NAME        "fpu_exp" 
#define      BUFF_SIZE 	        4096	//256 * 10 (256 hex brojeva, svaki ima 10 znakova i jos malo lufta)
#define      MAX_ARRAY_SIZE     256
#define      ARR_SIZE  	        256


//** DMA defines **//
#define MAX_PKT_LEN				4096	//(256*sizeof(float))

#define MM2S_DMACR_REG			0x00
#define MM2S_SA_REG				0x18
#define MM2S_LENGTH_REG			0x28
#define MM2S_STATUS_REG			0x04

#define S2MM_DMACR_REG			0x30
#define S2MM_DA_REG				0x48
#define S2MM_LENGTH_REG			0x58
#define S2MM_STATUS_REG			0x34

#define DMACR_RUN_STOP			1
#define DMACR_RESET				1<<2
#define IOC_IRQ_EN				1<<12
#define ERR_IRQ_EN				1<<14


//** Global variables **//
int endRead = 0;
int cntr = 0;
int cntrIn = 0;
int cntrOut = 0;
int posIn = 0;
int posOut = 0;
unsigned int write_counter = 0;

// DODATAK
static u32 *fpu_array = NULL;
static int arr_size = 0;
static int initialized = 0;
//////////////

//** Function Declerations **//

static int  __init fpu_init(void);
static void __exit fpu_exit(void);

static int  fpu_probe(struct platform_device *pdev);
static int  fpu_remove(struct platform_device *pdev);
int         fpu_open(struct inode *pinode, struct file *pfile);
int         fpu_close(struct inode *pinode, struct file *pfile);
ssize_t     fpu_read(struct file *pfile, char __user *buffer, size_t length, loff_t *offset);
ssize_t     fpu_write(struct file *pfile, const char __user *buffer, size_t length, loff_t *offset);
static int  fpu_mmap(struct file *f, struct vm_area_struct *vma_s);

static irqreturn_t dma_MM2S_isr(int irq, void* dev_id);
static irqreturn_t dma_S2MM_isr(int irq, void* dev_id);

int dma_init(void __iomem *base_address);
unsigned int dma_simple_write(dma_addr_t TxBufferPtr, unsigned int pkt_len, void __iomem *base_address); 
unsigned int dma_simple_read(dma_addr_t RxBufferPtr, unsigned int pkt_len, void __iomem *base_address);

//** Struct Declarations **//

struct fpu_info {
	unsigned long mem_start;
	unsigned long mem_end;
	void __iomem *base_addr;
	int irq_num0;
	int irq_num1;
};

dev_t my_dev_id;
static struct class *my_class;
static struct device *my_device;
static struct cdev *my_cdev;
static struct fpu_info *dma_p = NULL;

struct file_operations my_fops = {
	.owner 		= THIS_MODULE,
	.open 		= fpu_open,
	.release 	= fpu_close,
	.read 		= fpu_read,
	.write 		= fpu_write,
	.mmap		= fpu_mmap
};

static struct of_device_id fpu_of_match[] = {
	{ .compatible = "fp_exp_dma", },
	{ /* end of list */ },
};

MODULE_DEVICE_TABLE(of, fpu_of_match);

static struct platform_driver fpu_exp = {
	.driver = {
		.name 			= DRIVER_NAME,
		.owner 			= THIS_MODULE,
		.of_match_table	= fpu_of_match,
	},
	.probe		= fpu_probe,
	.remove		= fpu_remove,
};

dma_addr_t tx_phy_buffer;
dma_addr_t rx_phy_buffer;
u32 *tx_vir_buffer;
u32 *rx_vir_buffer;
volatile int transaction_over0 = 0;
volatile int transaction_over1 = 0;

//** Init & Exit Functions **//     /* VEZBA 5*/

/*This function is called when module is loadaed in kernel */       
static int __init fpu_init(void) {

	int ret = 0;
	int i = 0;
	printk(KERN_INFO "[fpu_init] Initialize Module \"%s\"\n", DRIVER_NAME);
	
	// Allocate character device region
	ret = alloc_chrdev_region(&my_dev_id, 0, 1, "fpu_region");
	if(ret) {
		printk(KERN_ALERT "[fpu_init] Failed CHRDEV!\n");
		return -1;
	}
	printk(KERN_INFO "[fpu_init] Successful CHRDEV!\n");
	
	// Create device class
	my_class = class_create(THIS_MODULE, "fpu_class");
	if(my_class == NULL) {
		printk(KERN_ALERT "[fpu_init] Failed class create!\n");
		goto fail_0;
	}
	printk(KERN_INFO "[fpu_init] Successful class chardev create!\n");
	
	// Create device
	my_device = device_create(my_class, NULL, MKDEV(MAJOR(my_dev_id), 0), NULL, "fpu_exp");
	if(my_device == NULL) {
		goto fail_1;
	}
	printk(KERN_INFO "[fpu_init] Device fpu_exp created\n");
	
	// Allocate and add character device
	my_cdev = cdev_alloc();	
	my_cdev->ops = &my_fops;
	my_cdev->owner = THIS_MODULE;
	ret = cdev_add(my_cdev, my_dev_id, 1);
	if(ret) {
		printk(KERN_ERR "[fpu_init] Failed to add cdev\n");
		goto fail_2;
	}
	printk(KERN_INFO "[fpu_init] Module init done\n");

    // Allocate coherent DMA buffer
	tx_vir_buffer = dma_alloc_coherent(NULL, MAX_PKT_LEN, &tx_phy_buffer, GFP_DMA | GFP_KERNEL);
	printk(KERN_INFO "[fpu_init] Virtual and physical TX addresses coherent starting at %#x and ending at %#x\n", tx_phy_buffer, tx_phy_buffer+(uint)(MAX_PKT_LEN));
	if(!tx_vir_buffer) {
		printk(KERN_ALERT "[fpu_init] Could not allocate dma_alloc_coherent TX\n");
		goto fail_3;
	}
	else {
		for (i = 0; i < MAX_PKT_LEN/4; i++)
        	tx_vir_buffer[i] = 0x00000000;
		printk("[fpu_init] Successfully allocated memory for transmission buffer\n");
	}

	rx_vir_buffer = dma_alloc_coherent(NULL, MAX_PKT_LEN, &rx_phy_buffer, GFP_DMA | GFP_KERNEL);
	printk(KERN_INFO "[fpu_init] Virtual and physical RX addresses coherent starting at %#x and ending at %#x\n", rx_phy_buffer, rx_phy_buffer+(uint)(MAX_PKT_LEN));
	if(!rx_vir_buffer) {
		printk(KERN_ALERT "[fpu_init] Could not allocate dma_alloc_coherent RX\n");
		goto fail_3;
	}
	else {
		for (i = 0; i < MAX_PKT_LEN/4; i++)
        	rx_vir_buffer[i] = 0x00000000;
		printk("[fpu_init] Successfully allocated memory for receiving buffer\n");
	}

	printk(KERN_INFO "[fpu_init] Memory reset.\n");

	
	return platform_driver_register(&fpu_exp);
	
	// Error handling and cleanup       //fail_3 nema na vezbama 5, ovde ima zog dma_alloc_coherent funkcije
	fail_3:
		cdev_del(my_cdev);
	fail_2:
		device_destroy(my_class, MKDEV(MAJOR(my_dev_id),0));
	fail_1:
		class_destroy(my_class);
	fail_0:
		unregister_chrdev_region(my_dev_id, 1);
	return -1;
} 

/*This function is called when module is removed from kernel*/
static void __exit fpu_exit(void) {

    /* Exit Device Module */
	dma_free_coherent(NULL, MAX_PKT_LEN, &tx_phy_buffer, GFP_DMA | GFP_KERNEL);
	dma_free_coherent(NULL, MAX_PKT_LEN, &rx_phy_buffer, GFP_DMA | GFP_KERNEL);
    platform_driver_unregister(&fpu_exp); //ove funckije nema u kodu sa 5ih vezbi, ali je pozivamo kako bi se "bezbednije" uklonio driver
	cdev_del(my_cdev);
	device_destroy(my_class, MKDEV(MAJOR(my_dev_id),0));
	class_destroy(my_class);
	unregister_chrdev_region(my_dev_id, 1);
	printk(KERN_INFO "[fpu_exit] Goodbye Kernel\"%s\".\n", DRIVER_NAME);

}

module_init(fpu_init);
module_exit(fpu_exit);

//** Probe & Remove Functions **//  /* VEZBE 9 i 10*/

static int fpu_probe(struct platform_device *pdev)  {
	struct resource *r_mem;
	int rc = 0;

    printk(KERN_INFO "[fpu_probe] Entered Probe\n");

	r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!r_mem){
	    printk(KERN_ALERT "[fpu_probe] Failed to get reg resource.\n");
	    return -ENODEV;
	}
	printk(KERN_INFO "[fpu_probe] Probing dma_p\n");

	dma_p = (struct fpu_info *) kmalloc(sizeof(struct fpu_info), GFP_KERNEL);
	
    if(!dma_p) {
		printk(KERN_ALERT "[fpu_probe] Could not allocate dma device\n");
		return -ENOMEM;
	}

	dma_p->mem_start = r_mem->start;
	dma_p->mem_end = r_mem->end;
	
    if(!request_mem_region(dma_p->mem_start, dma_p->mem_end - dma_p->mem_start + 1,	"dma_device")) {
		printk(KERN_ALERT "[fpu_probe] Could not lock memory region at %p\n",(void *)dma_p->mem_start);
		rc = -EBUSY;
		goto error01;
	}

	dma_p->base_addr = ioremap(dma_p->mem_start, dma_p->mem_end - dma_p->mem_start + 1);
	
    if (!dma_p->base_addr) {
		printk(KERN_ALERT "[fpu_probe] Could not allocate memory\n");
		rc = -EIO;
		goto error02;
	}

	printk(KERN_INFO "[fpu_probe] dma base address start at %#x\n", (u32)dma_p->base_addr);

	dma_p->irq_num0 = platform_get_irq(pdev, 0);
	
    if(!dma_p->irq_num0) {
		printk(KERN_ERR "[fpu_probe] Could not get IRQ0 resource for dma\n");
		rc = -ENODEV;
		goto error03;
	}

	if (request_irq(dma_p->irq_num0, dma_MM2S_isr, 0, "dma_device", dma_p)) {
		printk(KERN_ERR "[fpu_probe] Could not register M2SS IRQ %d\n", dma_p->irq_num0);
		return -EIO;
		goto error03;
	}

	else {
		printk(KERN_INFO "[fpu_probe] Registered M2SS IRQ %d\n", dma_p->irq_num0);
	}


	dma_p->irq_num1 = platform_get_irq(pdev, 1);
	
    if(!dma_p->irq_num1) {
		printk(KERN_ERR "[fpu_probe] Could not get IRQ1 resource for dma\n");
		rc = -ENODEV;
		goto error03;
	}


    if (request_irq(dma_p->irq_num1, dma_S2MM_isr, 0, "dma_device", dma_p)) {
		printk(KERN_ERR "[fpu_probe] Could not register S2MM IRQ %d\n", dma_p->irq_num1);
		return -EIO;
		goto error03;
	}

	else {
		printk(KERN_INFO "[fpu_probe] Registered S2MM IRQ %d\n", dma_p->irq_num1);
	}

	enable_irq(dma_p->irq_num0);
	enable_irq(dma_p->irq_num1);
	dma_init(dma_p->base_addr);
	printk(KERN_NOTICE "[fpu_probe] fpu platform driver registered - dma\n");
	
    return 0;

	error03:
		iounmap(dma_p->base_addr);
	error02:
		release_mem_region(dma_p->mem_start, dma_p->mem_end - dma_p->mem_start + 1);
		kfree(dma_p);
	error01:
		return rc;		

}

static int fpu_remove(struct platform_device *pdev)  {

	printk(KERN_ALERT "[fpu_remove] dma_p device platform driver removed\n");
	iowrite32(0, dma_p->base_addr);
	free_irq(dma_p->irq_num0, dma_p);
	free_irq(dma_p->irq_num1, dma_p);
	iounmap(dma_p->base_addr);
	release_mem_region(dma_p->mem_start, dma_p->mem_end - dma_p->mem_start + 1);
	kfree(dma_p);
	printk(KERN_INFO "[fpu_remove] Succesfully removed dma_p device platform driver\n");
	
	return 0;
}

//** Open & Close Functions **//  /**/

int fpu_open(struct inode *pinode, struct file *pfile) {
	printk(KERN_INFO "[fpu_open] Succesfully opened driver\n");
	return 0;
}

int fpu_close(struct inode *pinode, struct file *pfile) {
	printk(KERN_INFO "[fpu_close] Succesfully closed driver\n");
	return 0;
}

//** Read & Write Functions **//  /**/
/*This function read data from the driver and dipslays it to the user*/
ssize_t fpu_read(struct file *pfile, char __user *buf, size_t length, loff_t *offset) {		

	static int finished = 0;
    char *kernel_buf;
    int i;
    int ret;
    size_t len = 0;

    // Check if the array is initialized
    if (!initialized) {
        printk(KERN_WARNING "[fpu_read] Array not initialized\n");
        return 0;
    }

    // Check if read is already done
    if (finished) {
        finished = 0;  // Reset for the next call
        return 0;
    }

    // Allocate memory for the kernel buffer
    kernel_buf = kmalloc(BUFF_SIZE, GFP_KERNEL);
    if (!kernel_buf) {
        printk(KERN_ERR "[fpu_read] Memory allocation failed\n");
        return -ENOMEM;
    }

    // Populate the kernel buffer with the array values
    for (i = 0; i < arr_size; i++) {
		printk(KERN_INFO "[fpu_read] Izlazni_niz[%d]: %#010x\n", i, fpu_array[i]);

        len += snprintf(kernel_buf + len, BUFF_SIZE - len, "0x%08x", fpu_array[i]);
        if (i < arr_size - 1) {
            len += snprintf(kernel_buf + len, BUFF_SIZE - len, ", ");
        }
		else {
			len += snprintf(kernel_buf + len, BUFF_SIZE - len, "\n");
		}
        if (len >= BUFF_SIZE) {
            printk(KERN_WARNING "[fpu_read] Buffer size exceeded\n");
            kfree(kernel_buf);
            return -EFAULT;
        }
    }

    // Copy data to user space
    ret = copy_to_user(buf, kernel_buf, len);
    if (ret) {
        printk(KERN_WARNING "[fpu_read] Copy to user failed\n");
        kfree(kernel_buf);
        return -EFAULT;
    }

    kfree(kernel_buf);
    finished = 1;  // Mark read as done

    return len;
}

/*This function writes data to the driver*/
ssize_t fpu_write(struct file *pfile, const char __user *buf, size_t length, loff_t *offset) {	

	char kernel_buf[BUFF_SIZE];
    int ret;
    int pos;
    u32 value;

    // Check if the buffer length is within limits
    if (length >= BUFF_SIZE) {
        printk(KERN_WARNING "[fpu_write] Input too large\n");
        return -EFAULT;
    }

    // Copy data from user space
    ret = copy_from_user(kernel_buf, buf, length);
    if (ret) {
        printk(KERN_WARNING "[fpu_write] Copy from user failed\n");
        return -EFAULT;
    }
    
    kernel_buf[length] = '\0'; // Null-terminate the string

    // Check for initialization command
    if (sscanf(kernel_buf, "N=%d", &arr_size) == 1) {
        if (arr_size > 0 && arr_size <= MAX_ARRAY_SIZE) {
            // Allocate and initialize the array
            if (fpu_array != NULL) {
                kfree(fpu_array); // Free the previous array if it exists
            }
            fpu_array = kzalloc(arr_size * sizeof(u32), GFP_KERNEL);
            if (!fpu_array) {
                printk(KERN_ERR "[fpu_write] Memory allocation failed\n");
                return -ENOMEM;
            }
            initialized = 1;
            printk(KERN_INFO "[fpu_write] Array initialized with size %d\n", arr_size);
        } 
		
		else {
            printk(KERN_WARNING "[fpu_write] Invalid array size\n");
            return -EINVAL;
        }
    } 
    // Check for position=value command
    else if (sscanf(kernel_buf, "Pozicija=%d=0x%x", &pos, &value) == 2) {
        if (!initialized) {
            printk(KERN_WARNING "[fpu_write] Array not initialized\n");
            return -EINVAL;
        }
        if (pos >= 0 && pos < arr_size) {
            fpu_array[pos] = value;
			write_counter++;
            printk(KERN_INFO "[fpu_write] Position %d updated with value %#010x\n", pos, value);
        } else {
            printk(KERN_WARNING "[fpu_write] Invalid position\n");
            return -EINVAL;
        }
    } 
    // Invalid command
    else {
        printk(KERN_WARNING "[fpu_write] Invalid command format\n");
        return -EINVAL;
    }

	if(write_counter == arr_size) {
		for(pos = 0; pos < arr_size; pos++){
			*tx_vir_buffer = fpu_array[pos];
			dma_simple_write(tx_phy_buffer, sizeof(fpu_array), dma_p->base_addr);
		}
	}

    return length;
}

//** Mmap Function **//  /* VEZBA 12*/

static int fpu_mmap(struct file *f, struct vm_area_struct *vma_s) {
    int ret = 0;
    long length = vma_s->vm_end - vma_s->vm_start;

    printk(KERN_INFO "[fpu_mmap] Buffer is being memory mapped\n");
    printk(KERN_INFO "[fpu_mmap] Buffer TX Length: %ld\n", length);

    if (length > MAX_PKT_LEN) {
        printk(KERN_INFO "[fpu_mmap] Trying to mmap more space than it`s allocated\n");
        return -EIO;
    }

	// Map TX buffer
    printk(KERN_INFO "[fpu_mmap] Mapping TX Buffer\n");
    ret = dma_mmap_coherent(NULL, vma_s, tx_vir_buffer, tx_phy_buffer, length);

    if (ret < 0) {
        printk(KERN_INFO "[fpu_mmap] Memory map failed with error: %d\n", ret);
        return ret;
    }

	printk(KERN_INFO "[fpu_mmap] Memory map succeeded\n");
	printk(KERN_INFO "[fpu_mmap] Calling dma_simple_write\n");

	dma_simple_write(tx_phy_buffer, sizeof(float)*256, dma_p->base_addr);

    return 0;
}

//** DMA Functions **//  /*VEZBA 12*/

int dma_init(void __iomem *base_address) {

	u32 MM2S_DMACR_val = 0;
	u32 S2MM_DMACR_val = 0;
	u32 enInterrupt0 = 0;
	u32 enInterrupt1 = 0;

	iowrite32(0x0, base_address + MM2S_DMACR_REG);
	iowrite32(DMACR_RESET, base_address + MM2S_DMACR_REG);
	MM2S_DMACR_val = ioread32(base_address + MM2S_DMACR_REG);
	enInterrupt0 = MM2S_DMACR_val | IOC_IRQ_EN | ERR_IRQ_EN;
	iowrite32(enInterrupt0, base_address + MM2S_DMACR_REG);	
	printk(KERN_INFO "[dma_init] Successfully initialized MM2S DMA \n");
	
	iowrite32(0x0, base_address + S2MM_DMACR_REG);
	iowrite32(DMACR_RESET, base_address + S2MM_DMACR_REG);
	S2MM_DMACR_val = ioread32(base_address + S2MM_DMACR_REG);
	enInterrupt1 = S2MM_DMACR_val | IOC_IRQ_EN | ERR_IRQ_EN;
	iowrite32(enInterrupt1, base_address + S2MM_DMACR_REG);	
	printk(KERN_INFO "[dma_init] Successfully initialized S2MM DMA \n");

	return 0;
}

unsigned int dma_simple_write(dma_addr_t TxBufferPtr, unsigned int pkt_len, void __iomem *base_address) {

	u32 MM2S_DMACR_val = 0;
	u32 enInterrupt = 0;

	MM2S_DMACR_val = ioread32(base_address + MM2S_DMACR_REG);

	enInterrupt = MM2S_DMACR_val | IOC_IRQ_EN | ERR_IRQ_EN;
	iowrite32(enInterrupt, base_address + MM2S_DMACR_REG);

	MM2S_DMACR_val = ioread32(base_address + MM2S_DMACR_REG);
	MM2S_DMACR_val |= DMACR_RUN_STOP;

	printk(KERN_INFO "[dma_simple_write] Pre: %#010x \n", TxBufferPtr);
	printk(KERN_INFO "[dma_simple_write] Vrednost PRE na adresi %p iznosi %#010x\n", tx_vir_buffer, *((unsigned int *)tx_vir_buffer));
	
	//TxBufferPtr = 0;		//test

	transaction_over0 = 1;
	iowrite32(MM2S_DMACR_val, base_address + MM2S_DMACR_REG);
	iowrite32((u32)TxBufferPtr, base_address + MM2S_SA_REG);
	iowrite32(pkt_len, base_address + MM2S_LENGTH_REG);
	while(transaction_over0 == 1);										// ovde puca kad se salje vise clanova, samo prvi put ide u interapt i vise ne
	printk(KERN_INFO "[dma_simple_write] Sent: %#010x \n", TxBufferPtr);
	printk(KERN_INFO "[dma_simple_write] Vrednost POSLE na adresi %p iznosi %#010x\n", tx_vir_buffer, *((unsigned int *)tx_vir_buffer));
	printk(KERN_INFO "[dma_simple_write] Successfully wrote in DMA \n");
	*tx_vir_buffer = fpu_array[cntrIn++];
	//dma_simple_write(tx_phy_buffer, MAX_PKT_LEN, dma_p->base_addr);
	dma_simple_read(rx_phy_buffer, pkt_len, dma_p->base_addr);				
    return 0;
	
}

unsigned int dma_simple_read(dma_addr_t RxBufferPtr, unsigned int pkt_len, void __iomem *base_address) {

	u32 S2MM_DMACR_value;
	u32 enInterrupt = 0;

	S2MM_DMACR_value = ioread32(base_address + S2MM_DMACR_REG);

	enInterrupt = S2MM_DMACR_value | IOC_IRQ_EN | ERR_IRQ_EN;
	iowrite32(enInterrupt, base_address + S2MM_DMACR_REG);

	S2MM_DMACR_value = ioread32(base_address + S2MM_DMACR_REG);
	S2MM_DMACR_value |= DMACR_RUN_STOP;

	printk(KERN_INFO "[dma_simple_read] Pre: %#010x \n", RxBufferPtr);
	printk(KERN_INFO "[dma_simple_read] Vrednost PRE na adresi %p iznosi %#010x\n", rx_vir_buffer, *((unsigned int *)rx_vir_buffer));

	// RxBufferPtr = 0;	//test

	transaction_over1 = 1;
	iowrite32(S2MM_DMACR_value, base_address + S2MM_DMACR_REG);
	iowrite32((u32)RxBufferPtr, base_address + S2MM_DA_REG);
	iowrite32(pkt_len, base_address + S2MM_LENGTH_REG);
	while(transaction_over1 == 1);

	printk(KERN_INFO "[dma_simple_read] Received: %#010x \n", RxBufferPtr);
	printk(KERN_INFO "[dma_simple_read] Vrednost POSLE na adresi %p iznosi %#010x\n", rx_vir_buffer, *((unsigned int *)rx_vir_buffer));	
	printk(KERN_INFO "[dma_simple_read] Successfully read from DMA \n");

	return 0;
}

static irqreturn_t dma_MM2S_isr(int irq, void* dev_id) {

	unsigned int IrqStatus;  
	IrqStatus = ioread32(dma_p->base_addr + MM2S_STATUS_REG);
	iowrite32(IrqStatus | 0x00007000, dma_p->base_addr + MM2S_STATUS_REG);
	printk(KERN_INFO "[dma_isr] Finished DMA MM2S transaction!\n");
	transaction_over0 = 0;

	//iowrite32(DMACR_RESET, dma_p->base_addr + MM2S_DMACR_REG);
	return IRQ_HANDLED;
}

static irqreturn_t dma_S2MM_isr(int irq, void* dev_id){

	unsigned int IrqStatus;  
	IrqStatus = ioread32(dma_p->base_addr + S2MM_STATUS_REG);
	iowrite32(IrqStatus | 0x00007000, dma_p->base_addr + S2MM_STATUS_REG);
	printk(KERN_INFO "[dma_isr] Finished DMA S2MM transaction!\n");
	fpu_array[posOut] = *rx_vir_buffer;
	printk(KERN_INFO "[dma_isr] RESULT %d: %#x\n", posOut, fpu_array[posOut]);
	posOut++;
	transaction_over1 = 0;
	
	iowrite32(DMACR_RESET, dma_p->base_addr + S2MM_DMACR_REG);		//ovo ponovo gavranise interapt, da moze ponovo da ulazi u ISRove
	return IRQ_HANDLED;
}