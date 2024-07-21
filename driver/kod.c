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
MODULE_DESCRIPTION("FPU Exp Driver");
MODULE_LICENSE("Dual BSD/GPL");

#define      DRIVER_NAME     "fpu_driver" 
#define      BUFF_SIZE 	    200
#define      ARR_SIZE  	5


//** DMA defines **//
#define MAX_PKT_LEN				4
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
unsigned int dma_simple_read(dma_addr_t TxBufferPtr, unsigned int pkt_len, void __iomem *base_address);

//** Struct Declarations **//

struct fpu_info {
	unsigned long mem_start;
	unsigned long mem_end;
	void __iomem *base_addr;
	int irq_num;
    /*/chat
    struct tasklet_struct dma_tasklet;
    dma_addr_t tx_phy_buffer;
    dma_addr_t rx_phy_buffer;
    void *tx_vir_buffer;
    void *rx_vir_buffer;
    *////gpt
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
	{ .compatible = "xlnx,axi-dma-0", },
	{ /* end of list */ },
};

MODULE_DEVICE_TABLE(of, fpu_of_match);

static struct platform_driver fpu_driver = {
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
u32 izlazni_niz[ARR_SIZE];
u32 ulazni_niz[ARR_SIZE * 2];

//** Init & Exit Functions **//     /* VEZBA 5*/

/*This function is called when module is loadaed in kernel */       
static int __init fpu_init(void) {

	int ret = 0;
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
	my_device = device_create(my_class, NULL, MKDEV(MAJOR(my_dev_id), 0), NULL, "fpuult");
	if(my_device == NULL) {
		goto fail_1;
	}
	printk(KERN_INFO "[fpu_init] Device fpuult created\n");
	
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
	tx_vir_buffer = dma_alloc_coherent(my_device, MAX_PKT_LEN, &tx_phy_buffer, GFP_DMA | GFP_KERNEL);
	printk(KERN_INFO "[fpu_init] Virtual and physical addresses coherent starting at %#x and ending at %#x\n", tx_phy_buffer, tx_phy_buffer+(uint)(MAX_PKT_LEN));
	if(!tx_vir_buffer) {
		printk(KERN_ALERT "[fpu_init] Could not allocate dma_alloc_coherent");
		goto fail_3;
	}
	else {
		printk("[fpu_init] Successfully allocated memory for transaction buffer\n");
	}
	*tx_vir_buffer = 0;
	printk(KERN_INFO "[fpu_init] Memory reset.\n");

	return platform_driver_register(&fpu_driver);

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
	dma_free_coherent(my_device, MAX_PKT_LEN, &tx_phy_buffer, GFP_DMA | GFP_KERNEL);
    platform_driver_unregister(&fpu_driver); //ove funckije nema u kodu sa 5ih vezbi, ali je pozivamo kako bi se "bezbednije" uklonio driver
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

	r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!r_mem){
	    printk(KERN_ALERT "[fpu_probe] Failed to get reg resource.\n");
	    return -ENODEV;
	}
	printk(KERN_ALERT "[fpu_probe] Probing dma_p\n");

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

	dma_p->irq_num = platform_get_irq(pdev, 0);
	
    if(!dma_p->irq_num) {
		printk(KERN_ERR "[fpu_probe] Could not get IRQ resource for dma\n");
		rc = -ENODEV;
		goto error03;
	}

	if (request_irq(dma_p->irq_num, dma_MM2S_isr, 0, "dma_device", dma_p)) {
		printk(KERN_ERR "[fpu_probe] Could not register M2SS IRQ %d\n", dma_p->irq_num);
		return -EIO;
		goto error03;
	}

	else {
		printk(KERN_INFO "[fpu_probe] Registered M2SS IRQ %d\n", dma_p->irq_num);
	}

    if (request_irq(dma_p->irq_num, dma_S2MM_isr, 0, "dma_device", dma_p)) {
		printk(KERN_ERR "[fpu_probe] Could not register S2MM IRQ %d\n", dma_p->irq_num);
		return -EIO;
		goto error03;
	}

	else {
		printk(KERN_INFO "[fpu_probe] Registered S2MM IRQ %d\n", dma_p->irq_num);
	}

	enable_irq(dma_p->irq_num);
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
	free_irq(dma_p->irq_num, dma_p);
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

ssize_t fpu_read(struct file *pfile, char __user *buf, size_t length, loff_t *offset) {		

	char buff[BUFF_SIZE];
	int ret = 0; 

	if(endRead) {
		endRead = 0;
		return 0;
	}
	if(posOut > 0) {
		if(cntrOut < posOut) {
			length = scnprintf(buff, BUFF_SIZE, "		RES %d: %#x\n", (cntrOut + 1), izlazni_niz[cntrOut]);
			ret = copy_to_user(buf, buff, length);
			if(ret) {
				printk(KERN_WARNING "[fpu_read] Copy to user failed\n");
				return -EFAULT;
			}
			cntrOut++;
		}
		if(cntrOut == posOut) {
			endRead = 1;
			cntrOut = 0;
			posOut = 0;
			posIn = 0;
			cntrIn = 0;
			cntr = 0;
		}
	}	
	else {
		printk(KERN_INFO "[fpu_read] Driver is empty\n");
		return -EFAULT;
	}
	printk(KERN_INFO "[fpu_read] Succesfully read driver\n");
	return length;
}

ssize_t fpu_write(struct file *pfile, const char __user *buf, size_t length, loff_t *offset) {	

	char buff[length + 1];
	int brojac = 1;
	int flag = 0;
	int pomeraj = 0;
	int ret;
    int i = 0;
	char str1[50];
	char str2[50];
	u32 tmp1, tmp2;
	ret = copy_from_user(buff, buf, length);
    	if (ret) {
       		 printk(KERN_WARNING "[fpu_write] copy from user failed\n");
       		 return -EFAULT;
   	}

	buff[length] = '\0';
	for(i = 0; buff[i] != '\0'; i++) {
		if(buff[i] == ';') {
			brojac++;
		}
	}

	if(posIn >= (ARR_SIZE*2 - 1)) {
		cntr = 1;
		printk(KERN_WARNING "[fpu_write] Driver is already full\n");
		goto label1;
	}
	while(brojac != 1) {
		if(brojac > (ARR_SIZE + 1)) {
			printk(KERN_WARNING "[fpu_write] Too much requests for multiplication\n");
			flag = 1;
			break;
		}
		if(posIn < (ARR_SIZE*2-1)) {
			ret = sscanf(buff + pomeraj, "%50[^,], %50[^;];", str1, str2);
			if(ret != 2) {
				printk(KERN_WARNING "[fpu_write] Parsing failed\n");
       				return -EFAULT;
			}
			sscanf(str1, "%x", &tmp1);
			ulazni_niz[posIn] = tmp1;
			printk(KERN_INFO "[fpu_write] BROJ %d: %#x\n", (posIn + 1), ulazni_niz[posIn]);	
			posIn++;
			sscanf(str2, "%x", &tmp2);
			ulazni_niz[posIn] = tmp2;
			printk(KERN_INFO "[fpu_write] BROJ %d: %#x\n", (posIn + 1), ulazni_niz[posIn]);
			posIn++; 
			pomeraj = pomeraj +  strlen(str1) + strlen(str2) + 3;
			--brojac;
		}
		else {
			printk(KERN_WARNING "[fpu_write] Driver is full\n");
			break;
		}
	}

	printk(KERN_INFO "[fpu_write] Succesfully wrote in driver\n");
	label1:
		if(cntr == 0  && flag != 1) {
			cntr++;
			*tx_vir_buffer = ulazni_niz[cntrIn++];	
			dma_simple_write(tx_phy_buffer, MAX_PKT_LEN, dma_p->base_addr);
		}
		return length;
}

//** Mmap Function **//  /* VEZBA 12*/

static int fpu_mmap(struct file *f, struct vm_area_struct *vma_s) {

	int ret = 0;
	long length = vma_s->vm_end - vma_s->vm_start;
	printk(KERN_INFO "[fpu_dma_mmap] DMA TX Buffer is being memory mapped\n");
	ret = dma_mmap_coherent(my_device, vma_s, tx_vir_buffer, tx_phy_buffer, length);
	if(ret < 0) {
		printk(KERN_ERR "[fpu_dma_mmap] Memory map DMA failed\n");
		return ret;
	}
	return 0;
}

//** DMA Functions **//  /*VEZBA 12*/

int dma_init(void __iomem *base_address) {

	u32 MM2S_DMACR_val = 0;
	u32 enInterrupt = 0;
	iowrite32(0x0, base_address + MM2S_DMACR_REG);
	iowrite32(DMACR_RESET, base_address + MM2S_DMACR_REG);
	MM2S_DMACR_val = ioread32(base_address + MM2S_DMACR_REG);
	enInterrupt = MM2S_DMACR_val | IOC_IRQ_EN | ERR_IRQ_EN;
	iowrite32(enInterrupt, base_address + MM2S_DMACR_REG);	
	printk(KERN_INFO "[dma_init] Successfully initialized DMA \n");
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
	transaction_over0 = 1;
	iowrite32(MM2S_DMACR_val, base_address + MM2S_DMACR_REG);
	iowrite32((u32)TxBufferPtr, base_address + MM2S_SA_REG);
	iowrite32(pkt_len, base_address + MM2S_LENGTH_REG);
	while(transaction_over0 == 1);
	printk(KERN_INFO "[dma_simple_write] Successfully wrote in DMA \n");
	*tx_vir_buffer = ulazni_niz[cntrIn++];
	dma_simple_write(tx_phy_buffer, MAX_PKT_LEN, dma_p->base_addr);		
    return 0;
	
}

unsigned int dma_simple_read(dma_addr_t RxBufferPtr, unsigned int pkt_len, void __iomem *base_address) {

	u32 S2MM_DMACR_value;
	S2MM_DMACR_value = ioread32(base_address + S2MM_DMACR_REG);
	S2MM_DMACR_value |= DMACR_RUN_STOP; 	
	transaction_over1 = 1;
	iowrite32(S2MM_DMACR_value, base_address + S2MM_DMACR_REG);
	iowrite32((u32)RxBufferPtr, base_address + S2MM_DA_REG);
	iowrite32(pkt_len, base_address + S2MM_LENGTH_REG);
	while(transaction_over1 == 1);
	if(cntrIn < (posIn - 1)) {
		*tx_vir_buffer = ulazni_niz[cntrIn++];
		dma_simple_write(rx_phy_buffer, MAX_PKT_LEN, dma_p->base_addr);
	} 
	else {
		cntr = 0;
	}
	printk(KERN_INFO "[dma_simple_read] Successfully read from DMA \n");
	return 0;
}

static irqreturn_t dma_MM2S_isr(int irq, void* dev_id) {

	unsigned int IrqStatus;  
	IrqStatus = ioread32(dma_p->base_addr + MM2S_STATUS_REG);
	iowrite32(IrqStatus | 0x00007000, dma_p->base_addr + MM2S_STATUS_REG);
	printk(KERN_INFO "[dma_isr] Finished DMA MM2S transaction!\n");
	transaction_over0 = 0;
	return IRQ_HANDLED;
}

static irqreturn_t dma_S2MM_isr(int irq, void* dev_id){

	unsigned int IrqStatus;  
	IrqStatus = ioread32(dma_p->base_addr + S2MM_STATUS_REG);
	iowrite32(IrqStatus | 0x00007000, dma_p->base_addr + S2MM_STATUS_REG);
	printk(KERN_INFO "[dma_isr] Finished DMA S2MM transaction!\n");
	izlazni_niz[posOut] = *tx_vir_buffer;
	printk(KERN_INFO "[fpu_write] RESULT %d: %#x\n", (posOut + 1), izlazni_niz[posOut]);
	posOut++;
	transaction_over1 = 0;
	return IRQ_HANDLED;
}