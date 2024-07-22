#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/completion.h>

#define MAX_ARRAY_SIZE 256
#define BUFF_SIZE 4096

static dev_t dev;
static struct cdev cdev;
static struct class *cl;

static u32 *fpu_array = NULL;
static int arr_size = 0;
static int initialized = 0;

static dma_addr_t dma_tx_addr;
static dma_addr_t dma_rx_addr;
static u32 *dma_tx_buffer;
static u32 *dma_rx_buffer;

static struct dma_chan *dma_tx_chan;
static struct dma_chan *dma_rx_chan;
static struct completion dma_tx_complete;
static struct completion dma_rx_complete;

static void dma_tx_callback(void *completion)
{
    complete(completion);
}

static void dma_rx_callback(void *completion)
{
    complete(completion);
}

ssize_t fpu_write(struct file *pfile, const char __user *buf, size_t length, loff_t *offset)
{
    char kbuf[32];
    int pos;
    u32 val;
    int ret;

    if (length > sizeof(kbuf) - 1)
        return -EINVAL;

    if (copy_from_user(kbuf, buf, length))
        return -EFAULT;

    kbuf[length] = '\0';

    if (sscanf(kbuf, "N=%d", &arr_size) == 1) {
        if (arr_size < 0 || arr_size > MAX_ARRAY_SIZE)
            return -EINVAL;

        if (fpu_array)
            kfree(fpu_array);

        fpu_array = kzalloc(arr_size * sizeof(u32), GFP_KERNEL);
        if (!fpu_array)
            return -ENOMEM;

        initialized = 1;
        return length;
    } else if (sscanf(kbuf, "%d=0x%x", &pos, &val) == 2) {
        if (pos < 0 || pos >= arr_size)
            return -EINVAL;

        if (!initialized)
            return -EINVAL;

        fpu_array[pos] = val;
        return length;
    }

    return -EINVAL;
}

ssize_t fpu_read(struct file *pfile, char __user *buf, size_t length, loff_t *offset)
{
    static int finished = 0;
    char *kernel_buf;
    int i;
    int ret;
    size_t len = 0;

    if (!initialized) {
        printk(KERN_WARNING "[fpu_read] Array not initialized\n");
        return 0;
    }

    if (finished) {
        finished = 0;
        return 0;
    }

    kernel_buf = kmalloc(BUFF_SIZE, GFP_KERNEL);
    if (!kernel_buf) {
        printk(KERN_ERR "[fpu_read] Memory allocation failed\n");
        return -ENOMEM;
    }

    memcpy(dma_tx_buffer, fpu_array, arr_size * sizeof(u32));

    dma_async_issue_pending(dma_tx_chan);
    wait_for_completion(&dma_tx_complete);

    dma_async_issue_pending(dma_rx_chan);
    wait_for_completion(&dma_rx_complete);

    for (i = 0; i < arr_size; i++) {
        len += snprintf(kernel_buf + len, BUFF_SIZE - len, "0x%08x", dma_rx_buffer[i]);
        if (i < arr_size - 1) {
            len += snprintf(kernel_buf + len, BUFF_SIZE - len, ", ");
        }
        if (len >= BUFF_SIZE) {
            printk(KERN_WARNING "[fpu_read] Buffer size exceeded\n");
            kfree(kernel_buf);
            return -EFAULT;
        }
    }

    ret = copy_to_user(buf, kernel_buf, len);
    if (ret) {
        printk(KERN_WARNING "[fpu_read] Copy to user failed\n");
        kfree(kernel_buf);
        return -EFAULT;
    }

    kfree(kernel_buf);
    finished = 1;

    return len;
}

static int __init fpu_init(void)
{
    int ret;
    struct device *dev_ret;
    dma_cap_mask_t mask;

    if ((ret = alloc_chrdev_region(&dev, 0, 1, "fpu_exp")) < 0) {
        return ret;
    }
    cdev_init(&cdev, &fpu_fops);
    if ((ret = cdev_add(&cdev, dev, 1)) < 0) {
        unregister_chrdev_region(dev, 1);
        return ret;
    }
    if (IS_ERR(cl = class_create(THIS_MODULE, "char"))) {
        cdev_del(&cdev);
        unregister_chrdev_region(dev, 1);
        return PTR_ERR(cl);
    }
    if (IS_ERR(dev_ret = device_create(cl, NULL, dev, NULL, "fpu_exp"))) {
        class_destroy(cl);
        cdev_del(&cdev);
        unregister_chrdev_region(dev, 1);
        return PTR_ERR(dev_ret);
    }

    dma_tx_buffer = dma_alloc_coherent(NULL, MAX_ARRAY_SIZE * sizeof(u32), &dma_tx_addr, GFP_KERNEL);
    if (!dma_tx_buffer) {
        ret = -ENOMEM;
        goto err_alloc_tx;
    }

    dma_rx_buffer = dma_alloc_coherent(NULL, MAX_ARRAY_SIZE * sizeof(u32), &dma_rx_addr, GFP_KERNEL);
    if (!dma_rx_buffer) {
        ret = -ENOMEM;
        goto err_alloc_rx;
    }

    dma_cap_zero(mask);
    dma_cap_set(DMA_MEMCPY, mask);

    dma_tx_chan = dma_request_channel(mask, NULL, NULL);
    if (!dma_tx_chan) {
        ret = -ENODEV;
        goto err_req_tx;
    }

    dma_rx_chan = dma_request_channel(mask, NULL, NULL);
    if (!dma_rx_chan) {
        ret = -ENODEV;
        goto err_req_rx;
    }

    init_completion(&dma_tx_complete);
    init_completion(&dma_rx_complete);

    printk(KERN_INFO "fpu_exp driver initialized\n");
    return 0;

err_req_rx:
    dma_release_channel(dma_tx_chan);
err_req_tx:
    dma_free_coherent(NULL, MAX_ARRAY_SIZE * sizeof(u32), dma_tx_buffer, dma_tx_addr);
err_alloc_rx:
    dma_free_coherent(NULL, MAX_ARRAY_SIZE * sizeof(u32), dma_rx_buffer, dma_rx_addr);
err_alloc_tx:
    device_destroy(cl, dev);
    class_destroy(cl);
    cdev_del(&cdev);
    unregister_chrdev_region(dev, 1);
    return ret;
}

static void __exit fpu_exit(void)
{
    dma_release_channel(dma_tx_chan);
    dma_release_channel(dma_rx_chan);
    dma_free_coherent(NULL, MAX_ARRAY_SIZE * sizeof(u32), dma_tx_buffer, dma_tx_addr);
    dma_free_coherent(NULL, MAX_ARRAY_SIZE * sizeof(u32), dma_rx_buffer, dma_rx_addr);
    device_destroy(cl, dev);
    class_destroy(cl);
    cdev_del(&cdev);
    unregister_chrdev_region(dev, 1);

    if (fpu_array) {
        kfree(fpu_array);
    }

    printk(KERN_INFO "fpu_exp driver exited\n");
}

module_init(fpu_init);
module_exit(fpu_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("FPU Experiment Driver");
