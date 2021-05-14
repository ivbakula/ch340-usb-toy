#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kref.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/sched.h>

#define MAX_TRANSFER		(PAGE_SIZE - 512)
#define MAX_WRITES		16	/* max number of concurrent writes */
#define CH340_MINOR_BASE	0

/* ioctls: */
#define CH340_SET_BAUDRATE	1
#define CH340_GET_VENDORDATA	2

#define VENDOR_WRITE_TYPE		0x40
#define VENDOR_READ_TYPE		0xC0

#define VENDOR_READ				0x95
#define VENDOR_WRITE			0x9A
#define VENDOR_SERIAL_INIT		0xA1
#define VENDOR_MODEM_OUT		0xA4
#define VENDOR_VERSION			0x5F

//For CMD 0xA4
#define UART_CTS		0x01
#define UART_DSR		0x02
#define UART_RING		0x04
#define UART_DCD		0x08
#define CONTROL_OUT		0x10
#define CONTROL_DTR		0x20
#define	CONTROL_RTS		0x40

//Uart state
#define UART_STATE			0x00
#define UART_OVERRUN_ERROR	0x01
#define UART_BREAK_ERROR	//no define
#define UART_PARITY_ERROR	0x02
#define UART_FRAME_ERROR	0x06
#define UART_RECV_ERROR		0x02
#define UART_STATE_TRANSIENT_MASK	0x07


static int ch340_open(struct inode *inode, struct file *file);
static int ch340_release(struct inode *inode, struct file *file);
static int ch340_probe(struct usb_interface *iface, const struct usb_device_id *id);
static void ch340_disconnect(struct usb_interface *iface);
static __poll_t ch340_poll(struct file *file, struct poll_table_struct *wait);
static ssize_t ch340_read(struct file *file, char __user *buffer, size_t count,
			  loff_t *ppos);
static ssize_t ch340_write(struct file *file, const char __user *buff,
			   size_t count, loff_t *ppos);
static long ch340_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

struct usb_ch340 {
	/* usb structures */
	struct usb_device	*udev;
	struct usb_interface	*iface;
	struct usb_anchor	submitted;
	
	/* locking */
	struct semaphore	limit_sem;
	struct mutex		io_mutex;
	spinlock_t		err_lock;
	struct task_struct	*wait_disconnect;
	wait_queue_head_t	bulk_in_wait;
	wait_queue_head_t	outq_poll;

	/* reference counting */
	struct kref		kref;

	/* IO data */
	struct urb		*bulk_in_urb;
	__u8			bulk_out_endpointAddr;
	__u8			bulk_in_endpointAddr;
	size_t			bulk_in_size;
	unsigned char		*bulk_in_buffer;
	size_t			bulk_in_filled;
	size_t			bulk_in_copied;
	int			errors;
	unsigned long		disconnected:1;
	bool			data_to_read;		
	bool			staging_urb;
	bool			ongoing_io;
	bool			poll_in_queue;
};

static struct file_operations ch340_fops = {
	.open = ch340_open,
	.release =		ch340_release,
	.poll =			ch340_poll,
	.read =			ch340_read,
	.write =		ch340_write,
	.unlocked_ioctl =	ch340_ioctl, 
};

static struct usb_class_driver ch340_class = {
	.name =		"usbTTL%d",
	.fops =		&ch340_fops,
	.minor_base =	CH340_MINOR_BASE,	
};

static struct usb_device_id id_table[] = {
	{USB_DEVICE(0x1a86, 0x7523)},
	{ },
};
MODULE_DEVICE_TABLE(usb, id_table);

static struct usb_driver ch340_driver = {
	.name		= "ch340",
	.probe		= ch340_probe,
	.disconnect	= ch340_disconnect,
	.id_table	= id_table,
};

static void ch340_delete(struct kref *kref)
{
	struct usb_ch340 *dev = container_of(kref, struct usb_ch340, kref);

	usb_free_urb(dev->bulk_in_urb);

	/* decrement reference counts of usb_{device,interface} */
	usb_put_intf(dev->iface);
	usb_put_dev(dev->udev);

	kfree(dev->bulk_in_buffer);
	kfree(dev);
}

static int ch340_open(struct inode *inode, struct file *file)
{
	struct usb_ch340 *dev;
	struct usb_interface *interface;
	int subminor;
	int retval = 0;

	subminor = iminor(inode);
	interface = usb_find_interface(&ch340_driver, subminor);
	if (!interface) {
		pr_err("%s - error, can't find device for minor %d\n",
			__func__, subminor);
		retval = -ENODEV;
		goto exit;
	}

	dev = usb_get_intfdata(interface);
	if (!dev) {
		printk(KERN_ALERT "usb_get_intfdata(interface) == NULL\n");
		retval = -ENODEV;
		goto exit;
	}

	kref_get(&dev->kref);
	file->private_data = dev;
	if(!file->private_data)
	    printk(KERN_ALERT "file->private_data = NULL\n");
exit:
	return retval;
}

static int ch340_release(struct inode *inode, struct file *file)
{
	struct usb_ch340 *dev;
	
	dev = file->private_data;
	if (dev == NULL) 
		return -ENODEV;	

	/* decrease reference counts */
	kref_put(&dev->kref, ch340_delete);
	return 0;
}

static int ch340_get_baudrate(unsigned long baud_rate, unsigned char *factor, 
							unsigned char *div)
{
	unsigned char a;
	unsigned char b;
	unsigned long c;

	switch ( baud_rate ) {
	case 921600: 
		a = 0xf3; 
		b = 7; 
		break; 
	case 307200:
		a = 0xd9; 
		b = 7; 
		break; 
	default: 
		if ( baud_rate > 6000000/255 ) { 
			b = 3;
			c = 6000000;
		} else if ( baud_rate > 750000/255 ) {  
			b = 2;
			c = 750000;
		} else if (baud_rate > 93750/255) { 
			b = 1;
			c = 93750;
		} else {
			b = 0;
			c = 11719;
		}
		a = (unsigned char)(c / baud_rate);
		if (a == 0 || a == 0xFF) return -EINVAL;
		if ((c / a - baud_rate) > (baud_rate - c / (a + 1))) 
			a ++;
		a = 256 - a;
		break;
	}
	*factor = a;
	*div= b;
	return 0;

}
static int ch340_ctrl_write( __u8 request, __u16 value, __u16 index,
		struct usb_device *udev, unsigned char *buf, __u16 len )
{
	int retval;
	retval = usb_control_msg( udev, 
			usb_sndctrlpipe(udev, 0),
			request,
			VENDOR_WRITE_TYPE,
			value, index, buf, len, 1000 );

	return retval;
}


static int ch340_ctrl_read( __u8 request, __u16 value, __u16 index,
		struct usb_device *udev, unsigned char *buf, __u16 len )
{
	int retval;

	retval = usb_control_msg( udev, usb_rcvctrlpipe(udev, 0), 
			request, VENDOR_READ_TYPE, value, index, buf, len, 1000 );
	return retval;
}


static long ch340_ioctl(struct file *file, unsigned int cmd, 
						unsigned long arg) 
{
	struct usb_ch340 *dev = file->private_data;
	unsigned short value = 0;
	unsigned short index = 0;
	unsigned long baudrate;
	unsigned char div;
	unsigned char factor;


	switch(cmd) {
	case CH340_SET_BAUDRATE:
		/* 8-bit bytes, 1 stop bit, no parity, */
		baudrate = arg;
		ch340_get_baudrate(baudrate, &factor, &div);

		/* enable 8-bit bytes and SFR_UART Tx and Rx */
		value |= 0x03 | 0xc0;		
		value = value << 8;

		/* enable SFR_UART control register and timer */
		value |= 0x9c ;
		index |= 0x80 | div;
		index |= factor << 8;
		ch340_ctrl_write(VENDOR_SERIAL_INIT, value, index, dev->udev, NULL, 0);
		break;

	case CH340_GET_VENDORDATA:
		break;
	}

	return 0;
}

/* read callback: called when there is data to read. Responsibility of 
 * this callback is to wake up any sleeping thread that might be waning
 * to read (blocking thread or polling thread) */
static void ch340_read_bulk_callback(struct urb *urb)
{
	struct usb_ch340 *dev; 
	unsigned long flags;

	dev = urb->context;

	spin_lock_irqsave(&dev->err_lock, flags);
	if(urb->status) {
		if (!(urb->status == -ENOENT ||
		    urb->status == -ECONNRESET ||
		    urb->status == -ESHUTDOWN))
			dev_err(&dev->iface->dev,
				"%s - nonzero write bulk status received: %d\n",
				__func__, urb->status);

		dev->errors = urb->status;

	} else {
		dev->data_to_read = true;
		dev->staging_urb = false;
		dev->bulk_in_filled = urb->actual_length;
	}
	dev->ongoing_io = false;
	spin_unlock_irqrestore(&dev->err_lock, flags);

	/* wake up polling thread (if there is one) */
	wake_up_interruptible(&dev->outq_poll);		

	/* wake up blocking thread (if there is one) */
	wake_up_interruptible(&dev->bulk_in_wait);
}

/* submit read bulk urb and wait */
static int ch340_stage_readurb(struct usb_ch340 *dev, size_t count)
{
	int retval;	
	usb_fill_bulk_urb(dev->bulk_in_urb, dev->udev, 
		usb_rcvbulkpipe(dev->udev, dev->bulk_in_endpointAddr),
		dev->bulk_in_buffer, min(dev->bulk_in_size, count), 
		ch340_read_bulk_callback, dev);
	
	spin_lock_irq(&dev->err_lock);
	dev->staging_urb = true;
	dev->ongoing_io = true;
	dev->data_to_read = false;
	spin_unlock_irq(&dev->err_lock);

	dev->bulk_in_filled = 0;
	dev->bulk_in_copied = 0;

	retval = usb_submit_urb(dev->bulk_in_urb, GFP_KERNEL);
	if (retval < 0) {
		dev_err(&dev->iface->dev,
			"%s - failed submitting read urb, error %d\n",
			__func__, retval);
		spin_lock_irq(&dev->err_lock);
		dev->staging_urb = false;
		spin_unlock_irq(&dev->err_lock);
	}

	return retval;
}

static __poll_t ch340_poll(struct file *file, struct poll_table_struct *wait)
{
	struct usb_ch340 *dev = file->private_data;
	unsigned long irqflags;
	bool status;
	__poll_t mask = 0;

	poll_wait(file, &dev->outq_poll, wait);

	mutex_lock(&dev->io_mutex);
	spin_lock_irqsave(&dev->err_lock, irqflags);
	if(dev->disconnected) {
		spin_unlock_irqrestore(&dev->err_lock, irqflags);
		mask = POLLHUP;
		goto done;
	}

	if(dev->data_to_read && !dev->ongoing_io) 
		mask = POLLIN | POLLRDNORM;

	status = dev->staging_urb;
	spin_unlock_irqrestore(&dev->err_lock, irqflags);

	if(!status && !mask)
	    ch340_stage_readurb(dev, dev->bulk_in_size);

done:
	mutex_unlock(&dev->io_mutex);

	return mask;
}

static ssize_t ch340_read(struct file *file, char __user *buffer, size_t count,
			  loff_t *ppos)
{
	struct usb_ch340 *dev;
	int rv;
	bool ongoing_io;

	dev = file->private_data;

	/* 
	 * aquire mutex. Use interruptible mutex so waiting process
	 * can be signalled. Why do we want interruptible mutex in 
	 * this particular case? Because there is no guarantee that this lock 
	 * will be released in reasonable amount of time, so we want to 
	 * be able to kill processes (or signal it somehow other)
	 *
	 */
	rv = mutex_lock_interruptible(&dev->io_mutex);
	if(rv < 0) {
		/* interrupted */
		return rv;
	}

	if(dev->disconnected) {
		rv = -ENODEV;
		goto exit;
	}

retry:
	spin_lock_irq(&dev->err_lock);
	ongoing_io = dev->ongoing_io;
	spin_unlock_irq(&dev->err_lock);

	if(ongoing_io) {
		/* nonblocking IO shall not wait. */
		if(file->f_flags & O_NONBLOCK) {
			rv = -EAGAIN;
			goto exit;
		}
		/*
		 * IO may take forever, hence wait in an interruptible state
		 */
		rv = wait_event_interruptible(dev->bulk_in_wait, (!dev->ongoing_io));
		if(rv < 0) 
			/* signal recieved */
			goto exit;
	}

	rv = dev -> errors;
	if(rv < 0) {
		dev->errors = 0;
		rv = (rv == -EPIPE) ? rv : -EIO;
		goto exit;
	}

	/* no data in the buffer */
	if(!dev->bulk_in_filled) {
		rv = ch340_stage_readurb(dev, count);
		if(rv < 0)
			goto exit;
		else
			goto retry;
	} else {
		size_t available = dev -> bulk_in_filled - dev->bulk_in_copied;
		size_t chunk = min(available, count);

		if(!available) {
			rv = ch340_stage_readurb(dev, count);
			if(rv < 0)
				goto exit;
			else
				goto retry;
		}
		if(copy_to_user(buffer,
				dev->bulk_in_buffer + dev->bulk_in_copied,
				chunk))
			rv = -EFAULT;
		else
			rv = chunk;

		dev->bulk_in_copied += chunk;

		if(available < count)
			ch340_stage_readurb(dev, count-chunk);
	}
exit:
	mutex_unlock(&dev->io_mutex);
	return rv;
}
static void ch340_write_callback(struct urb *urb)
{
	struct usb_ch340 *dev;
	dev = urb->context;

	if(urb->status) {
		if(!(urb->status == -ENOENT ||
		     urb->status == -ECONNRESET ||
		     urb->status == -ESHUTDOWN)) {
			printk(KERN_ALERT "non-zero write bulk status received: %d\n", urb->status);
		}
		printk(KERN_ALERT "some sort of error in bulk write: %d\n", urb->status);
	}

	usb_free_coherent(urb->dev, urb->transfer_buffer_length,
				urb->transfer_buffer, urb->transfer_dma);
	up(&dev->limit_sem);
}


static ssize_t ch340_write(struct file *file, const char __user *buff,
			   size_t count, loff_t *ppos)
{
	struct usb_ch340 *dev;
	int retval = 0;
	struct urb *urb = NULL;	
	char *buf = NULL;
	size_t writesize = min(count, (size_t)MAX_TRANSFER);

	dev = file->private_data;
	if (!dev) {
	    printk(KERN_ALERT "dev = NULL\n");
	    return -ENODEV;
	}
	if(count == 0) 
		goto exit;
	if(!(file->f_flags & O_NONBLOCK)) {
		if(down_interruptible(&dev->limit_sem)) {
			retval = -ERESTARTSYS;
			goto exit;
		} 
	} else {
		if(down_trylock(&dev->limit_sem)) {
			retval = -EAGAIN;
			goto exit;
		}
	}
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if(!urb) {
		retval = -ENOMEM;
		goto error;
	}

	buf = usb_alloc_coherent(dev->udev, writesize, GFP_KERNEL,
				&urb->transfer_dma);


	if(!buf) {
		retval = -ENOMEM;
		goto error;
	}	

	if(copy_from_user(buf, buff, writesize)) {
		retval = -EFAULT;
		goto error;
	}

	mutex_lock(&dev->io_mutex);
	if(dev->disconnected) {
		mutex_unlock(&dev->io_mutex);
		retval = -ENODEV;
		goto error;
	}

	usb_fill_bulk_urb(urb, dev->udev,
			  usb_sndbulkpipe(dev->udev, dev->bulk_out_endpointAddr),
			  buf, writesize, ch340_write_callback, dev);
	urb -> transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		
	/* send data out to the bulk port */
	retval = usb_submit_urb(urb, GFP_KERNEL);
	mutex_unlock(&dev->io_mutex);

	if(retval) {
		printk(KERN_ALERT "failed to write urb, error: %d\n", retval);
	}

	/* 
	 * release our reference to this urb, the USB core will eventually free
	 * it entirely
	 */
	usb_free_urb(urb);

	return writesize;

error:
	if(urb) {
		usb_free_coherent(dev->udev, writesize, buf, urb->transfer_dma);
		usb_free_urb(urb);
	}
	up(&dev->limit_sem);
exit:
	return retval;
}


static int ch340_probe(struct usb_interface *interface,
	const struct usb_device_id *id)
{
	struct usb_ch340 *dev;
	struct usb_endpoint_descriptor *bulk_in, *bulk_out;
	int retval;
	
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
	    return -ENOMEM;
	
	dev->udev = usb_get_dev(interface_to_usbdev(interface));

	/* increment the reference count of the usb interface structure.
	 * Each live reference to a interface must be refcounted */
	dev->iface = usb_get_intf(interface); 
	init_usb_anchor(&dev->submitted);

	/* initialize locking mechanisms */
	sema_init(&dev->limit_sem, MAX_WRITES);
	mutex_init(&dev->io_mutex);
	spin_lock_init(&dev->err_lock);
	init_waitqueue_head(&dev->bulk_in_wait);
	init_waitqueue_head(&dev->outq_poll);
	kref_init(&dev->kref);

	/* Allocate input URB */
	dev->bulk_in_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->bulk_in_urb) {
	    retval = -ENOMEM;
	    goto error;
	}

	/* Find endpoints */
	retval = usb_find_common_endpoints(interface->cur_altsetting,
			&bulk_in, &bulk_out, NULL, NULL);
	if (retval) {
		dev_err(&interface->dev,
			"Could not find both bulk-in and bulk-out endpoints\n");
		goto error;
	}
	dev->bulk_out_endpointAddr = bulk_out->bEndpointAddress;
	dev->bulk_in_endpointAddr = bulk_in->bEndpointAddress;
	dev->bulk_in_size = usb_endpoint_maxp(bulk_in); /* get endpoint's max packet size */
	dev->bulk_in_buffer = kmalloc(dev->bulk_in_size, GFP_KERNEL);
	if (!dev->bulk_in_buffer) {
		retval = -ENOMEM;
		goto error;
	}

	/* Save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);

	/* register the device now */
	retval = usb_register_dev(interface, &ch340_class);
	if (retval) {
		dev_err(&interface->dev,
			"Unable to get a minor number for this device.\n");
		usb_set_intfdata(interface, NULL);
		goto error;
	}
	
	dev_info(&interface->dev,
		"USB ch340 now attached to usbTTL%d\n", 
		interface->minor);
	
	return 0;

error:
	kref_put(&dev->kref, ch340_delete);
	return retval;
}

static void ch340_disconnect(struct usb_interface *interface)
{
	struct usb_ch340 *dev;
	int minor = interface->minor;
	dev = usb_get_intfdata(interface);

	usb_set_intfdata(interface, NULL);

	usb_deregister_dev(interface, &ch340_class);
	mutex_lock(&dev->io_mutex);
	dev->disconnected = 1;
	mutex_unlock(&dev->io_mutex);
	
	/* wake up poll thread if there is one */
	wake_up_interruptible(&dev->outq_poll);

	usb_kill_urb(dev->bulk_in_urb);
	usb_kill_anchored_urbs(&dev->submitted);

	kref_put(&dev->kref, ch340_delete);
	dev_info(&interface->dev, 
		"USB device usbTTL%d now disconnected.\n", minor);
}

module_usb_driver(ch340_driver);
MODULE_LICENSE("GPL");
