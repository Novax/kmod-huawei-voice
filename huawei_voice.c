/*
  USB Driver for GSM modems

  Copyright (C) 2005  Matthias Urlichs <smurf@smurf.noris.de>

  This driver is free software; you can redistribute it and/or modify
  it under the terms of Version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  Portions copied from the Keyspan driver by Hugh Blemings <hugh@blemings.org>

  History: see the git log.

  Work sponsored by: Sigos GmbH, Germany <info@sigos.de>

  This driver exists because the "normal" serial driver doesn't work too well
  with GSM modems. Issues:
  - data loss -- one single Receive URB is not nearly enough
  - nonstandard flow (Option devices) control
  - controlling the baud rate doesn't make sense

  This driver is named "huawei_voice" because the most common device it's
  used for is a PC-Card (with an internal OHCI-USB interface, behind
  which the GSM interface sits), made by Option Inc.

  Some of the "one port" devices actually exhibit multiple USB instances
  on the USB bus. This is not a bug, these ports are used for different
  device features.
*/

#define DRIVER_VERSION "v0.0.1"
#define DRIVER_AUTHOR "Anatoly Sidorov <novax_mail@inbox.ru>"
#define DRIVER_DESC "USB Driver for DIAG(voice) port Huawei 3G modems"

#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/errno.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>
#include "usb-wwan.h"

/* Function prototypes */
static int  huawei_voice_probe(struct usb_serial *serial,
			const struct usb_device_id *id);
static int huawei_voice_send_setup(struct usb_serial_port *port);
static void huawei_voice_instat_callback(struct urb *urb);
static int huawei_voice_write(struct tty_struct *tty, struct usb_serial_port *port,
                   const unsigned char *buf, int count);
static int huawei_voice_startup(struct usb_serial *serial);


/* Vendor and product IDs */

#define HUAWEI_VENDOR_ID			0x12D1
#define HUAWEI_PRODUCT_E600			0x1001

/* some devices interfaces need special handling due to a number of reasons */
enum huawei_voice_blacklist_reason {
		OPTION_BLACKLIST_NONE = 0,
		OPTION_BLACKLIST_SENDSETUP = 1,
		OPTION_BLACKLIST_RESERVED_IF = 2
};

struct huawei_voice_blacklist_info {
	const u32 infolen;	/* number of interface numbers on blacklist */
	const u8  *ifaceinfo;	/* pointer to the array holding the numbers */
	enum huawei_voice_blacklist_reason reason;
};

static const u8 four_g_w14_no_sendsetup[] = { 0, 1 };
static const struct huawei_voice_blacklist_info four_g_w14_blacklist = {
	.infolen = ARRAY_SIZE(four_g_w14_no_sendsetup),
	.ifaceinfo = four_g_w14_no_sendsetup,
	.reason = OPTION_BLACKLIST_SENDSETUP
};

static const struct usb_device_id huawei_voice_ids[] = {
	{ USB_DEVICE_AND_INTERFACE_INFO(HUAWEI_VENDOR_ID, HUAWEI_PRODUCT_E600, 0xff, 0xff, 0xff) },
	{ } /* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, huawei_voice_ids);

static struct usb_driver huawei_voice_driver = {
	.name       = "huawei_voice",
	.probe      = usb_serial_probe,
	.disconnect = usb_serial_disconnect,
#ifdef CONFIG_PM
	.suspend    = usb_serial_suspend,
	.resume     = usb_serial_resume,
	.supports_autosuspend =	1,
#endif
	.id_table   = huawei_voice_ids,
	.no_dynamic_id = 	1,
};

/* The card has three separate interfaces, which the serial driver
 * recognizes separately, thus num_port=1.
 */

static struct usb_serial_driver huawei_voice_1port_device = {
	.driver = {
		.owner =	THIS_MODULE,
		.name =		"huawei_voice1",
	},
	.description       = "Huawei 3G modem diag(voice) port",
	.usb_driver        = &huawei_voice_driver,
	.id_table          = huawei_voice_ids,
	.bulk_in_size      = 320,
	.bulk_out_size     = 80,
	.num_ports         = 1,
	.probe             = huawei_voice_probe,
	.open              = usb_wwan_open,
	.close             = usb_wwan_close,
	.dtr_rts           = usb_wwan_dtr_rts,
	.write             = huawei_voice_write,
	.write_room        = usb_wwan_write_room,
	.chars_in_buffer   = usb_wwan_chars_in_buffer,
	.set_termios       = usb_wwan_set_termios,
	.tiocmget          = usb_wwan_tiocmget,
	.tiocmset          = usb_wwan_tiocmset,
	.attach            = huawei_voice_startup,
	.disconnect        = usb_wwan_disconnect,
	.release           = usb_wwan_release,
	.read_int_callback = huawei_voice_instat_callback,
#ifdef CONFIG_PM
	.suspend           = usb_wwan_suspend,
	.resume            = usb_wwan_resume,
#endif
};

static int debug;

/* per port private data */

#define N_IN_URB 4
#define N_OUT_URB 4
#define IN_BUFLEN 4096
#define OUT_BUFLEN 4096

struct huawei_voice_port_private {
	/* Input endpoints and buffer for this port */
	struct urb *in_urbs[N_IN_URB];
	u8 *in_buffer[N_IN_URB];
	/* Output endpoints and buffer for this port */
	struct urb *out_urbs[N_OUT_URB+1];
	u8 *out_buffer[N_OUT_URB+1];

	unsigned long out_busy;		/* Bit vector of URBs in use */
	int opened;
	struct usb_anchor delayed;

	/* Settings for the port */
	int rts_state;	/* Handshaking pins (outputs) */
	int dtr_state;
	int cts_state;	/* Handshaking pins (inputs) */
	int dsr_state;
	int dcd_state;
	int ri_state;

	unsigned long tx_start_time[N_OUT_URB + 1];
};

/* Functions used by new usb-serial code. */
static int __init huawei_voice_init(void)
{
	int retval;
	retval = usb_serial_register(&huawei_voice_1port_device);
	if (retval)
		goto failed_1port_device_register;
	retval = usb_register(&huawei_voice_driver);
	if (retval)
		goto failed_driver_register;

	printk(KERN_INFO KBUILD_MODNAME ": " DRIVER_VERSION ":"
	       DRIVER_DESC "\n");

	return 0;

failed_driver_register:
	usb_serial_deregister(&huawei_voice_1port_device);
failed_1port_device_register:
	return retval;
}

static void __exit huawei_voice_exit(void)
{
	usb_deregister(&huawei_voice_driver);
	usb_serial_deregister(&huawei_voice_1port_device);
}

module_init(huawei_voice_init);
module_exit(huawei_voice_exit);

static int huawei_voice_probe(struct usb_serial *serial,
			const struct usb_device_id *id)
{
	struct usb_wwan_intf_private *data;

	if(serial->dev->descriptor.idVendor == HUAWEI_VENDOR_ID &&
		serial->dev->descriptor.idProduct == HUAWEI_PRODUCT_E600 &&
		serial->interface->cur_altsetting->desc.bInterfaceNumber != 1)
		return -ENODEV;

	data = serial->private = kzalloc(sizeof(struct usb_wwan_intf_private), GFP_KERNEL);

	if (!data)
		return -ENOMEM;
	data->send_setup = huawei_voice_send_setup;
	spin_lock_init(&data->susp_lock);
	data->private = (void *)id->driver_info;
	return 0;
}

static enum huawei_voice_blacklist_reason is_blacklisted(const u8 ifnum,
				const struct huawei_voice_blacklist_info *blacklist)
{
	const u8  *info;
	int i;

	if (blacklist) {
		info = blacklist->ifaceinfo;

		for (i = 0; i < blacklist->infolen; i++) {
			if (info[i] == ifnum)
				return blacklist->reason;
		}
	}
	return OPTION_BLACKLIST_NONE;
}

static void huawei_voice_instat_callback(struct urb *urb)
{
	int err;
	int status = urb->status;
	struct usb_serial_port *port =  urb->context;
	struct huawei_voice_port_private *portdata = usb_get_serial_port_data(port);

	dbg("%s", __func__);
	dbg("%s: urb %p port %p has data %p", __func__, urb, port, portdata);

	if (status == 0) {
		struct usb_ctrlrequest *req_pkt =
				(struct usb_ctrlrequest *)urb->transfer_buffer;

		if (!req_pkt) {
			dbg("%s: NULL req_pkt", __func__);
			return;
		}
		if ((req_pkt->bRequestType == 0xA1) &&
				(req_pkt->bRequest == 0x20)) {
			int old_dcd_state;
			unsigned char signals = *((unsigned char *)
					urb->transfer_buffer +
					sizeof(struct usb_ctrlrequest));

			dbg("%s: signal x%x", __func__, signals);

			old_dcd_state = portdata->dcd_state;
			portdata->cts_state = 1;
			portdata->dcd_state = ((signals & 0x01) ? 1 : 0);
			portdata->dsr_state = ((signals & 0x02) ? 1 : 0);
			portdata->ri_state = ((signals & 0x08) ? 1 : 0);

			if (old_dcd_state && !portdata->dcd_state) {
				struct tty_struct *tty =
						tty_port_tty_get(&port->port);
				if (tty && !C_CLOCAL(tty))
					tty_hangup(tty);
				tty_kref_put(tty);
			}
		} else {
			dbg("%s: type %x req %x", __func__,
				req_pkt->bRequestType, req_pkt->bRequest);
		}
	} else
		err("%s: error %d", __func__, status);

	/* Resubmit urb so we continue receiving IRQ data */
	if (status != -ESHUTDOWN && status != -ENOENT) {
		err = usb_submit_urb(urb, GFP_ATOMIC);
		if (err)
			dbg("%s: resubmit intr urb failed. (%d)",
				__func__, err);
	}
}

/** send RTS/DTR state to the port.
 *
 * This is exactly the same as SET_CONTROL_LINE_STATE from the PSTN
 * CDC.
*/
static int huawei_voice_send_setup(struct usb_serial_port *port)
{
	struct usb_serial *serial = port->serial;
	struct usb_wwan_intf_private *intfdata =
		(struct usb_wwan_intf_private *) serial->private;
	struct huawei_voice_port_private *portdata;
	int ifNum = serial->interface->cur_altsetting->desc.bInterfaceNumber;
	int val = 0;
	dbg("%s", __func__);

	if (is_blacklisted(ifNum,
			   (struct huawei_voice_blacklist_info *) intfdata->private)
	    == OPTION_BLACKLIST_SENDSETUP) {
		dbg("No send_setup on blacklisted interface #%d\n", ifNum);
		return -EIO;
	}

	portdata = usb_get_serial_port_data(port);

	if (portdata->dtr_state)
		val |= 0x01;
	if (portdata->rts_state)
		val |= 0x02;

	return usb_control_msg(serial->dev,
		usb_rcvctrlpipe(serial->dev, 0),
		0x22, 0x21, val, ifNum, NULL, 0, USB_CTRL_SET_TIMEOUT);
}

/* Helper functions used by usb_wwan_setup_urbs */
static struct urb *huawei_voice_setup_urb(struct usb_serial *serial, int endpoint,
				      int dir, void *ctx, char *buf, int len,
				      void (*callback) (struct urb *))
{
	struct urb *urb;

	if (endpoint == -1)
		return NULL;	/* endpoint not needed */

	urb = usb_alloc_urb(0, GFP_KERNEL);	/* No ISO */
	if (urb == NULL) {
		dbg("%s: alloc for endpoint %d failed.", __func__, endpoint);
		return NULL;
	}

	/* Fill URB using supplied data. */
	usb_fill_bulk_urb(urb, serial->dev,
			  usb_sndbulkpipe(serial->dev, endpoint) | dir,
			  buf, len, callback, ctx);

	return urb;
}

static void huawei_voice_outdat_callback(struct urb *urb)
{
       struct usb_serial_port *port;
       struct usb_wwan_port_private *portdata;
       struct usb_wwan_intf_private *intfdata;

       dbg("%s", __func__);

       port = urb->context;
       intfdata = port->serial->private;

        usb_serial_port_softint(port);
        usb_autopm_put_interface_async(port->serial->interface);
        portdata = usb_get_serial_port_data(port);
        spin_lock(&intfdata->susp_lock);
        intfdata->in_flight--;
        spin_unlock(&intfdata->susp_lock);

        smp_mb__before_clear_bit();
        clear_bit(4, &portdata->out_busy);
}

int huawei_voice_startup(struct usb_serial *serial)
{
    struct usb_serial_port *port;
    struct huawei_voice_port_private *portdata;

	usb_wwan_startup(serial);

	port = serial->port[0];
	portdata = usb_get_serial_port_data(port);

	portdata->out_urbs[4] = huawei_voice_setup_urb(serial,
											   port->bulk_out_endpointAddress,
											   USB_DIR_OUT,
											   port,
											   portdata->out_buffer[3],
											   0,
											   huawei_voice_outdat_callback);

	return 0;
}

/* Write */
int huawei_voice_write(struct tty_struct *tty, struct usb_serial_port *port,
                   const unsigned char *buf, int count)
{
        struct huawei_voice_port_private *portdata;
        struct usb_wwan_intf_private *intfdata;
        int todo;
        struct urb *this_urb = NULL;    /* spurious */
        int err;
        unsigned long flags;

        dbg("%s", __func__);

        portdata = usb_get_serial_port_data(port);
        intfdata = port->serial->private;

        todo = usb_wwan_write(tty, port, buf, count);

        dbg("%s: Write zero packet", __func__);

        this_urb = portdata->out_urbs[4];
		dbg("%s: endpoint %d buf %d", __func__,
			usb_pipeendpoint(this_urb->pipe), 4);

		err = usb_autopm_get_interface_async(port->serial->interface);
		if (err < 0)
				return todo;

		 /* send the data */
		this_urb->transfer_buffer_length = 0;

		spin_lock_irqsave(&intfdata->susp_lock, flags);
		if (intfdata->suspended) {
				usb_anchor_urb(this_urb, &portdata->delayed);
				spin_unlock_irqrestore(&intfdata->susp_lock, flags);
		} else {
				intfdata->in_flight++;
				spin_unlock_irqrestore(&intfdata->susp_lock, flags);
				err = usb_submit_urb(this_urb, GFP_ATOMIC);
				if (err) {
						dbg("usb_submit_urb %p (write bulk) failed "
							"(%d)", this_urb, err);
						clear_bit(4, &portdata->out_busy);
						spin_lock_irqsave(&intfdata->susp_lock, flags);
						 intfdata->in_flight--;
						 spin_unlock_irqrestore(&intfdata->susp_lock,
												flags);
						 return todo;
				 }
		}

		portdata->tx_start_time[4] = jiffies;
        return todo;
}


MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");

module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debug messages");
