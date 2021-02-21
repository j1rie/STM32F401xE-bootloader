/*
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2017 Joerg Riechardt
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/dfu.h>
#include <libopencm3/stm32/tools.h>
#include <libopencm3/usb/dwc/otg_common.h>

#define APP_ADDRESS	0x08004000  // sector_start_addr[1]
#define MAX_ADDRESS	0x08040000
#define SECTOR_SIZE	2048

/* We need a special large control buffer for this device: */
uint8_t usbd_control_buffer[SECTOR_SIZE];
uint8_t curr_sector_num = 1; // sector of APP_ADDRESS
uint32_t sector_start_addr[8] = {0x08000000, 0x08004000, 0x08008000, 0x0800C000, 
			 	 0x08010000, 0x08020000, 0x08040000, 0x08060000};

static enum dfu_state usbdfu_state = STATE_DFU_IDLE;

static struct {
	uint8_t buf[sizeof(usbd_control_buffer)];
	uint16_t len;
	uint32_t addr;
	uint16_t blocknum;
} prog = {.addr = APP_ADDRESS};

const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x1209,
	.idProduct = 0x4443,
	.bcdDevice = 0x0100,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

const struct usb_dfu_descriptor dfu_function = {
	.bLength = sizeof(struct usb_dfu_descriptor),
	.bDescriptorType = DFU_FUNCTIONAL,
	.bmAttributes = USB_DFU_CAN_DOWNLOAD | USB_DFU_CAN_UPLOAD | USB_DFU_WILL_DETACH,
	.wDetachTimeout = 255,
	.wTransferSize = SECTOR_SIZE,
	.bcdDFUVersion = 0x0110,
};

const struct usb_interface_descriptor iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = 0xFE, /* Device Firmware Upgrade */
	.bInterfaceSubClass = 1,
	.bInterfaceProtocol = 2,
	.iInterface = 3,

	.extra = &dfu_function,
	.extralen = sizeof(dfu_function),
};

const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &iface,
}};

const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] = {
	"IRMP STM32 project",
	"STM32 Bootloader",
	"BL 01",
};

static uint8_t usbdfu_getstatus(uint32_t *bwPollTimeout)
{
	switch (usbdfu_state) {
	case STATE_DFU_DNLOAD_SYNC:
		usbdfu_state = STATE_DFU_DNBUSY;
		*bwPollTimeout = 1;
		return DFU_STATUS_OK;
	case STATE_DFU_MANIFEST_SYNC:
		/* Device will reset when read is complete */
		/* skip STATE_DFU_MANIFEST */
		usbdfu_state = STATE_DFU_MANIFEST_WAIT_RESET;
		return DFU_STATUS_OK;
	default:
		return DFU_STATUS_OK;
	}
}

static void usbdfu_getstatus_complete(usbd_device *device,
						struct usb_setup_data *req)
{
	(void)req;
	(void)device;

	switch (usbdfu_state) {
	case STATE_DFU_DNBUSY: ;
		uint32_t baseaddr = prog.addr + prog.blocknum * dfu_function.wTransferSize;
		gpio_toggle(GPIOC, GPIO13);
		if (baseaddr == sector_start_addr[curr_sector_num]) {
			flash_erase_sector(curr_sector_num, FLASH_CR_PROGRAM_X32);
			curr_sector_num++;
		}
		for (int i = 0; i < prog.len; i += 4)
			flash_program_word(baseaddr + i, *(uint32_t*)(prog.buf+i));

		/* Jump straight to dfuDNLOAD-IDLE, skipping dfuDNLOAD-SYNC */
		usbdfu_state = STATE_DFU_DNLOAD_IDLE;
		return;
	default:
		return;
	}
}

static enum usbd_request_return_codes usbdfu_control_request(usbd_device *device,
				  struct usb_setup_data *req, uint8_t **buf,
				  uint16_t *len,
				  void (**complete)(usbd_device *device,
						struct usb_setup_data *req))
{
	(void)device;

	if ((req->bmRequestType & 0x7F) != 0x21)
		return 0; /* Only accept class request */

	switch (req->bRequest) {
	case DFU_DNLOAD:
		if ((len == NULL) || (*len == 0)) {
			flash_lock();
			usbdfu_state = STATE_DFU_MANIFEST_SYNC;
			return 1;
		} else {
			/* Copy download data for use on GET_STATUS */
			prog.blocknum = req->wValue;
			prog.len = *len;
			memcpy(prog.buf, *buf, *len);
			if(usbdfu_state == STATE_DFU_IDLE)
				flash_unlock();
			usbdfu_state = STATE_DFU_DNLOAD_SYNC;
			return 1;
		}
	case DFU_CLRSTATUS:
		/* Clear error and return to dfuIDLE */
		if (usbdfu_state == STATE_DFU_ERROR)
			usbdfu_state = STATE_DFU_IDLE;
		return 1;
	case DFU_ABORT:
		/* Abort returns to dfuIDLE state */
		usbdfu_state = STATE_DFU_IDLE;
		return 1;
	case DFU_UPLOAD:
		if ((usbdfu_state == STATE_DFU_IDLE) ||
			(usbdfu_state == STATE_DFU_UPLOAD_IDLE)) {
			usbdfu_state = STATE_DFU_UPLOAD_IDLE;
			uint32_t baseaddr = prog.addr + req->wValue * dfu_function.wTransferSize;
			uint32_t copy_size = MAX_ADDRESS - baseaddr;
			gpio_toggle(GPIOC, GPIO13);
			if (copy_size >= dfu_function.wTransferSize) {
				memcpy(*buf, (void*)baseaddr, dfu_function.wTransferSize);
			} else {
				memcpy(*buf, (void*)baseaddr, copy_size);
				*len = copy_size;
				usbdfu_state = STATE_APP_DETACH;
			}
		}
		return 1;
	case DFU_GETSTATUS: {
		uint32_t bwPollTimeout = 0; /* 24-bit integer in DFU class spec */

		(*buf)[0] = usbdfu_getstatus(&bwPollTimeout);
		(*buf)[1] = bwPollTimeout & 0xFF;
		(*buf)[2] = (bwPollTimeout >> 8) & 0xFF;
		(*buf)[3] = (bwPollTimeout >> 16) & 0xFF;
		(*buf)[4] = usbdfu_state;
		(*buf)[5] = 0; /* iString not used here */
		*len = 6;

		*complete = usbdfu_getstatus_complete;

		return 1;
		}
	case DFU_GETSTATE:
		/* Return state with no state transition */
		*buf[0] = usbdfu_state;
		*len = 1;
		return 1;
	}

	return 0;
}

static bool dfuDnloadStarted(void) {
	return (usbdfu_state == STATE_DFU_DNBUSY || usbdfu_state == STATE_DFU_UPLOAD_IDLE) ? 1 : 0;
}

static bool dfuDnloadDone(void) {
	return (usbdfu_state == STATE_DFU_MANIFEST_WAIT_RESET) ? 1 : 0;
}

static bool dfuUploadDone(void) {
	return (usbdfu_state == STATE_APP_DETACH) ? 1 : 0;
}

static bool checkUserCode(uint32_t usrAddr) {
	uint32_t sp = *(volatile uint32_t *) usrAddr;

	if ((sp & 0x2FFE0000) == 0x20000000) {
		return (1);
	} else {
		return (0);
	}
}

static void jump_to_app_if_valid(void)
{
	/* Boot the application if it's valid */
	if(checkUserCode(APP_ADDRESS)) {
		/* Set vector table base address */
		SCB_VTOR = APP_ADDRESS & 0x3FFFF;
		/* Initialise master stack pointer */
		asm volatile ("msr msp, %0"::"g"
					(*(volatile uint32_t*)APP_ADDRESS));
		/* Jump to application */
		(*(void(**)())(APP_ADDRESS + 4))();
	}
}

static void RCC_DeInit(void)
{
	/* Set HSION bit */
	SET_REG(&RCC_CR, GET_REG(&RCC_CR)     | 0x00000001);

	/* Reset CFGR register */
	SET_REG(&RCC_CFGR, 0x00000000);

	/* Reset HSEON, CSSON and PLLON bits */
	SET_REG(&RCC_CR, GET_REG(&RCC_CR)     & 0xEAF6FFFF);

	/* Reset PLLCFGR register */
	SET_REG(&RCC_PLLCFGR, 0x24003010);

	/* Reset PLLI2SCFGR register */
	SET_REG(&RCC_PLLI2SCFGR, 0x20003000);

	/* Reset HSEBYP bit */
	SET_REG(&RCC_CR, GET_REG(&RCC_CR)     & 0xFFFBFFFF);

	/* Disable all interrupts */
	SET_REG(&RCC_CIR, 0x00000000);
}

int main(void)
{
	rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);

	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);
	gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO13);

	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_clear(GPIOA, GPIO12);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO12);

	volatile uint32_t delay;
	for(delay=800000;delay;delay--);

	rcc_periph_clock_enable(RCC_OTGFS);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

	usbd_device *usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config,
		usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
		
	//bugfix, disable VBUS sensing
#	include <libopencm3/usb/dwc/otg_fs.h>
	OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS;
	OTG_FS_GCCFG &= ~(OTG_GCCFG_VBUSASEN | OTG_GCCFG_VBUSBSEN);

	usbd_register_control_callback(usbd_dev,
			USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
			USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
			usbdfu_control_request);

	bool no_user_jump = !checkUserCode(APP_ADDRESS);

	int delay_count = 0;

#define DELAY 350000 // wait to be catched by upgrade program
	while ((delay_count++ < 1) || no_user_jump) {
		gpio_set(GPIOC, GPIO13);
		for (int i=0; i<DELAY; i++) {
			usbd_poll(usbd_dev);
			if(i== (DELAY/2))
				gpio_clear(GPIOC, GPIO13);
			if(dfuDnloadStarted()) {
				gpio_clear(GPIOC, GPIO13);
				while(!dfuDnloadDone() && !dfuUploadDone()) {
					usbd_poll(usbd_dev);
				}
				// wait for last status request
				while((GET_REG(&OTG_FS_DIEPCTL0) &  OTG_DIEPCTL0_NAKSTS) ==  OTG_DIEPCTL0_NAKSTS)
				break;
			}
		}
	}
	RCC_DeInit();
	jump_to_app_if_valid();
}
