#include <stdlib.h>
#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>

#define LEN(x)  (sizeof(x)/sizeof(x[0]))

static usbd_device *usbd_dev;

enum {
  EP_RAW_HID_OUT = 0x02,
  EP_RAW_HID_IN = 0x81,
};

const struct usb_device_descriptor dev_descr = {
  .bLength = USB_DT_DEVICE_SIZE,
  .bDescriptorType = USB_DT_DEVICE,
  .bcdUSB = 0x0200,
  .bDeviceClass = 0,
  .bDeviceSubClass = 0,
  .bDeviceProtocol = 0,
  .bMaxPacketSize0 = 64,
  .idVendor = 0xf055,
  .idProduct = 0x0202,
  .bcdDevice = 0x0200,
  .iManufacturer = 1,
  .iProduct = 2,
  .iSerialNumber = 3,
  .bNumConfigurations = 1,
};

static const uint8_t hid_report_descriptor[] = {
   0x06, 0x00, 0xFF,    // Global  Usage page = 0xFF00 (Vendor-defined pages are in the range 0xFF00 through 0xFFFF)
   0x09, 0x01,          // Local   Usage (vendor usage 1)
   0xA1, 0x01,          // Main    Collection (application) begin
   0x15, 0x00,          // Global  Logical minimum (0) applies to each byte
   0x26, 0xFF, 0x00,    // Global  Logical maximum (255)
   0x75, 0x08,          // Global  Report Size (8 bits)
   0x95, 0x05,          //         Report count (5)
   0xc0                 // Main    End collection
};

static const struct {
  struct usb_hid_descriptor hid_descriptor;
  struct {
    uint8_t bReportDescriptorType;
    uint16_t wDescriptorLength;
  } __attribute__((packed)) hid_report;
} __attribute__((packed)) hid_function = {
  .hid_descriptor = {
    .bLength = sizeof(hid_function),
    .bDescriptorType = USB_DT_HID,
    .bcdHID = 0x0100,
    .bCountryCode = 0,
    .bNumDescriptors = 1,
  },
  .hid_report = {
     .bReportDescriptorType = USB_DT_REPORT,
     .wDescriptorLength = sizeof(hid_report_descriptor),
  }
};

// endpoint for fan speed control
const struct usb_endpoint_descriptor hid_endpoint[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = EP_RAW_HID_IN ,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 16,
    .bInterval = 10,  // largest value that linux doesn't cry about
  },
  {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = EP_RAW_HID_OUT ,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 16,
    .bInterval = 10,  // largest value that linux doesn't cry about
  },
};

const struct usb_interface_descriptor hid_iface = {
  .bLength = USB_DT_INTERFACE_SIZE,
  .bDescriptorType = USB_DT_INTERFACE,
  .bInterfaceNumber = 0,
  .bAlternateSetting = 0,
  .bNumEndpoints = 2,
  .bInterfaceClass = USB_CLASS_HID,
  .bInterfaceSubClass = 0, /* non boot device (used for kb or mouse) */
  .bInterfaceProtocol = 0, /* not predefined protocol */
  .iInterface = 0,
  .endpoint = hid_endpoint,
  .extra = &hid_function,
  .extralen = sizeof(hid_function),
};

const struct usb_interface ifaces[] = {{
  .num_altsetting = 1,
  .altsetting = &hid_iface,
}};

const struct usb_config_descriptor config = {
  .bLength = USB_DT_CONFIGURATION_SIZE,
  .bDescriptorType = USB_DT_CONFIGURATION,
  .wTotalLength = 0,
  .bNumInterfaces = 1,
  .bConfigurationValue = 1,
  .iConfiguration = 0,
  .bmAttributes = 0x80,
  .bMaxPower = 0x32,  // times 2 ma current is drawn
  .interface = ifaces,
};

static const char *usb_strings[] = {
  "Subrat",          // mfr
  "Fan controller",  // product
  "123456789",       // serial
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

typedef struct {
  uint8_t ticks;
} blink_t;

const uint8_t BLINK_1x = 4;
const uint8_t BLINK_2x = 2 * BLINK_1x;
const uint8_t BLINK_4x = 4 * BLINK_1x;

blink_t blinkInfo = {
  .ticks = BLINK_4x,
};

static void blink(const uint8_t duration) {
  blinkInfo.ticks = duration;
}

static void blink_tick() {
  if (blinkInfo.ticks > 0) {
    blinkInfo.ticks--;
    if (blinkInfo.ticks % 2 == 0) {
      gpio_toggle(GPIOC, GPIO13);
    }

    if (blinkInfo.ticks == 0) {
      gpio_set(GPIOC, GPIO13);
    }
  }
}

static uint8_t pwm_duty_1 = 15;
static uint8_t pwm_duty_2 = 15;
static uint8_t pwm_duty_3 = 15;
static uint8_t pwm_duty_4 = 15;
static uint8_t pwm_duty_5 = 15;

static bool haveRxCb = false;

char rxbuf[64] __attribute__ ((aligned(4))) = {0};
char txbuf[64] __attribute__ ((aligned(4))) = {0};


static void usbdata_rx_cb(usbd_device *usbd_dev, uint8_t ep) {
  haveRxCb = true;
}

static void usbdata_tx_cb(usbd_device *usbd_dev, uint8_t ep) {
  usbd_ep_write_packet(usbd_dev, EP_RAW_HID_IN, txbuf, 64);
}

static enum usbd_request_return_codes hid_control_request(usbd_device *dev,
                                                          struct usb_setup_data *req,
                                                          uint8_t **buf,
                                                          uint16_t *len,
                                                          void (**complete)(usbd_device *, struct usb_setup_data *))
{
  (void)complete;
  (void)dev;

  if((req->bmRequestType != 0x81) ||
     (req->bRequest != USB_REQ_GET_DESCRIPTOR) ||
     (req->wValue != 0x2200))
    return USBD_REQ_NOTSUPP;

  /* Handle the HID report descriptor. */
  *buf = (uint8_t *)hid_report_descriptor;
  *len = sizeof(hid_report_descriptor);

  return USBD_REQ_HANDLED;
}

static void hid_set_config(usbd_device *dev, uint16_t wValue)
{
  (void)wValue;
  (void)dev;

  usbd_ep_setup(dev, EP_RAW_HID_IN, USB_ENDPOINT_ATTR_INTERRUPT, 64, &usbdata_tx_cb);
  usbd_ep_setup(dev, EP_RAW_HID_OUT, USB_ENDPOINT_ATTR_INTERRUPT, 64, &usbdata_rx_cb);

  usbd_register_control_callback(
                                 dev,
                                 USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
                                 USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
                                 hid_control_request);

  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
  /* SysTick interrupt every N clock pulses: set reload to N-1 */
  systick_set_reload(99999);
  systick_interrupt_enable();
  systick_counter_enable();
}

void clock_setup() {
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_TIM2);
  rcc_periph_clock_enable(RCC_TIM3);
}

void gpio_setup() {
  // fan pwm
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_TIM2_CH4 |  // PA3
                GPIO_TIM2_CH3 |  // PA2
                GPIO_TIM2_CH2);  // PA1

  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_TIM3_CH4 |  // PB1
                GPIO_TIM3_CH3);  // PB0
  // usb
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
  gpio_clear(GPIOA, GPIO12);

  // onboard led
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
  gpio_set(GPIOC, GPIO13);
}

void timer_setup() {
  // TIM2
  //nvic_enable_irq(NVIC_TIM2_IRQ);
  timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  rcc_periph_reset_pulse(RST_TIM2);
  timer_set_prescaler(TIM2, ((rcc_apb1_frequency) / 1350000));

  /* Disable preload. */
  timer_enable_preload(TIM2);

  /* Continous mode. */
  timer_continuous_mode(TIM2);

  timer_set_repetition_counter(TIM2, 0);

  /* Period (25kHz). */
  timer_set_period(TIM2, 100);

  /* Counter enable. */
  timer_enable_counter(TIM2);

  timer_disable_oc_output(TIM2, TIM_OC4);
  timer_set_oc_mode(TIM2, TIM_OC4, TIM_OCM_PWM1);
  timer_set_oc_value(TIM2, TIM_OC4, 0);
  timer_enable_oc_output(TIM2, TIM_OC4);

  timer_disable_oc_output(TIM2, TIM_OC3);
  timer_set_oc_mode(TIM2, TIM_OC3, TIM_OCM_PWM1);
  timer_set_oc_value(TIM2, TIM_OC3, 0);
  timer_enable_oc_output(TIM2, TIM_OC3);

  timer_disable_oc_output(TIM2, TIM_OC2);
  timer_set_oc_mode(TIM2, TIM_OC2, TIM_OCM_PWM1);
  timer_set_oc_value(TIM2, TIM_OC2, 0);
  timer_enable_oc_output(TIM2, TIM_OC2);

  // TIM3
  //nvic_enable_irq(NVIC_TIM3_IRQ);
  timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  rcc_periph_reset_pulse(RST_TIM3);
  timer_set_prescaler(TIM3, ((rcc_apb1_frequency) / 1350000));

  /* Disable preload. */
  timer_enable_preload(TIM3);

  /* Continous mode. */
  timer_continuous_mode(TIM3);

  timer_set_repetition_counter(TIM3, 0);

  /* Period (25kHz). */
  timer_set_period(TIM3, 100);

  /* Counter enable. */
  timer_enable_counter(TIM3);

  timer_disable_oc_output(TIM3, TIM_OC4);
  timer_set_oc_mode(TIM3, TIM_OC4, TIM_OCM_PWM1);
  timer_set_oc_value(TIM3, TIM_OC4, 0);
  timer_enable_oc_output(TIM3, TIM_OC4);

  timer_disable_oc_output(TIM3, TIM_OC3);
  timer_set_oc_mode(TIM3, TIM_OC3, TIM_OCM_PWM1);
  timer_set_oc_value(TIM3, TIM_OC3, 0);
  timer_enable_oc_output(TIM3, TIM_OC3);
}

void update_fan_duty_cycles() {
  timer_set_oc_value(TIM2, TIM_OC2, pwm_duty_1); // fan1
  timer_set_oc_value(TIM2, TIM_OC3, pwm_duty_2); // fan2
  timer_set_oc_value(TIM2, TIM_OC4, pwm_duty_3); // fan3-cpu
  timer_set_oc_value(TIM3, TIM_OC3, pwm_duty_4); // fan4
  timer_set_oc_value(TIM3, TIM_OC4, pwm_duty_5); // fan5
}

int main(void) {
  rcc_clock_setup_in_hsi_out_48mhz();

  clock_setup();
  gpio_setup();
  timer_setup();

  update_fan_duty_cycles();

  /*
   * This is a somewhat common cheap hack to trigger device re-enumeration
   * on startup.  Assuming a fixed external pullup on D+, (For USB-FS)
   * setting the pin to output, and driving it explicitly low effectively
   * "removes" the pullup.  The subsequent USB init will "take over" the
   * pin, and it will appear as a proper pullup to the host.
   * The magic delay is somewhat arbitrary, no guarantees on USBIF
   * compliance here, but "it works" in most places.
   */

  for (unsigned i = 0; i < 800000; i++) {
    __asm__("nop");
  }

  usbd_dev = usbd_init(&st_usbfs_v1_usb_driver,
                       &dev_descr,
                       &config,
                       usb_strings,
                       LEN(usb_strings),
                       usbd_control_buffer,
                       LEN(usbd_control_buffer));
  usbd_register_set_config_callback(usbd_dev, hid_set_config);

  while (1) {
    usbd_poll(usbd_dev);

    if (haveRxCb) {
      haveRxCb = false;
      blink(BLINK_1x);
      int len = usbd_ep_read_packet(usbd_dev, EP_RAW_HID_OUT, rxbuf, 64);
      if (len == 2) {
        pwm_duty_1
          = pwm_duty_2
          = pwm_duty_4
          = pwm_duty_5
          = rxbuf[0] > 100 ? 100 : rxbuf[0];
        pwm_duty_3 = rxbuf[1] > 100 ? 100 : rxbuf[1];
        update_fan_duty_cycles();
      } else if (len == 5) {
        pwm_duty_1 = rxbuf[0] > 100 ? 100 : rxbuf[0];
        pwm_duty_2 = rxbuf[1] > 100 ? 100 : rxbuf[1];
        pwm_duty_3 = rxbuf[2] > 100 ? 100 : rxbuf[2];
        pwm_duty_4 = rxbuf[3] > 100 ? 100 : rxbuf[3];
        pwm_duty_5 = rxbuf[4] > 100 ? 100 : rxbuf[4];
        update_fan_duty_cycles();
      } else {
        //unsupported command
      }

      memcpy(txbuf, rxbuf, len);
      usbd_ep_write_packet(usbd_dev, EP_RAW_HID_IN, txbuf, len);
    }
  }
}

void sys_tick_handler(void)
{
  static uint8_t x = 0;
  x++;
  if (x == 0) {
    blink(BLINK_2x);
  }

  blink_tick();
  return;
}
