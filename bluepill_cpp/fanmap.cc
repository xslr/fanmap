#include <libusb.h>
#include <cstdio>

const uint16_t VID = 0xf055;
const uint16_t PID = 0x0202;
const uint16_t FANMAP_INTERFACENUM = 0;
const uint16_t FANMAP_OUT_ENDPOINT = 0x02;
const uint16_t FANMAP_IN_ENDPOINT = 0x81;

uint8_t tx_buf[128];

struct libusb_iso_packet_descriptor iso_desc[] = {};

int main() {
  int r = libusb_init(NULL);
  if (r < 0) {
    return r;
  }

  libusb_device_handle *dev = libusb_open_device_with_vid_pid(NULL, VID, PID);
  if (NULL == dev) {
    fprintf(stderr, "Device not found. Check usb permissions and reconnect\n");
    return 1;
  }

  r = libusb_set_auto_detach_kernel_driver(dev, 1);
  if (r < 0) {
    fprintf(stderr, "Cannot set auto detach: %s\n", libusb_error_name(r));
    return r;
  }

  r = libusb_claim_interface(dev, FANMAP_INTERFACENUM);
  if (r < 0) {
    fprintf(stderr, "Cannot claim interface: %s\n", libusb_error_name(r));
    return r;
  }

  int xferred;
  r = libusb_interrupt_transfer(dev, FANMAP_OUT_ENDPOINT, tx_buf, 16, &xferred, 5000);
  if (r < 0) {
    fprintf(stderr, "transfer error: %s\n", libusb_error_name(r));
    return r;
  } else {
    printf("sent %d bytes\n", xferred);
  }

  libusb_close(dev);

  libusb_exit(NULL);
  return 0;
}
