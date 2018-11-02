#include <libusb-1.0/libusb.h>
#include <vector>

void plot();
int write_data(libusb_device_handle *, unsigned char *, int);
int read_data(libusb_device_handle *, unsigned char *, int);
