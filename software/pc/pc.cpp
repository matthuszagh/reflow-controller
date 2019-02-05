#include "pc.hpp"
#include <iomanip>
#include <iostream>
#include <regex>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

/* This program must be run with super user privileges, otherwise it will fail to open device. */

std::vector<int> temp, target;

int main()
{
	libusb_context *ctx = nullptr;
	libusb_device_handle *devh = nullptr;
	unsigned char data[2];
	unsigned char data_read[64] = {0};
	int r;

	if (libusb_init(&ctx) < 0) {
		std::cerr << "Failed to create a libusb session\n";
		return 1;
	}
	std::puts("Initialized libusb.");

	devh = libusb_open_device_with_vid_pid(ctx, 0x03eb, 0x2044);
	if (devh == nullptr) {
		std::cerr << "Failed to open device.\n";
		return 1;
	}
	std::puts("Opened device.");

	if (libusb_kernel_driver_active(devh, 1) == 1) {
		std::cout << "Kernel driver active, must detach.\n";
		if (libusb_detach_kernel_driver(devh, 1) == 0)
			std::cout << "Kernel driver detached.\n";
	}

	r = libusb_claim_interface(devh, 1);
	if (r < 0) {
		std::cerr << "Failed to claim interface.\n";
		return 1;
	}
	std::puts("Interface associated with device.");

	data[0] = '!';
	data[1] = 'H';
	write_data(devh, data, sizeof(data));
	std::puts("!H written to device.");

	data[0] = '!';
	data[1] = 'S';
	write_data(devh, data, sizeof(data));
	std::puts("!S written to device.");

	while (true) {
		usleep(1e6);
		r = read_data(devh, data_read, sizeof(data_read));
		std::string data_str(reinterpret_cast<char *>(data_read));

		std::regex reg_pattern("\\d+");
		auto matches_begin =
			std::sregex_iterator(data_str.begin(), data_str.end(), reg_pattern);
		auto matches_end = std::sregex_iterator();
		int cnt = 0;
		double temp_val, room_val, target_val, pwm_val;
		int state_val;
		for (std::sregex_iterator it = matches_begin; it != matches_end; ++it) {
			std::smatch match = *it;
			std::string match_str = match.str();
			if (cnt == 0) {
				temp_val = (double)(std::stoi(match_str)) / 4;
				temp.push_back((int)temp_val);
			} else if (cnt == 1)
				room_val = (double)(std::stoi(match_str)) / 4;
			else if (cnt == 2) {
				target_val = (double)(std::stoi(match_str)) / 4;
				target.push_back(target_val);
			} else if (cnt == 3)
				pwm_val = (double)(std::stoi(match_str)) / 6250;
			else
				state_val = std::stoi(match_str);
			++cnt;
		}

		std::printf("temp: %6.2f, target: %6.2f, room: %6.2f, pwm: %6.4f, state: %3d\n",
			    temp_val, target_val, room_val, pwm_val, state_val);

		if (r < 0) {
			std::cout << "Failed to read data.\n";
			return 1;
		}
	}

	data[0] = '!';
	data[1] = 'H';
	write_data(devh, data, sizeof(data));

	if (libusb_release_interface(devh, 1) != 0) {
		std::cerr << "Failed to release the interface.\n";
		return 1;
	}

	libusb_close(devh);
	libusb_exit(ctx);

	return 0;
}

int write_data(libusb_device_handle *devh, unsigned char *data, int count)
{
	int bytes_written;
	int r = libusb_bulk_transfer(devh, (0x04 | LIBUSB_ENDPOINT_OUT), data, count,
				     &bytes_written, 0);
	if (!(r == 0 && bytes_written == count))
		std::cerr << "Write error.\n";
	return r;
}

int read_data(libusb_device_handle *devh, unsigned char *data, int count)
{
	int bytes_read;
	bool flag = 0;

	int r = libusb_bulk_transfer(devh, (0x83 | LIBUSB_ENDPOINT_IN), data, count, &bytes_read,
				     0);
	for (int i = 0; i < count; ++i) {
		if (flag == 0 && data[i] == '\n')
			flag = 1;
		else if (flag == 0)
			flag = 0;
		else {
			data[i] = '\0';
		}
	}

	return r;
}
