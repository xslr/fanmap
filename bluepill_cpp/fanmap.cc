#include <libusb.h>
#include <atomic>
#include <cassert>
#include <cstdio>
#include <chrono>
#include <thread>
#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <iostream>
#include <regex>
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

const uint16_t VID = 0xf055;
const uint16_t PID = 0x0202;
const uint16_t FANMAP_INTERFACENUM = 0;
const uint16_t FANMAP_OUT_ENDPOINT = 0x02;
const uint16_t FANMAP_IN_ENDPOINT = 0x81;
const std::chrono::milliseconds TEMPERATURE_READ_INTERVAL(500);

std::atomic<bool> run_thread = true;
uint16_t target_temperature = 65;

const std::string SENSOR_NAME = "coretemp";

uint8_t tx_buf[128];

struct libusb_iso_packet_descriptor iso_desc[] = {};

class TemperatureSensor {
public:
  TemperatureSensor(const fs::path &path)
    : m_sensorFile(path)
  {
    if (!m_sensorFile.is_open()) {
      throw std::runtime_error("Failed to open sensor file");
    }
    m_div = 1000;
  }

  ~TemperatureSensor() = default;
  TemperatureSensor(TemperatureSensor&& sensor) {
    std::swap(m_path, sensor.m_path);
    std::swap(m_sensorFile, sensor.m_sensorFile);
  }
  TemperatureSensor(const TemperatureSensor&) = delete;

  uint32_t read() {
    uint32_t reading;
    m_sensorFile >> reading;
    m_sensorFile.clear();
    m_sensorFile.seekg(0);
    return reading;
  }

private:
  fs::path m_path;
  std::ifstream m_sensorFile;
  int m_div;
};

std::vector<TemperatureSensor> get_cpu_temp_sensors() {
  const std::regex sensor_fname_regex("temp\\d+_input");

  std::vector<TemperatureSensor> sensors;
  const std::string sysfs_path = "/sys/class/hwmon";
  for (const auto &hwmon : fs::recursive_directory_iterator(sysfs_path)) {
    std::cout << hwmon.path() / "name" << std::endl;
    std::ifstream sensorNameFile(hwmon.path() / "name");
    assert(sensorNameFile.is_open());
    std::string sensorName;
    sensorNameFile >> sensorName;
    std::cout << "  >> " << sensorName << std::endl;

    if (sensorName != SENSOR_NAME) continue;

    for (const auto &hwmon_entry : fs::directory_iterator(hwmon.path())) {
      const auto& fname = hwmon_entry.path().filename();
      if (std::regex_match(std::string(fname), sensor_fname_regex)) {
        std::cout << "    + " << hwmon_entry.path() << std::endl;
        sensors.push_back(TemperatureSensor(hwmon_entry.path()));
      }
    }
  }

  return sensors;
}

void temp_reader_function() {
  auto sensors = get_cpu_temp_sensors();
  std::cout << "Found " << sensors.size() << " sensors." << std::endl;

  while (run_thread.load()) {
    std::this_thread::sleep_for(TEMPERATURE_READ_INTERVAL);
    int max_temp = 0;
    for (auto &sensor : sensors) {
      int reading = sensor.read();
      max_temp = std::max(max_temp, reading);
    }
    std::cout << "max_temp = " << max_temp << std::endl;
  }
}

int main() {
  int r = libusb_init(nullptr);
  if (r < 0) {
    return r;
  }

  libusb_device_handle *dev = libusb_open_device_with_vid_pid(nullptr, VID, PID);
  if (nullptr == dev) {
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

  tx_buf[0] = 30;
  tx_buf[1] = 40;

  int xferred;
  r = libusb_interrupt_transfer(dev, FANMAP_OUT_ENDPOINT, tx_buf, 16, &xferred, 5000);
  if (r < 0) {
    fprintf(stderr, "transfer error: %s\n", libusb_error_name(r));
    return r;
  } else {
    printf("sent %d bytes\n", xferred);
  }

  std::thread temp_reader_thread(temp_reader_function);
  temp_reader_thread.join();

  libusb_close(dev);

  libusb_exit(nullptr);
  return 0;
}
