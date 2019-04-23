import time
# import usb1
import usb.core
import usb.util
import os
from pathlib import Path

from sys import platform
if platform == 'win32':
    import wmi

TARGET_CPU_TEMP = 65
MAX_TEMPERATURE_DELTA = 2
UPDATE_INTERVAL = 1
MIN_CPU_DUTY = 10
MAX_CPU_DUTY = 100
SYS_DUTY_DFL = 15
CPU_DUTY_DFL = 15

USB_VID = 0xf055
USB_PID = 0x0202
USB_IF = 0
USB_EP_OUT = 0x02
USB_EP_IN = 0x81

HWMON_ROOT = Path('/sys/class/hwmon')


# globals
sys_duty = SYS_DUTY_DFL
cpu_duty = CPU_DUTY_DFL


def get_max_cpu_temp_win32(wmi_c) -> float:
    temp = 0

    for i in wmi_c.AIDA64_SensorValues():
        if i.ID.startswith('TCC'):
            temp = max(int(i.Value), temp)

    return temp


def get_max_cpu_temp_linux(temperature_handle) -> float:
    max_temp = 0.0

    for file in temperature_handle:
        file.seek(0)
        temp = int(file.readline().strip()) / 1000.0
        max_temp = max(temp, max_temp)

    return max_temp


def get_max_cpu_temp(temperature_handle) -> float:
    if platform.startswith('linux'):
        return get_max_cpu_temp_linux(temperature_handle)
    elif platform.startswith('win32'):
        return get_max_cpu_temp_win32(temperature_handle)


def start_update_loop(handle, temperature_handle):
    global cpu_duty

    while not time.sleep(UPDATE_INTERVAL):
        max_temp = get_max_cpu_temp(temperature_handle)
        deltaT = max_temp - TARGET_CPU_TEMP
        if deltaT > MAX_TEMPERATURE_DELTA:
            cpu_duty += (deltaT*2.0)
            cpu_duty = min(cpu_duty, MAX_CPU_DUTY)
        elif -deltaT > -MAX_TEMPERATURE_DELTA:
            cpu_duty += int(deltaT/2.0)
            cpu_duty = max(cpu_duty, MIN_CPU_DUTY)

        print('temp={}, duty={}'.format(max_temp, cpu_duty))

        # [fan5, fan4, fan3, fan2, fan1]
        tx_count = handle.write(USB_EP_OUT, [sys_duty, sys_duty, int(cpu_duty), sys_duty, sys_duty], 500)
        if tx_count != 5:
            raise RuntimeError('sent {} bytes'.format(tx_count))


if __name__ == '__main__':
    temperature_handle = None

    bluepill = usb.core.find(idVendor=USB_VID, idProduct=USB_PID)
    if bluepill is None:
        raise RuntimeError('Device not found')

    if platform.startswith('linux'):
        temperature_handle = []
        for hwmon_entry in HWMON_ROOT.iterdir():
            hwmon_name = hwmon_entry.joinpath('name').open(mode='r').readline().strip()
            if hwmon_name != 'coretemp':
                continue

            for sensor_entry in hwmon_entry.iterdir():
                if not sensor_entry.name.endswith('input'):
                    continue

                temperature_handle.append(sensor_entry.open(mode='r'))

        if bluepill.is_kernel_driver_active(USB_IF):
            print('detaching kernel driver')
            bluepill.detach_kernel_driver(USB_IF)

    elif platform.startswith('win32'):
        temperature_handle = wmi.WMI(namespace='root\\WMI')

    start_update_loop(bluepill, temperature_handle)
