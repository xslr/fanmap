import time
# import usb1
import usb.core
import usb.util
import os
from pathlib import Path

from sys import platform
if platform == 'win32':
    import wmi

target_temp = 70
max_temp_deviation = 2
update_interval = 0.5

vid = 0xf055
pid = 0x0202
usb_if = 0
usb_ep_out = 0x02
usb_ep_in = 0x81
sys_duty = 15
cpu_duty = 15
hwmon_root = Path('/sys/class/hwmon')


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

    while not time.sleep(update_interval):
        max_temp = get_max_cpu_temp(temperature_handle)
        if (max_temp - target_temp) > max_temp_deviation:
            cpu_duty += 1
            cpu_duty = min(cpu_duty, 100)
        elif (target_temp - max_temp) > max_temp_deviation:
            cpu_duty -= 1
            cpu_duty = max(cpu_duty, 0)

        print('temp={}, duty={}'.format(max_temp, cpu_duty))
        # [fan5, fan4, fan3, fan2, fan1]
        tx_count = handle.write(usb_ep_out, [sys_duty, sys_duty, cpu_duty, sys_duty, sys_duty], 500)
        if tx_count != 5:
            raise RuntimeError('sent {} bytes'.format(tx_count))


if __name__ == '__main__':
    temperature_handle = None

    bluepill = usb.core.find(idVendor=vid, idProduct=pid)
    if bluepill is None:
        raise RuntimeError('Device not found')

    if platform.startswith('linux'):
        temperature_handle = []
        for hwmon_entry in hwmon_root.iterdir():
            hwmon_name = hwmon_entry.joinpath('name').open(mode='r').readline().strip()
            if hwmon_name != ('coretemp'):
                continue

            for sensor_entry in hwmon_entry.iterdir():
                if not sensor_entry.name.endswith('input'):
                    continue

                temperature_handle.append(sensor_entry.open(mode='r'))

        if bluepill.is_kernel_driver_active(usb_if):
            print('detaching kernel driver')
            bluepill.detach_kernel_driver(usb_if)

    elif platform.startswith('win32'):
        temperature_handle = wmi.WMI(namespace='root\\WMI')

    start_update_loop(bluepill, temperature_handle)
