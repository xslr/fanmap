from collections import namedtuple
import time
# import usb1
import usb.core
import usb.util
import os
from pathlib import Path

from sys import platform
if platform == 'win32':
    import wmi

TARGET_CPU_TEMP = 70
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

CurvePoint = namedtuple('CurvePoint', ['temp', 'duty'])
cpu_fan_curve = [
    CurvePoint(0, 10),
    CurvePoint(30, 10),
    CurvePoint(40, 20),
    CurvePoint(50, 20),
    CurvePoint(60, 30),
    CurvePoint(70, 70),
    CurvePoint(80, 100),
]


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


def curve_points(temp):
    num_curve_pts = len(cpu_fan_curve)
    for i in range(0, num_curve_pts):
        this_curve_point = cpu_fan_curve[i]
        if i + 1 == num_curve_pts:
            next_curve_point = None
        else:
            next_curve_point = cpu_fan_curve[i + 1]

        if (temp >= this_curve_point.temp):
            if next_curve_point:
                if temp < next_curve_point.temp:
                    return this_curve_point, next_curve_point
                else:  # temp >= temp of next curve point. evaluate next curve point
                    continue
            else:
                return this_curve_point, None


def lerp_curve_points_to_duty(p_low, p_high, temp) -> int:
    duty = 100
    if None == p_low:
        if None != p_high:
            duty = p_high.duty
    elif None != p_high:
        duty = p_low.duty + ((temp - p_low.temp) / (p_high.temp - p_low.temp) * (p_high.duty - p_low.duty))

    return duty


def temp_to_duty(temp) -> int:
    duty = 100
    p_low, p_high = curve_points(temp)
    duty = lerp_curve_points_to_duty(p_low, p_high, temp)
    print(f'\r{temp}°C - {duty}% - {p_low} - {p_high}', end='')

    return duty


def start_update_loop(handle, temperature_handle):
    global cpu_duty

    while not time.sleep(UPDATE_INTERVAL):
        max_temp = get_max_cpu_temp(temperature_handle)
        cpu_duty = temp_to_duty(max_temp)
        # deltaT = max_temp - TARGET_CPU_TEMP
        # if deltaT > MAX_TEMPERATURE_DELTA:
        #     cpu_duty += (deltaT*2.0)
        #     cpu_duty = min(cpu_duty, MAX_CPU_DUTY)
        # elif deltaT < -MAX_TEMPERATURE_DELTA:
        #     cpu_duty += int(deltaT/2.0)
        #     cpu_duty = max(cpu_duty, MIN_CPU_DUTY)

        # print('temp={}, duty={}'.format(max_temp, cpu_duty), end='\r')

        # [fan5, fan4, fan3, fan2, fan1]
        tx_count = handle.write(USB_EP_OUT, [sys_duty, sys_duty, int(cpu_duty), int(cpu_duty), sys_duty], 500)
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
