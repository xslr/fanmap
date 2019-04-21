import wmi
import time
import usb1

wmi_c = None

target_temp = 65
max_temp_deviation = 2
update_interval = 2.0

vid = 0xf055
pid = 0x0202
usb_if = 0
usb_ep_out = 0x02
usb_ep_in = 0x81
sys_duty = 10
cpu_duty = 10


def get_max_cpu_temp():
    global wmi_c
    if wmi_c is None:
        wmi_c = wmi.WMI(namespace='root\\WMI')

    temp = 0

    for i in wmi_c.AIDA64_SensorValues():
        if i.ID.startswith('TCC'):
            temp = max(int(i.Value), temp)

    return temp


def start_update_loop(handle):
    global cpu_duty

    while not time.sleep(update_interval):
        max_temp = get_max_cpu_temp()
        if (max_temp - target_temp) > max_temp_deviation:
            cpu_duty += 1
            cpu_duty = min(cpu_duty, 100)
        elif (target_temp - max_temp) > max_temp_deviation:
            cpu_duty -= 1
            cpu_duty = max(cpu_duty, 0)

        print('temp={}, duty={}'.format(max_temp, cpu_duty))
        tx_count = handle.interruptWrite(usb_ep_out, [sys_duty, cpu_duty])


if __name__ == '__main__':
    with usb1.USBContext() as context:
        handle = context.openByVendorIDAndProductID(
            vid,
            pid,
            skip_on_error=True,
        )
        if handle is None:
            raise RuntimeError('device not present or no permissions')

        with handle.claimInterface(usb_if):
            # Do stuff with endpoints on claimed interface.
            # tx_count = handle.interruptWrite(usb_ep_out, [cpu_duty, sys_duty])
            # print('sent {} bytes'.format(tx_count))
            # exit(0)

            start_update_loop(handle)
