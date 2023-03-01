import argparse
import socket
import time
import asyncio
from bleak import BleakClient, BleakGATTCharacteristic
from bleak import BleakScanner
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData

UDP_IP = "127.0.0.1"
UDP_PORT = 2000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

wind_rate = 500 * 1000 * 1000
motion_rate = 1000 * 1000 * 1000

NMEA0183_Sentences = ""
fw_number = None
args = {}

monotonic_wind = time.monotonic_ns()
monotonic_motion = time.monotonic_ns()


def socket(msg):
    try:
        sock.sendto(bytes(msg, "utf-8"), (UDP_IP, UDP_PORT))

    except (socket.timeout, ConnectionRefusedError):
        sock.close()


OPENWIND_WIND_CHARACTERISTIC_UUID = '0000cc91-0000-1000-8000-00805f9b34fb'
OPENWIND_BAT_CHARACTERISTIC_UUID = '0000bb91-0000-1000-8000-00805f9b34fb'
OPENWIND_MOV_ENABLE_CHARACTERISTIC_UUID = '0000aa82-0000-1000-8000-00805f9b34fb'
OPENWIND_FW_CHARACTERISTIC_UUID = '00002a26-0000-1000-8000-00805f9b34fb'
OPENWIND_SN_CHARACTERISTIC_UUID = '00002a25-0000-1000-8000-00805f9b34fb'

deviceFound = False
deviceAddress = None
deviceConnected = False


def checksum(msg):
    # Find the start of the NMEA sentence
    start_chars = "!$"
    for c in start_chars:
        i = msg.find(c)
        if i >= 0:
            break
    else:
        return False, None, None

    # Calculate the checksum on the message
    sum1 = 0
    for c in msg[i + 1:]:
        if c == '*':
            break
        sum1 = sum1 ^ ord(c)

    sum1 = sum1 & 0xFF

    return '{:x}'.format(int(sum1))


def get_signed_int16(msb, lsb):
    value = (msb << 8) | lsb
    return value - 0xfffe if (msb & 0x80) else value


def bat_data(data):
    soc = float((data[0] << 8) | data[1])
    consumption = float(get_signed_int16(data[2], data[3]))
    remain = float((data[4] << 8) | data[5])
    volts = float((data[6] << 8) | data[7])
    temp = float(get_signed_int16(data[8], data[9]))
    total = float((data[10] << 8) | data[11])

    print(f"soc: {soc:3.1f}% con: {consumption:3.1f}mA remain: {remain:3.1f}mAh vols: {volts:3.1f}mV temp: {temp:3.1f}C"
          f" total: {total:3.1f}mAh")


def wind_data_callback(sender: BleakGATTCharacteristic, data: bytearray):
    global fw_number
    global monotonic_wind
    global monotonic_motion
    global wind_rate
    global motion_rate

    new_monotonic = time.monotonic_ns()
    wind_ts_diff = new_monotonic - monotonic_wind
    motion_ts_diff = new_monotonic - monotonic_motion

    if wind_ts_diff > wind_rate:
        monotonic_wind = new_monotonic
        AWA = float((data[2] << 8) | data[1]) * 0.1  # °
        AWS = float((data[4] << 8) | data[3]) * 0.01  # kts

        print("AWA: " + "{:3.1f}".format(AWA) + " AWS: " + "{:3.1f}".format(AWS))

        NMEA0183_WIND_Sentece = "$WIMWV," + "{:3.1f}".format(AWA) + ",R," + "{:3.1f}".format(AWS) + ",N,A*"
        cs = str(checksum(NMEA0183_WIND_Sentece))
        NMEA0183_WIND_Sentece = NMEA0183_WIND_Sentece + cs.rjust(2, '0') + "\n"
        # print(NMEA0183_WIND_Sentece)
        socket(NMEA0183_WIND_Sentece)

    # Only if Firmware Version is same or above 1.25
    if float(fw_number) >= 1.25 and motion_ts_diff > motion_rate:
        monotonic_motion = new_monotonic
        YAW = float((data[6] << 8) | data[5]) * 1 / 16 - 90  # °
        PITCH = float(get_signed_int16(data[8], data[7])) * 1 / 16  # ° # Pitch and roll swapped in 1.34
        ROLL = float(get_signed_int16(data[10], data[9])) * 1 / 16  # °
        CALIBRATE_STATUS = data[11]  # %

        mag_calib = data[11] & 0b00000011
        acc_calib = (data[11] & 0b00001100) >> 2
        gyro_calib = (data[11] & 0b00110000) >> 4
        # print("C: " + hex(CALIBRATE_STATUS) + " " + bin(CALIBRATE_STATUS))
        # print("CALIB: mag: "+ str(mag_calib) + " acc: " + str(acc_calib) + " gyro: " + str(gyro_calib))

        if YAW < 0:
            YAW = 360 + YAW

        if ROLL >= 180:
            ROLL = ROLL - 360

        print("YAW: {:3.1f} PITCH: {:3.1f} ROLL: {:3.1f}".format(YAW, PITCH, ROLL))

        NMEA0183_HEADING_Sentece = "$WIHDM," + "{:3.1f}".format(YAW) + ",M*"
        cs = str(checksum(NMEA0183_HEADING_Sentece))
        NMEA0183_HEADING_Sentece = NMEA0183_HEADING_Sentece + cs.rjust(2, '0') + "\n"
        socket(NMEA0183_HEADING_Sentece)


def simple_callback(device: BLEDevice, advertisement_data: AdvertisementData):
    global deviceFound
    global deviceAddress
    print(device.address, "RSSI:", device.rssi, advertisement_data)

    if device.name == "OpenWind":
        print("Found OpenWind")
        deviceFound = True
        deviceAddress = device.address


def ow_disconnect_callback(client):
    global deviceConnected
    deviceConnected = False
    print("OpenWind with address {} got disconnected!".format(client.address))


def is_open_wind(device: BLEDevice, adv_data: AdvertisementData):
    return adv_data.local_name == 'OpenWind'


async def run():
    global deviceFound
    global deviceAddress
    global deviceConnected
    global fw_number
    global args

    if args.device is None:
        print("looking for device")
        device = await BleakScanner.find_device_by_filter(is_open_wind, timeout=10)
    else:
        device = args.device

    # while True:
    #     await scanner.start()
    #     await asyncio.sleep(5.0)
    #     await scanner.stop()
    #     if deviceFound:
    #         deviceFound = False
    #         break
    #

    stop_event = asyncio.Event()

    async with BleakClient(device, ow_disconnect_callback) as client:
        deviceConnected = True
        svcs = client.services
        print("Services:")
        for service in svcs:
            print(service)
        fw_number = await client.read_gatt_char(OPENWIND_FW_CHARACTERISTIC_UUID)
        print("Model Number: {0}".format("".join(map(chr, fw_number))))

        sn_number = await client.read_gatt_char(OPENWIND_SN_CHARACTERISTIC_UUID)

        if float(fw_number) >= 1.27:
            print("Serial Number: {0}".format(sn_number.hex()))
        else:
            print("Serial Number: {0}".format("".join(map(chr, sn_number))))

        write_value = bytearray([0x2C])
        await client.write_gatt_char(OPENWIND_MOV_ENABLE_CHARACTERISTIC_UUID, write_value)
        await asyncio.sleep(1.0)
        await client.start_notify(OPENWIND_WIND_CHARACTERISTIC_UUID, wind_data_callback)

        batt_info = await client.read_gatt_char(OPENWIND_BAT_CHARACTERISTIC_UUID)
        bat_data(batt_info)
        #        await stop_event.wait()
        while client.is_connected:
            await asyncio.sleep(10.0)
            batt_info = await client.read_gatt_char(OPENWIND_BAT_CHARACTERISTIC_UUID)
            bat_data(batt_info)


async def scanner():
    global args

    def callback(device: BLEDevice, advertising_data: AdvertisementData):
        if advertising_data.local_name == 'OpenWind':
            print("Found OpenWind device address={}".format(device.address))

    async with BleakScanner(callback) as scanner:
        await asyncio.sleep(args.scan_time)


def main():
    global args
    parser = argparse.ArgumentParser(prog="OpenWindRelay",
                                     description="Connects to Openwind and relays wind and heading NMEA data to udp socket")
    parser.add_argument('-v', '--verbose', action='store_true')
    parser.add_argument('-w', '--wind_rate', type=int, default=500, action='store',
                        help='minimum time between NMEA wind sentences in ms')
    parser.add_argument('-m', '--motion_rate', type=int, default=1000, action='store',
                        help='minimum time between NMEA heading sentences in ms')
    parser.add_argument('-b', '--batt_rate', type=int, action='store',
                        help='time between printing battery info to stdout in seconds')
    parser.add_argument('-c', '--batt_cutoff', type=int, default=10, action='store',
                        help='battery %% to shutoff')
    parser.add_argument('-d', '--device', action='store',
                        help='device address/uuid to connect')
    parser.add_argument('-l', '--list_devices', action='store_true', help='Only list OpenWind devices')
    parser.add_argument('-s', '--scan_time', type=int, default=10, action='store',
                        help='Bluetooth scan time in seconds')

    args = parser.parse_args()

    if args.list_devices:
        asyncio.run(scanner())
    else:
        asyncio.run(run())

    # while True:
    #     try:
    #         loop = asyncio.get_event_loop()
    #         loop.run_until_complete(run())
    #
    #         while True:
    #             print("waiting...")
    #             time.sleep(5)
    #
    #             if not deviceConnected:
    #                 loop.run_until_complete(run())
    #     except:
    #         print("something wrong but lets try again...")


if __name__ == "__main__":
    main()
