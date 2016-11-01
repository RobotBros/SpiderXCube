from __future__ import print_function
from pywinusb import hid
import sys
import time

BatteryFactor = 4
BatteryCurrentResistor = 2
BatteryCurrentAmpFactor = 15
FRAME_LEN = 32

DATA = [0xfa, 0xfb, 0xaa, 
        0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50,
        0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50,
]

def format_data(data):
    assert isinstance(data, list) or isinstance(data, tuple), 'data should be list or tuple'
    return reduce(lambda y, x: y + '0x{:002x} '.format(x), data, '')

class NRF24L01Usb(object):

    def __init__(self, debug=False, recv_callback=None):
        self.opened = False
        self.DEBUG = debug
        self.recv_callback = recv_callback if recv_callback else self.default_callback

    def default_callback(self, data):
        pass

    def recv_handle(self, data):
        data = data.data[1:]
        if self.DEBUG:
            print('NRF24L01 Recv: ', format_data(data))
        self.recv_callback(data)

    def open(self):
        f = hid.HidDeviceFilter(vendor_id=0x1915)
        device = f.get_devices()
        if not device:
            raise Exception("NRF24L01 USB Doogle not found!")

        self.nrf = device[0]
        self.nrf.open()

        self.nrf.set_raw_data_handler(self.recv_handle)
        self.nrf_in = self.nrf.find_input_reports()[0]
        self.nrf_out = self.nrf.find_output_reports()[0]
        self.opened = True


    def _check_sum(self, data):
        assert isinstance(data, list) or isinstance(data, tuple), 'data should be list or tuple'
        return reduce(lambda x, y: x + y, data, 0) & 0xff ^ 0xff

    def build_frame(self, data):
        assert isinstance(data, list) or isinstance(data, tuple), 'data should be list or tuple'
        assert len(data) <= FRAME_LEN - 2, 'data should not exist the maxiumn frame len: {}'.format(FRAME_LEN - 2)
        frame = [0x00] + data + [self._check_sum(data)] + [0x0 for i in range(FRAME_LEN - len(data) - 1)]
        return frame

    def send(self, data):
        assert isinstance(data, list) or isinstance(data, tuple), 'data should be list or tuple'
        if not self.opened:
            self.open_nrf24l01_hid()

        frame = self.build_frame(data)
        self.nrf_out.set_raw_data(frame)
        if self.DEBUG:
            print('NRF24L01 Sending: ', format_data(frame))
        return self.nrf_out.send()

    def close(self):
        self.nrf.close()


if __name__ == "__main__":
    def recv_callback(data):
        vol_ad = data[3] << 8 | data[4]
        current_ad = data[5] << 8 | data[6]

        vol = (3.3 * (vol_ad) / 4096 * (BatteryFactor))
        current = (( 3.3 * 1000 / 4096 ) / BatteryCurrentResistor * ( current_ad ) / BatteryCurrentAmpFactor )
        print('Voltage: {:.2f}, Current: {:.2f}'.format(vol, current))
    nrf = NRF24L01Usb(debug=False, recv_callback=recv_callback)
    nrf.open()
    while 1:
        if nrf.send(DATA):
            pass
        else:
            print('Send failed.')
        time.sleep(1)
