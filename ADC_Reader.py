#!/usr/bin/python
from Rasia_Prototyp.msg import zeros
import rospy
import time
from grove.i2c import Bus

ADC_DEFAULT_IIC_ADDR = 0X04

ADC_CHAN_NUM = 8

REG_RAW_DATA_START = 0X10
REG_VOL_START = 0X20
REG_RTO_START = 0X30

REG_SET_ADDR = 0XC0


class Pi_hat_adc():
    def __init__(self, bus_num=1, addr=ADC_DEFAULT_IIC_ADDR):
        self.bus = Bus(bus_num)
        self.addr = addr

    # get all raw adc data,THe max value is 4095,cause it is 12 Bit ADC
    def get_all_adc_raw_data(self):
        array = []
        for i in range(ADC_CHAN_NUM):
            data = self.bus.read_i2c_block_data(self.addr, REG_RAW_DATA_START + i, 2)
            val = data[1] << 8 | data[0]
            array.append(val)
        return array

    def get_nchan_adc_raw_data(self, n):
        data = self.bus.read_i2c_block_data(self.addr, REG_RAW_DATA_START + n, 2)
        val = data[1] << 8 | data[0]
        return val

    # get all data with unit mv.
    def get_all_vol_milli_data(self):
        array = [0 for _ in range(ADC_CHAN_NUM)]
        for i in range(ADC_CHAN_NUM):
            data = self.bus.read_i2c_block_data(self.addr, REG_VOL_START + i, 2)
            val = data[1] << 8 | data[0]
            array[i] = val
        return array

    def get_nchan_vol_milli_data(self, n):
        data = self.bus.read_i2c_block_data(self.addr, REG_VOL_START + n, 2)
        val = data[1] << 8 | data[0]
        return val


CHANNELS = 8
SAMPLES = 2000

def sign(num):
    if num < 0:
        return '-'
    if num > 0:
        return '+'
    return '0'


def get_zero_crossing(data):
    channels = [0 for _ in range(CHANNELS)]
    for i in range(1, len(data)):  # line
        for j in range(CHANNELS):  # channel
            if sign(data[i][j]) != sign(data[i - 1][j]):
                channels[j] += 1
    return channels


def move_detected(vol, threshhold_val):
    for channel in vol:
        if abs(channel) < threshhold_val:
            return False
    return True

def convert(val):
    return ((float(val) * (10 / 3.3)) - 5000.) / 1000.

if __name__ == '__main__':
    rospy.init_node('ADC_node')
    print "Start adc node!"
    threshold = float(rospy.get_param('/ADC_node/threshold'))

    adc = Pi_hat_adc()
    pub = rospy.Publisher("ADC_NODE", zeros, queue_size=1)
    import time
    while not rospy.is_shutdown():
        vol_data_arr = adc.get_all_vol_milli_data()
        converteed_val = [((float(vol_data) * (10 / 3.3)) - 5000.) / 1000. for vol_data in vol_data_arr]

        if move_detected(converteed_val, threshold):
            data = [vol_data_arr]
            data.extend([[0 for _ in range(CHANNELS)] for _ in range(SAMPLES-1)])

            start = time.time()
            for i in range(1999):
                data[1+i] = adc.get_all_vol_milli_data()
            print time.time() - start
            converted_data = [[convert(d) for d in sample] for sample in data]
            zero_crosses = get_zero_crossing(converted_data)
            msg = zeros()
            for ch in zero_crosses:
                msg.samples.append(ch)
            pub.publish(msg)
        else:
            rate.sleep()
