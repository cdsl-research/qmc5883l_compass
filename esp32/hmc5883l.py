import math
from ustruct import pack
from array import array


class HMC5883L:
    __gain__ = {
        '0.88': (0 << 5, 0.73),
        '1.3':  (1 << 5, 0.92),
        '1.9':  (2 << 5, 1.22),
        '2.5':  (3 << 5, 1.52),
        '4.0':  (4 << 5, 2.27),
        '4.7':  (5 << 5, 2.56),
        '5.6':  (6 << 5, 3.03),
        '8.1':  (7 << 5, 4.35)
    }

    def __init__(self, i2c, gauss='1.3', declination=(0, 0)):
        # i2c初期化
        self.i2c = i2c
        self.i2c_add = 30
        i2c.start()

        # 地磁気センサ初期化
        i2c.writeto_mem(self.i2c_add, 0x00, pack('B', 0b111000))
        reg_value, self.gain = self.__gain__[gauss]
        i2c.writeto_mem(self.i2c_add, 0x01, pack('B', reg_value))
        i2c.writeto_mem(self.i2c_add, 0x02, pack('B', 0x00))
        i2c.stop()

        self.declination = (declination[0] + declination[1] / 60) * math.pi / 180
        self.data = array('B', [0] * 6)

    def read(self):
        # 地磁気センサからデータの読み込み
        data = self.data
        gain = self.gain

        self.i2c.readfrom_mem_into(self.i2c_add, 0x03, data)

        x = (data[0] << 8) | data[1]
        z = (data[2] << 8) | data[3]
        y = (data[4] << 8) | data[5]

        x = x - (1 << 16) if x & (1 << 15) else x
        y = y - (1 << 16) if y & (1 << 15) else y
        z = z - (1 << 16) if z & (1 << 15) else z

        x = round(x * gain, 4)
        y = round(y * gain, 4)
        z = round(z * gain, 4)

        return x, y, z

    def heading(self, x, y):
        #地磁気センサの値(x,y)から角度を求める

        heading_rad = math.atan2(y, x)
        heading_rad += self.declination
        # 修正
        if heading_rad < 0:
            heading_rad += 2 * math.pi
        elif heading_rad > 2 * math.pi:
            heading_rad -= 2 * math.pi

        # ラジアン → 度
        heading = heading_rad * 180 / math.pi
        degrees = math.floor(heading)
        minutes = round((heading - degrees) * 60)
        return degrees, minutes

    def format_result(self, x, y, z):
        # 角度表示
        degrees, minutes = self.heading(x, y)
        return 'X: {:.4f}, Y: {:.4f}, Z: {:.4f}, Heading: {}° {}′ '.format(x, y, z, degrees, minutes)

    def calc_deg(self, x, y):
        # 角度の返却
        degrees, minutes = self.heading(x, y)
        return (degrees, minutes)