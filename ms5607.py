import time
import math

class MS5607:
    """
    Instantiable class representing a ms5607 sensor.
    adjusted for usage under micropyhon 
    
    Get the latest at:
    below reference wher used to build a class which could 
    in micro pyhon, enjoy and with thanks to original contributors
    
    ported from https://github.com/rsolomon/py-MS5607

    Ported from http://code.google.com/p/ardroneme/
    """

    _CMD_RESET = 0x1E
    _CMD_ADC_READ = 0x00
    _CMD_ADC_CONV = 0x40
    _CMD_ADC_D1 = 0x00
    _CMD_ADC_D2 = 0x10
    _CMD_ADC_256 = 0x00
    _CMD_ADC_512 = 0x02
    _CMD_ADC_1024 = 0x04
    _CMD_ADC_2048 = 0x06
    _CMD_ADC_4096 = 0x08
    _CMD_PROM_RD = 0xA0

    def __init__(self, i2c_bus, addr=0x77, debug=False):
        self._i2c_ms = i2c_bus
        self._i2c_addr = addr
        self._dbg = debug
 
        self._i2c_ms.start()
        self.reset_sensor()
        time.sleep(.004)
        self._coefficients = self.__read_coefficients()

    def reset_sensor(self):
        arr = bytearray(1)
        arr[0] = MS5607._CMD_RESET
        self._i2c_ms.writeto(self._i2c_addr, arr)

    def __read_coefficient(self, coef_num):
        arr = bytearray(1)
        results = bytearray(2)
        arr[0] = MS5607._CMD_PROM_RD + coef_num * 2
        self._i2c_ms.writeto(self._i2c_addr, arr)
        time.sleep(.003)
        results = self._i2c_ms.readfrom(self._i2c_addr, 2)
        rC = 256 * results[0] # read MSB
        rC = rC + results[1]  # read LSB
        return rC

    def __read_coefficients(self):
        coefficients = [0] * 6
        for n in range(0, 6):
            coefficients[n] = self.__read_coefficient(n + 1)
        if self._dbg:
            print('PROM values -> [%s]' % ', '.join(map(str, coefficients)))
        return coefficients

    def __prom_valid(self, n_prom):
        # will use prom data array to validate prom content
        n_rem = 0x00
        crc_read = n_prom[7]
        if self._dbg:
            print ('CRC_0 - %x' % crc_read)
        n_prom[7]=(0xFF00 & (n_prom[7]))
        for cnt in range (16):
            if (cnt & 0x01) == 1:
                n_rem = n_rem ^ n_prom[cnt >> 1] & 0x00ff
                if self._dbg:
                    print ('CRC_1 - %x' % n_rem)
            else:
                n_rem = n_rem ^ n_prom[cnt >> 1] >> 8
                if self._dbg:
                    print ('CRC_2 - %x' % n_rem)
            for x in range (7, -1, -1):
                if n_rem & 0x8000 != 0:
                    n_rem = (n_rem << 1) ^ 0x3000
                    if self._dbg:
                        print ('CRC_3 - %x' % n_rem)
                else:
                    n_rem = n_rem << 1
                    if self._dbg:
                        print ('CRC_4 - %x' % n_rem)
        n_rem = (0x000F & (n_rem >> 12))
        if self._dbg:
            print ('CRC_5 -> %x' % n_rem)
        if crc_read & 0x000f == n_rem:
            return True
        else:
            return False
    
    def __read_adc(self, cmd):
        arr = bytearray(1)
        read_bytes = bytearray(3)
        arr[0] = (MS5607._CMD_ADC_CONV | cmd)       # Send conversion command
        if self._dbg:
            print ('ADC cmd -> %x)' % arr[0])
        self._i2c_ms.writeto(self._i2c_addr, arr)
        # Map of times to delay for conversions
        delay_time = {}
        delay_time[MS5607._CMD_ADC_256] = 0.001
        delay_time[MS5607._CMD_ADC_512] = 0.003
        delay_time[MS5607._CMD_ADC_1024] = 0.004
        delay_time[MS5607._CMD_ADC_2048] = 0.006
        delay_time[MS5607._CMD_ADC_4096] = 0.010

        time.sleep(delay_time[cmd & 0x0f])       # Wait necessary conversion time

        arr[0] = MS5607._CMD_ADC_READ          # Send conversion command
        self._i2c_ms.writeto(self._i2c_addr, arr)
        read_bytes = self._i2c_ms.readfrom(self._i2c_addr, 3)
        if self._dbg:
            print ('ADC bytes -> %s, %s, %s' % (hex(read_bytes[0]), hex(read_bytes[1]),hex(read_bytes[2])))
        tmp = 65536 * read_bytes[0] # Read MSB
        tmp = tmp + 256 * read_bytes[1] # Read byte
        tmp = tmp + read_bytes[2] # Read LSB
        if self._dbg:
            print ('24Bit value -> %d' % tmp)
        return tmp
        
    def __convert_pressure_temperature(self, pressure, temperature):

        # Calculate 1st order pressure and temperature
        dT = temperature - self._coefficients[4] * 256
        if self._dbg:
            print ('dT %f' % dT)
        # Offset at actual temperature
        off = self._coefficients[1] * 4 + ((float(dT) / 2048) * (float(self._coefficients[3]) / 1024))
        if self._dbg:
            print ('Off %f' % off)
        # Sensitivity at actual temperature
        sens = self._coefficients[0] * 2 + ((float(dT) / 4096) * (float(self._coefficients[2]) / 1024))
        if self._dbg:
            print ('Sens %f' % sens)
        # Temperature compensated pressure
        press = (float(pressure) / 2048) * (float(sens) / 1024) - off
        return press
        
    def __convert_temperature(self, pressure, temperature):

        # Calculate 1st order pressure and temperature
        dT = temperature - self._coefficients[4] * 256
        if self._dbg:
            print ('dTemp %f' % dT)
        # Offset at actual temperature
        off = self._coefficients[1] * 4 + ((float(dT) / 2048) * (float(self._coefficients[3]) / 1024))
        tmp = 2000 + dT * (float(self._coefficients[5])) / 2**23
        #if self._dbg:
        #    print ('Offset %f' % off)
        # Sensitivity at actual temperature
        #sens = self._coefficients[0] * 2 + ((float(dT) / 4096) * (float(self._coefficients[2]) / 1024))
        #if self._dbg:
        #    print ('TempSens %f' % sens)
        # Temperature compensated pressure
        #press = (float(pressure) / 2048) * (float(sens) / 1024) - off
        
        return tmp
        
    def __pascal_to_cm(self, pressure_pa):
        # Lookup table converting pressure in Pa to altitude in cm.
        # Each LUT entry is the altitude in cm corresponding to an implicit
        # pressure value, calculated as [PA_INIT - 1024*index] in Pa.
        # The table is calculated for a nominal sea-level pressure  = 101325 Pa.
        pzlut_entries = 77
        pa_init = 104908
        pa_delta = 1024

        lookup_table = [
            -29408, -21087, -12700,  -4244,   4279,
            12874,  21541,  30281,  39095,  47986,
            56953,  66000,  75126,  84335,  93628,
            103006, 112472, 122026, 131672, 141410,
            151244, 161174, 171204, 181335, 191570,
            201911, 212361, 222922, 233597, 244388,
            255300, 266334, 277494, 288782, 300204,
            311761, 323457, 335297, 347285, 359424,
            371719, 384174, 396795, 409586, 422552,
            435700, 449033, 462560, 476285, 490216,
            504360, 518724, 533316, 548144, 563216,
            578543, 594134, 609999, 626149, 642595,
            659352, 676431, 693847, 711615, 729752,
            748275, 767202, 786555, 806356, 826627,
            847395, 868688, 890537, 912974, 936037,
            959766, 984206]

        if pressure_pa > pa_init:
            return lookup_table[0]

        inx = int(pa_init - pressure_pa) >> 10
        if inx >= pzlut_entries - 1:
            return lookup_table[pzlut_entries - 1]

        pa1 = pa_init - (inx << 10)
        z1 = lookup_table[inx]
        z2 = lookup_table[inx + 1]
        return (z1 + ((int(pa1 - pressure_pa) * (z2 - z1)) >> 10))

    def get_check_prom(self):
        n_prom = [0] * 8
        for n in range(0, 8):
            n_prom[n] = self.__read_coefficient(n)
        return self.__prom_valid(n_prom)

    def get_test(self, samples=2):
        accum = 0
        for n in range(0, samples):
            time.sleep(0.001)
            D2 = self.__read_adc(MS5607._CMD_ADC_D2 | MS5607._CMD_ADC_4096)
            if self._dbg:
                print ('D2 -> %d' % D2)
            time.sleep(0.001)
            D1 = self.__read_adc(MS5607._CMD_ADC_D1 | MS5607._CMD_ADC_4096)
            if self._dbg:
                print ('D1 -> %d' % D1)
            dT=D2 - self._coefficients[4] * 2**8
            OFF=self._coefficients[1] * 2**17 + dT * self._coefficients[3] / 2**6
            SENS=self._coefficients[0] * 2**16 + dT * self._coefficients[2] / 2**7
            T=(2000 + (dT * self._coefficients[5]) / 2**23) / 100
            P=(((D1 * SENS) / 2**21) - OFF) / 2**15 / 100
            print ('T -> %d' % T)
            print ('P -> %d' % P)
        return
    
    def get_altitude(self, samples=48):
        accum = 0
        for n in range(0, samples):
            time.sleep(0.001)
            temperature = self.__read_adc(MS5607._CMD_ADC_D2 | MS5607._CMD_ADC_4096)
            if self._dbg:
                print ('D2 -> %d' % temperature)
            time.sleep(0.001)
            pressure = self.__read_adc(MS5607._CMD_ADC_D1 | MS5607._CMD_ADC_4096)
            if self._dbg:
                print ('D1 -> %d' % pressure)
            press_conv = self.__convert_pressure_temperature(pressure, temperature)
            if self._dbg:
                print ('HPa -> %d' % press_conv)
            accum += press_conv

        avg = accum / samples
        if self._dbg:
            print ('Avage pressue %f HPa' % avg)
        return self.__pascal_to_cm(avg)
        
    def get_pressure(self, samples=48):
        accum = 0
        for n in range(0, samples):
            time.sleep(0.001)
            temperature = self.__read_adc(MS5607._CMD_ADC_D2 | MS5607._CMD_ADC_4096)
            if self._dbg:
                print ('D2 -> %d' % temperature)
            time.sleep(0.001)
            pressure = self.__read_adc(MS5607._CMD_ADC_D1 | MS5607._CMD_ADC_4096)
            if self._dbg:
                print ('D1 -> %d' % pressure)
            press_conv = self.__convert_pressure_temperature(pressure, temperature)
            if self._dbg:
                print ('HPa -> %d' % press_conv)
            accum += press_conv

        avg = accum / samples
        if self._dbg:
            print ('Avage pressure %f HPa' % avg)
        return avg
        
    def get_temperature(self, samples=48):
        accum = 0
        for n in range(0, samples):
            time.sleep(0.001)
            temperature = self.__read_adc(MS5607._CMD_ADC_D2 | MS5607._CMD_ADC_4096)
            if self._dbg:
                print ('D2 -> %d' % temperature)
            time.sleep(0.001)
            pressure = self.__read_adc(MS5607._CMD_ADC_D1 | MS5607._CMD_ADC_4096)
            if self._dbg:
                print ('D1 -> %d' % pressure)
            temperature_conv = self.__convert_temperature(pressure, temperature)
            if self._dbg:
                print ('Temp -> %d' % temperature_conv)
            accum += temperature_conv

        avg = accum / samples
        if self._dbg:
            print ('Avage temperature %f C' % avg)
        return avg