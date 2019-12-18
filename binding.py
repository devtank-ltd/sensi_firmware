import os
import sys
import math
import time
import serial
import weakref
import serial.tools.list_ports


_DEBUG = True if "DEBUG" in os.environ else False

class io_board_prop_t(object):
    def __init__(self, index, parent):
        self.parent = weakref.ref(parent)
        self.index  = index


class pps_t(io_board_prop_t):
    @property
    def value(self):
        parent = self.parent()
        r = parent.command("pps %u" % self.index)
        v = r[0].split(b':')[1].strip()
        return int(v)


class pwm_data_t(object):
    def __init__(self, freq, duty):
        self.freq = freq
        self.duty = duty


class pwm_t(io_board_prop_t):
    def __init__(self, index, parent):
        io_board_prop_t.__init__(self, index, parent)
        self._freq = None
        self._duty = None

    def refresh(self):
        parent = self.parent()
        r = parent.command("pwm")
        freq = int(r[0].split(b':')[1].strip())
        duty = int(r[1].split(b':')[1].strip())
        self._freq = freq
        self._duty = duty

    def _update(self, freq, duty):
        parent = self.parent()
        r = parent.command("pwm %u %u" % (freq, duty))
        freq = int(r[0].split(b':')[1].strip())
        duty = int(r[1].split(b':')[1].strip())
        self._freq = freq
        self._duty = duty

    @property
    def values(self):
        self.refresh()
        return pwm_data_t(self._freq, self._duty)

    @property
    def duty(self):
        self.refresh()
        return self._duty

    @duty.setter
    def duty(self, new_value):
        self._update(self._freq, new_value)
        return self._duty

    @property
    def frequency(self):
        self.refresh()
        return self._freq

    @frequency.setter
    def frequency(self, new_value):
        self._update(new_value, self._duty)
        return self._freq


class input_t(io_board_prop_t):
    @property
    def value(self):
        parent = self.parent()
        r = parent.command("input %u" % self.index)
        v = r[0].split(b':')[1].strip()
        return False if v == b"OFF" else True if v == b"ON" else None


class output_t(io_board_prop_t):
    @property
    def value(self):
        parent = self.parent()
        r = parent.command("output %u" % self.index)
        v = r[0].split(b':')[1].strip()
        return False if v == b"OFF" else True if v == b"ON" else None

    @value.setter
    def value(self, v):
        parent = self.parent()
        r = parent.command("output %u %u" % (self.index, int(v)))
        assert r[0] == b'Set output %02u to %s' % (self.index, b"ON" if v else b"OFF")


class adc_t(io_board_prop_t):
    def __init__(self, index, parent):
        io_board_prop_t.__init__(self, index, parent)
        self._min_value = 0
        self._max_value = 0
        self._avg_value = 0
        self._age = 0

    def update_values(self):
        parent = self.parent()
        r = parent.command("adc %u" % self.index)
        assert len(r) == 4
        parts = [part.strip() for part in r[0].split(b':') ]
        assert parts[0] == b"ADC"
        assert int(parts[1].split(b' ')[0]) == self.index
        self._min_value = int(r[1].split(b':')[1].strip())
        self._max_value = int(r[2].split(b':')[1].strip())
        parts = r[3].split(b':')[1].split(b'/')
        self._avg_value = float(parts[0].strip()) / int(parts[1].strip())
        self._age = time.time()

    def _refresh(self):
        if not self._age or (time.time() - self._age) > 1:
            self.update_values()

    @property
    def min_value(self):
        self._refresh()
        return self._min_value

    @property
    def max_value(self):
        self._refresh()
        return self._max_value

    @property
    def avg_value(self):
        self._refresh()
        return self._avg_value


class adcex_t(io_board_prop_t):
    def __init__(self, index, parent):
        io_board_prop_t.__init__(self, index, parent)
        self._age = time.time()
        self._raw_value = 0
        self._real_value = 0

    def update_values(self):
        parent = self.parent()
        r = parent.command("adcex %u" % self.index)
        assert len(r) == 3
        parts = [part.strip() for part in r[0].split(b':') ]
        assert len(parts) == 2
        assert parts[0] == b"ADCEX"
        assert int(parts[1].split(b' ')[0]) == self.index
        self._raw_value = int(r[1].split(b':')[1])
        r0 = ((self._raw_value * 470) / 0x8000) / 2
        _RTD_A = 3.9083e-3
        _RTD_B = -5.775e-7
        Z1 = -_RTD_A
        Z2 = _RTD_A * _RTD_A - (4 * _RTD_B)
        Z3 = (4 * _RTD_B) / 100
        Z4 = 2 * _RTD_B
        temp = Z2 + (Z3 * r0)
        temp = (math.sqrt(temp) + Z1) / Z4
        if temp < 0:
            rpoly = r0
            temp = -242.02
            temp += 2.2228 * rpoly
            rpoly *= r0  # square
            temp += 2.5859e-3 * rpoly
            rpoly *= r0 # ^3
            temp -= 4.8260e-6 * rpoly
            rpoly *= r0  # ^4
            temp -= 2.8183e-8 * rpoly
            rpoly *= r0  # ^5
            temp += 1.5243e-10 * rpoly
        self._real_value = temp

    def _refresh(self):
        if not self._age or (time.time() - self._age) > 1:
            self.update_values()

    @property
    def raw_value(self):
        self._refresh()
        return self._raw_value

    @property
    def real_value(self):
        self._refresh()
        return self._real_value


class io_board_py_t(object):
    __LOG_START_SPACER = b"============{"
    __LOG_END_SPACER   = b"}============"
    __PROP_MAP = {b"ppss" : pps_t,
                  b"inputs" : input_t,
                  b"outputs" : output_t,
                  b"adcs" : adc_t,
                  b"adcexs" : adcex_t}
    __READTIME = 2
    __WRITEDELAY = 0.001 # Stop flooding STM32 faster than it can deal with

    def __init__(self, dev):
        self.ppss = []
        self.inputs = []
        self.outputs = []
        self.adcs = []
        self.comm = serial.Serial(
                port=dev,
                baudrate=115200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1)
        r = self.command("count")
        print("")
        m = {}
        for line in r:
            parts = line.split(b':')
            name  = parts[0].lower().strip()
            count = int(parts[1])
            for n in range(0, count):
                child = io_board_py_t.__PROP_MAP[name](n + 1, self)
                children = getattr(self, name.decode())
                children += [child]

    def _read_line(self):
        line = self.comm.readline().strip()
        if _DEBUG:
            print("<<", line)
        return line

    def read_response(self):
        line = self._read_line()
        start = time.time()
        while line != io_board_py_t.__LOG_START_SPACER:
            line = self._read_line()
            assert time.time() - start < io_board_py_t.__READTIME

        line = self._read_line()
        data_lines = []

        while line != io_board_py_t.__LOG_END_SPACER:
            data_lines += [line]
            line = self._read_line()
            assert time.time() - start < io_board_py_t.__READTIME

        return data_lines

    def command(self, cmd):
        for c in cmd:
            self.comm.write(c.encode())
            time.sleep(io_board_py_t.__WRITEDELAY)
        self.comm.write(b'\n')
        self.comm.flush()
        if _DEBUG:
            print(">>", cmd)
        return self.read_response()
