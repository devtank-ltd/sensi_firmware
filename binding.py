from __future__ import print_function
import os
import sys
import math
import time
import serial
import weakref
import serial.tools.list_ports


_debug_fnc = print if "DEBUG" in os.environ else None


def debug_print(msg):
    if _debug_fnc:
        _debug_fnc(msg)

def set_debug_print(_func):
    global _debug_fnc
    _debug_fnc = _func

def get_debug_print():
    return _debug_fnc



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
        self.adc_scale  = 1
        self.adc_offset = 0

    def update_values(self):
        parent = self.parent()
        r = parent.command("adc %u" % self.index)
        assert len(r) == 4
        parts = [part.strip() for part in r[0].split(b':') ]
        assert parts[0] == b"ADC"
        assert int(parts[1].split(b' ')[0]) == self.index
        raw_min_value = int(r[1].split(b':')[1].strip())
        raw_max_value = int(r[2].split(b':')[1].strip())
        parts = r[3].split(b':')[1].split(b'/')
        raw_avg_value = float(parts[0].strip()) / int(parts[1].strip())
        self._age = time.time()
        self._avg_value = raw_avg_value * self.adc_scale + self.adc_offset
        self._min_value = raw_min_value * self.adc_scale + self.adc_offset
        self._max_value = raw_max_value * self.adc_scale + self.adc_offset

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
    __PROP_MAP = {"ppss" : pps_t,
                  "inputs" : input_t,
                  "outputs" : output_t,
                  "adcs" : adc_t,
                  "adcexs" : adcex_t}
    __READTIME = 2
    __WRITEDELAY = 0.001 # Stop flooding STM32 faster than it can deal with
    NAME_MAP = { "F4_OUT"      : lambda board : board.adcs[1],
                 "F3_OUT"      : lambda board : board.adcs[2],
                 "F2_OUT"      : lambda board : board.adcs[3],
                 "F1_OUT"      : lambda board : board.adcs[4],
                 "TH2_OUT"     : lambda board : board.adcs[5],
                 "AIN_BUF_CH3" : lambda board : board.adcs[6],
                 "AIN_BUF_CH1" : lambda board : board.adcs[7],
                 "I_MON"       : lambda board : board.adcs[8],
                 "TH4_OUT"     : lambda board : board.adcs[9],
                 "TH3_OUT"     : lambda board : board.adcs[10],
                 "TH1_OUT"     : lambda board : board.adcs[11],
                 "AIN_BUF_CH4" : lambda board : board.adcs[12],
                 "AIN_BUF_CH2" : lambda board : board.adcs[13],
                 "GPIO1_EXT"   : lambda board : board.inputs[1],
                 "GPIO2_EXT"   : lambda board : board.inputs[2],
                 "GPIO3_EXT"   : lambda board : board.inputs[3],
                 "GPIO4_EXT"   : lambda board : board.inputs[4],
                 "GPIO5_EXT"   : lambda board : board.inputs[5],
                 "SB1"         : lambda board : board.inputs[6],
                 "SB2"         : lambda board : board.inputs[7],
                 "HS_OUT1"     : lambda board : board.outputs[1],
                 "HS_OUT2"     : lambda board : board.pwms[1],
                 "HS_OUT3"     : lambda board : board.outputs[2],
                 "HS_OUT4"     : lambda board : board.outputs[3],
                 "RL1"         : lambda board : board.outputs[4],
                 "RTD1"        : lambda board : board.adcexs[1],
                 "RTD2"        : lambda board : board.adcexs[2],
                 "RTD3"        : lambda board : board.adcexs[3],
                 "RTD4"        : lambda board : board.adcexs[4],
                 }

    # Scale then offset
    _ADC_CORRECTION_MAP = {"F1_OUT" : (0.045995342549755, -33.4574807550702)}

    def __init__(self, dev):
        self.ppss = []
        self.inputs = []
        self.outputs = []
        self.adcs = []
        self.adcexs = []
        self.comm = serial.Serial(
                port=dev,
                baudrate=115200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1)
        r = self.command("count")
        m = {}
        for line in r:
            parts = line.split(b':')
            name  = parts[0].lower().strip()
            name = name.decode()
            count = int(parts[1])
            for n in range(0, count):
                child_class = io_board_py_t.__PROP_MAP[name]
                child = child_class(n + 1, self)
                children = getattr(self, name)
                children += [child]

        cal_map = type(self)._ADC_CORRECTION_MAP

        for adc_name, adc_adj in cal_map.items():
            if adc_name in type(self).NAME_MAP:
                adc = getattr(self, adc_name)
                adc.adc_scale  = adc_adj[0]
                adc.adc_offset = adc_adj[1]



    def __getattr__(self, item):
        if item in type(self).NAME_MAP:
            return type(self).NAME_MAP[item](self)
        raise AttributeError("Attribute %s not found" % item)

    def _read_line(self):
        line = self.comm.readline().rstrip()
        debug_print(b">> : %s" % line)
        return line

    def _read_response(self):
        line = self._read_line()
        start = time.time()
        while line != type(self).__LOG_START_SPACER:
            if (time.time() - start) > type(self).__READTIME:
                raise ValueError("Comms read took too long.")
            line = self._read_line()

        line = self._read_line()
        data_lines = []

        while line != type(self).__LOG_END_SPACER:
            if (time.time() - start) > type(self).__READTIME:
                raise ValueError("Comms read took too long.")
            data_lines += [line]
            line = self._read_line()

        return data_lines

    def command(self, cmd):
        for c in cmd:
            self.comm.write(c.encode())
            time.sleep(type(self).__WRITEDELAY)
        self.comm.write(b'\n')
        self.comm.flush()
        debug_print("<< " + cmd)
        return self._read_response()
