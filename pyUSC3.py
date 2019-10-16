"""
control for NuQ 50W Marking Laser
via SCAPS hardware interface over serial

communication protocol:
https://www.nufern.com/filestorage/fiber_lasers/NUQA_1064_NA_0050_C1_manual.pdf

required modules:
- serial: pip install pyserial

Created on 07.08.2019

@author: johnathon.barhydt


Laser Notes:
    NuQA 50W Laser
    M2: 1.5 optimal
    lambda: 1064nm
    warm-up: 60 sec
    PRR: 20-200kHz (50kHz default)
    pulse: 80-500 n sec

Initialization Process:
    Apply 24 V to DC power input
    Apply  5 V to INTERLOCK PIN on DB25 (PIN 23, BRN, INT12)
    (optional) 5 v to AUX input on DB25 (PIN 17, YLW) for pointer power
    Connect GATE to BNC (PORT #3) or bridge GATE on DB25 (PINS 18/19, ONG RED, OSC10, AMP11)
    Connect RS-232 to host via DB9 cable
Operation Process (pointer must be OFF):
    Supply 24V power
    Send b'GSTA\r\n' until a response appears
    Send b'S_232\r\n' to activate RS-232 Control Mode
    Send b'PRR xx\r\n' to set frequency to xx kHz
    Send b'POW xx\r\n' to set power to xx% of maximum
    Send b'NUQON\r\n' to enable laser output
    Bring GATE signal to 5v HIGH to fire the laser (can simultaneously send to EXT TRIGGER of camera)
    Bring GATE signal to LOW to turn off laser amplifier
    Send b'NUQOFF\r\n' to disable laser power output
"""

import serial
from serial.tools import list_ports

SCAPS_BAUD_RATE = 115200


class Usc3(object):
    """
    SCAPS laser control object via direct serial comm to NuQ Laser
    """
    _trm = b'\r\n'
    _dlm = b' '
    _end = b'\n'

    comm_codes = {
        '0': 'Success',
        '1': 'Too many parameters',
        '2': 'Too few parameters',
        '3': 'Unknown command',
        '4': 'Job not valid',
        '5': 'Operation not allowed during marking',
        '6': 'Entity not found',
        '7': 'Not initialized',
        '8': 'Parameter out of range',
        '9': 'Internal error',
        '10': 'Operation not allowed',
        '11': 'The internal queue is full and cannot accept any more commands',
        '12': 'Command not available, probably due to missing Flash license',
        '13': 'Out of memory',
        '14': 'Job already exists',
        '15': 'File not found',
        '16': 'Command removed',
        '17': 'Wrong parameter type',
        '18': 'SN increment must be 0'
    }

    def __init__(self, serial_number: str = None, port: str = None):
        self.serial_number = serial_number
        self.port = port
        if port is None:
            self.serial = self.connect()
        else:
            self.serial = serial.Serial(port=port, baudrate=SCAPS_BAUD_RATE)

    def _port_seek(self):
        comports = list_ports.comports()
        devices = [{'PORT': port.device, 'VID': port.vid, 'SN': port.serial_number} for port in comports]
        if self.serial_number is None:
            port_list = [device['PORT'] for device in devices if device['VID'] == 0x2341]
        else:
            port_list = [device['PORT'] for device in devices if device['SN'] == self.serial_number]
        return port_list[-1]

    def _send(self, byte_string):
        return self.serial.write(byte_string)

    def _recv(self):
        return self.serial.read_until(self._end)

    def _execute_command(self, data: bytes, val: int = None):
        to_write = data
        if val is not None:
            to_write += self._dlm + bytes(str(val)[1:-1].replace(',', ''), 'Ascii')
        to_write += self._trm
        self._send(to_write)
        response = self._recv()
        if response in self.comm_codes:
            return self.comm_codes[response]
        else:
            return response

    def connect(self):
        usc3_port = self._port_seek()
        if len(usc3_port) == 0:
            print("USC-3 Laser Control Device Not Found.")
            return None
        else:
            self.port = usc3_port
            usc3_serial = serial.Serial(port=self.port, baudrate=SCAPS_BAUD_RATE)
            return usc3_serial

    def get_mark_time(self):
        return self._execute_command(b'GTI 0')

    def save(self):
        return self._execute_command(b'SVG')

    def load(self):
        return self._execute_command(b'LDG')

    def get_temperature(self):
        return self._execute_command(b'GUT')

    def get_home_position(self):
        return [int(n) for n in self._execute_command(b'HP').split(' ')]

    def set_home_position(self, vector):
        return self._execute_command(b'HP', vector)

    def get_digital_out(self):
        return self._execute_command(b'OOF')

    def set_digital_out(self, binary_value):
        return self._execute_command(b'OOF', binary_value)

    def get_digital_in(self):
        return self._execute_command(b'OIF')

    def get_analog_in(self, address):
        return self._execute_command(b'AIN', address)

    def get_pulse(self):
        return self._execute_command(b'LPJ')

    def set_pulse(self, output_list, pulse_width):
        """
        Input pin       Output pin   Bit value
        =====================================
        Opto-insulated Inputs and Outputs
        OptoIn_0 [a]    OptoOut_0 [c]   1
        OptoIn_1 [b]    OptoOut_1       2
        OptoIn_2        OptoOut_2 [d]   3
        OptoIn_3        OptoOut_3       4
        OptoIn_4        OptoOut_4       5
        OptoIn_5        OptoOut_5       6


        Digital Inputs and Outputs

        DigiIn_0        DigiOut_0       7
        DigiIn_1        DigiOut_1       8
        DigiIn_2        DigiOut_2       9
        DigiIn_3        DigiOut_3       10
        DigiIn_4        DigiOut_4       11
        DigiIn_5        DigiOut_5       12
        DigiIn_6        DigiOut_6       13
        DigiIn_7        DigiOut_7       14
        DigiIn_8        DigiOut_8       15
        DigiIn_9        DigiOut_9       16

        Stepper Inputs and Outputs

        SmIn_0          SmOut_0         17
        SmIn_1          SmOut_1         18
        SmIn_2          SmOut_2         19
                        SmOut_3         20
                        SmOut_4         21
                        SmOut_5         22

        [a]: Reserved for trigger start
        [b]: Reserved for external stop
        [c]: Reserved for marking active
        [d]: Only reserved for the red pointer, if the red pointer is active

        :param output_list: list of pins to send pulse
        :param pulse_width: pulse time in us
        :return:
        """
        pins = sum([2**(pin-1) for pin in output_list])
        return self._execute_command(b'LPJ', pins)

    def run(self):
        return self._execute_command(b'M', 1)

    def stop(self):
        return self._execute_command(b'M', 0)

    def get_loop_count(self):
        return self._execute_command(b'LC')

    def set_loop_count(self, num_loops: int = 1):
        return self._execute_command(b'AC', num_loops)

    def set_laser_disable_flag(self, input_pin: int = 6, trigger_mode: int = 0x10):
        """
        Input Bit Value
        2 ... 5     OptoIn_2..5
        6 ... 15    DigiIn_0..9
        TM flag
        0           level triggered = 0
        0x10        edge triggered = 1
        :param input_pin:
        :param trigger_mode: 0x00 for level, 0x10 for edge-trigger
        :return:
        """
        return self._execute_command(b'FLAGS', input_pin+trigger_mode)
