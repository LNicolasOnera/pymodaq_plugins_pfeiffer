"""
omnicontrol_driver.py - Read and control a Pfeiffer Vacuum OmniControl over RS-458

Implements the "Pfeiffer Vacuum protocol for RS-458 interface" as documented 
in the OmniControl Communication Guid.

Serial line settings are fixed by the protocol:
    9600 baud, 8 data bits, no parity, 1 stop bit.

Telegram frame:
    
    a2 a1 a0  * 0  n2 n1 n0  l1 l0    dn...d0  c2 c1 c0  CR
    \_____/   \_/  \____/     \_/     \____/    \____/
    address   act  param.No  length    data    checksum

    The action digit is either '0' (read) or '1' (write)
    
IMPORTANT:
   * The OmniControl should be in "device mode" for this code to work.
   => (4 rectangles in the bottom left corner) -> settings -> RS-458 -> device mode
   (You can also check that the default address of the OmniControl is still '101')

   * The OmniControl class should always be closed after its usage in order to
   free the port for later usage (for example: 'with OmniControl(PORT) as dev:').
    
Requires: pyserial
"""

import serial
import math
from dataclasses import dataclass

CR = b'\r'
ERROR_PAYLOADS = {"NO_DEF", "_RANGE", '_LOGIC'}

class PfeifferError(Exception):
    """Raised on a protocol-level error reply or a malformed/failed telegram"""
    
    
# --------------------------------------------------------------------------------------------- #
# Data types
# --------------------------------------------------------------------------------------------- #

# Each entry: (length, decode(raw_str), decode(raw_str) -> value, encode(value) -> raw_str)

def _enc_bool_old(v):   return "111111" if v else "000000"
def _dec_bool_old(s):   return s == "111111"

def _enc_uint(v):       return f"{int(v):06d}"
def _dec_uint(s):       return int(s)

def _enc_ureal(v):      return f"{int(round(float(v) * 100)):06d}"
def _dec_ureal(s):      return int(s) / 100.0

def _enc_str(v):        return f"{str(v):<6.6s}"
def _dec_str(s):        return s.strip()

def _enc_bool_new(v):   return "1" if v else "0"
def _dec_bool_new(s):   return s == "1"

def _enc_ushort(v):     return f"{int(v):03d}"
def _dec_ushort(s):     return int(s)

def _enc_str16(v):      return f"{str(v):<16.6s}"

def _enc_str8(v):       return f"{str(v):<8.8s}"

def _enc_expo(v):
    v = float(v)
    
    if v <= 0: return "000000"
    
    exponent = math.floor(math.log10(v))
    mantissa = round(v / (10.0 ** exponent) * 1000)
    if mantissa >= 10000:
        mantissa //= 10 
        exponent += 1
    code = exponent + 20
    if not 0 <= code  <= 99:
        raise PfeifferError(f"value {v!r} out of u_expo_new range")
    return f"{mantissa:04d}{code:02d}"

def _dec_expo(s):
    # "100023" -> mantissa 1000 (=> 1.000), exponent 23-20 = 3 => 1.0e3
    mantissa = int(s[:4]) / 1000.0
    exponent = int(s[4:]) - 20
    return mantissa * (10.0 ** exponent)

@dataclass(frozen=True)
class DataType:
    code: int
    length: int
    decode: callable 
    encode: callable 
    name: str
    
DATA_TYPES = {
    0: DataType(0,  6, _dec_bool_old, _enc_bool_old, "boolean_old"),
    1: DataType(1, 6, _dec_uint, _enc_uint, "u_integer"),
    2: DataType(2,6, _dec_ureal, _enc_ureal, "u_real"),
    4: DataType(4,6, _dec_str, _enc_str, "string"),
    6: DataType(6,1, _dec_bool_new, _enc_bool_new, "boolean_new"),
    7: DataType(7,3, _dec_ushort, _enc_ushort, "u_short_int"),
    10: DataType(10, 6, _dec_expo, _enc_expo, "u_expo_new"),
    11: DataType(11, 16, _dec_str, _enc_str16, "string16"),
    12: DataType(12, 8, _dec_str, _enc_str8, "string8")
}
   
# --------------------------------------------------------------------------------------------- #
# Parameter registery
# --------------------------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Parameter:
    number: int
    type_code: int 
    access: str
    unit: str  
    description: str    
    
PARAMETERS: dict[str, Parameter] = {
    # --- Control commands ---
    "DeGas":        Parameter(40, 6, "RW", "", "Clean measuring element"),
    "Sens OnOff":   Parameter(41, 7, "RW", "", "Switch BA/CC or the cold cathode on/off"),
    "DirDigOut":    Parameter(70, 1, "RW", "", "Digital outputs (per-digit set/reset/keep)"),
    "DirRelOut":    Parameter(71, 1, "RW", "", "Relay outputs (per-digit set/reset/keep"),
    # --- Status requests ---
    "ErrorCode":    Parameter(303, 4, "R", "", "Error code"),
    "FWversion":    Parameter(312, 4, "R", "", "Firmware Version"),
    "ElecName":     Parameter(349, 4, "R", "", "Device designation"),
    "HWversion":    Parameter(352, 4, "R", "", "Hardware version"),
    "SerialNo":     Parameter(355, 11, "R", "", "Serial number"),
    "DirDigInp":    Parameter(386, 1, "R", "", "Digital inputs (set/reset/keep)"),
    "DirAlgInp":    Parameter(387, 2, "R", "", "Analog input"),
    "OrderCode":    Parameter(388, 11, "R", "", "Order number"),
    # --- Set value settings ---
    "DirAlgOut":    Parameter(727, 2, "RW", "V", "Analog output"),
    "Pressure":     Parameter(740, 10, "RW", "hPa", "Pressure value"),
    "UserGasCor":   Parameter(742, 2, "RW", "", "Correction factor"),
    "BaseAdr":      Parameter(797, 1, "RW", "", "Interface address"),
}

# --------------------------------------------------------------------------------------------- #
# Telegram building / parsing
# --------------------------------------------------------------------------------------------- #

def _checksum(payload: str) -> str:
    return f"{sum(ord(c) for c in payload)%256:03d}"

def build_telegram(address: int, action: str, number: int, data:str) -> bytes:
    """Assemble one telegram."""
    if not 0 <= address <= 999:
        raise ValueError("address must be between 0 and 999")
    if action not in ("0", "1"):
        raise ValueError("action must be '0' (read) or '1' (write)")
    body = f"{address:03d}{action}0{number:03d}{len(data):02d}{data}"
    return (body + _checksum(body)).encode("ascii") + CR

def parse_response(raw: bytes) -> tuple[int, int, str]:
    """Validate a reply and return (address, parameter_number, data_field)."""
    if not raw:
        raise PfeifferError("no response (timeout) - check wiring, address, baud rate")
    text = raw.rstrip(b"\r\n").decode("ascii", errors="replace")
    received_cksum = text[-3:]
    body = text[:-3]

    if int(_checksum(body)) != int(received_cksum):
        raise PfeifferError(f"checksum mismatch in reply: {text!r}")
    
    address = int(body[:3])
    number = int(body[5:8])
    length = int(body[8:10])
    data = body[10:10+length]
    
    if data in ERROR_PAYLOADS:
        msg = {
            "NO_DEF": "parameter does not exist",
            "_RANGE": "value out of permissible range",
            "_LOGIC": "logical access error (e.g. read-only parameter written)"
        }[data]
        raise PfeifferError(f"device error on parameter {number}: {data} ({msg})")
    
    return address, number, data

# --------------------------------------------------------------------------------------------- #
# Main driver
# --------------------------------------------------------------------------------------------- #
class Omnicontrol:
    """Talk to OmniControl or a unit on its RS-485 bus"""
    
    def __init__(self, port: str, address: int = 101, timeout: float = 0.5):
        self.address = address
        self.ser = serial.Serial(
                port=port,
                baudrate=9600,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout,
            )
    
    def _transact(self, action:str, number: int, data:str, address: int | None = None) -> str | None:
        """Low level interaction with the device"""
        addr = self.address if address is None else address
        telegram = build_telegram(addr, action, number, data)
        self.ser.reset_input_buffer()
        self.ser.write(telegram)
        self.ser.flush()
        # Group ("9xx") and global ("100") addresses are not answered
        if addr == 0 or 900 <= addr <= 999:
            return None
        reply = self.ser.read_until(CR)
        _, _, field = parse_response(reply)
        return field
    
    # --- generic read / write ---
    
    def read(self, parameter, address: int | None = None):
        """Read a parameter, a parameter is either a registery name or (number, type_code)"""
        number, data_type = self._resolve(parameter)
        raw = self._transact('0', number, '=?', address)
        return data_type.decode(raw)
    
    def write(self, parameter, value, address: int | None = None):
        """Write a parameter, a parameter is either a registery name of (number, type_code)"""
        number, data_type = self._resolve(parameter)
        raw = self._transact("1", number, value, address)
        return data_type.decode(raw) if raw is not None else None
        
    @staticmethod
    def _resolve(parameter) -> tuple[int, DataType]:
        if isinstance(parameter, str):
            if parameter not in PARAMETERS:
                raise KeyError(f"unknown parameter: {parameter!r};"
                               f"use one of {sorted(PARAMETERS)}"
                               f"or pass (number, type_code)")
            p = PARAMETERS[parameter]
            return p.number, DATA_TYPES[p.type_code]
        number, type_code = parameter
        if type_code not in DATA_TYPES:
            raise ValueError("unknown data type code {type_code}")
        return int(number), DATA_TYPES[type_code]
    
    # --- convenience for documented parameters ---
    
    def read_pressure(self, address=None) -> float:
        """Read pressure in hPa (mbar)."""
        pressure = self.read("Pressure", address)
        if pressure > 2000:
            raise OverflowError("the pressure is outside of the gauge range")
        return pressure
    
    def read_error_code(self, address=None) -> str:
        return self.read("ErrorCode", address)
    
    def read_firmware(self, address=None) -> str:
        return self.read("FWversion", address)
    
    def read_name(self, address=None) -> str:
        return self.read("ElecName", address)
    
    def read_serial(self, address=None) -> str:
        return self.read("SerialNo", address)
    
    def device_info(self, address=None) -> str:
        info = {}
        for key, method in (("name", self.read_name),
                            ("firmware", self.read_firmware),
                            ("hardware", lambda a=address: self.read("HWversion", a)),
                            ("serial", self.read_serial),
                            ("order_code", lambda a=address: self.read("OrderCode", a)),
                            ("error", self.read_error_code)):
            try:
                info[key] = method(address) if callable(method) else method
            except PfeifferError as exc:
                info[key] = f"<{exc}>"
        return str(info)
    
    # --- context manager / cleanup ---
    
    def close(self):
        self.ser.close()
    
    def __enter__(self):
        return self
    
    def __exit__(self, *exc):
        self.close()