import re
import os
import struct
from dataclasses import dataclass
from typing import Tuple, Union
from PyQt5.QtSerialPort import QSerialPort
from dacite import from_dict
from proto import flapjack_pb2

@dataclass
class Conf:
    
    PID_MIN_VALUE: float = 0.0
    PID_MAX_VALUE: float = 5.0
    PID_STARTING_ROLL_P: float = 0.2
    PID_STARTING_ROLL_I: float = 0.3
    PID_STARTING_ROLL_D: float = 0.05
    PID_STARTING_PITCH_P: float = 0.2
    PID_STARTING_PITCH_I: float = 0.3
    PID_STARTING_PITCH_D: float = 0.05
    PID_STARTING_YAW_P: float = 0.3
    PID_STARTING_YAW_I: float = 0.05
    PID_STARTING_YAW_D: float = 0.00015
    PID_STARTING_INTEGRAL_LIMIT: float = 25.0

    CFG_MOTOR_MIN_THROTTLE: float = 0.20
    CFG_MOTOR_MAX_THROTTLE: float = 0.40
    CFG_MOTOR_STARTUP_THROTTLE: float = 0.25

    # CMD_TOTAL_SIZE_BYTES: int
    # CMD_HEADER_SIZE_BYTES: int
    # CMD_DATA_SIZE_BYTES: int
    # eCMD_FLIGHT_MODE_HOVER: int
    # eCMD_FLIGHT_MODE_AIRPLANE: int
    # eCMD_OP_STATE_STOPPED: int
    # eCMD_OP_STATE_RUNNING: int
    # eCMD_OP_STATE_ERROR: int
    # eNUMBER_OF_OP_STATES: int
    # eCMD_TYPE_EMPTY: int
    # eCMD_TYPE_CHANGE_OP_STATE: int
    # eCMD_TYPE_CHANGE_FLIGHT_MODE: int
    # eCMD_TYPE_CHANGE_VELOCITY: int
    # eCMD_TYPE_CHANGE_PID: int
    # eNUMBER_OF_CMD_TYPES: int
    # eCMD_PID_ROLL: int
    # eCMD_PID_PITCH: int
    # eCMD_PID_YAW: int
    # eCMD_PID_THROTTLE: int

    CMD_TYPE_VELOCITY_CHANGE_MIN: int = -100
    CMD_TYPE_VELOCITY_CHANGE_MAX: int = 100

    LOG_DATA_TYPE_ATTITUDE: str =     "attitude"
    LOG_DATA_TYPE_PID_ATTITUDE: str = "pid_attitude"
    LOG_DATA_TYPE_IMU_CALIB: str =    "imu_calib"
    LOG_DATA_TYPE_IMU_DATA: str =     "imu_data"
    LOG_DATA_TYPE_RAW_IMU_DATA: str = "imu_raw_data"
    LOG_DATA_TYPE_ACTUATORS: str =    "actuators"
    LOG_DATA_TYPE_DEBUG: str =        "debug"

def resolve_values(config: dict) -> dict:
    """
    Given a dict of {name: value}, where value may be a number or a string
    referring to other keys, try to resolve all references into numbers.
    """
    resolved = {}

    def resolve_value(val, seen=None):
        if seen is None:
            seen = set()
        if isinstance(val, (int, float)):
            return val
        if not isinstance(val, str):
            return val

        # Avoid circular references
        expr = val.strip()
        if expr in seen:
            return expr
        seen.add(expr)

        # Replace macro names with resolved numeric values
        tokens = re.split(r"(\W+)", expr)  # split but keep delimiters
        for i, t in enumerate(tokens):
            if t in config:
                tokens[i] = str(resolve_value(config[t], seen))
        expr_resolved = "".join(tokens)

        # Try evaluating the math expression
        try:
            return eval(expr_resolved, {"__builtins__": None}, {})
        except Exception:
            return expr_resolved  # leave as string if not evaluatable

    for k, v in config.items():
        resolved[k] = resolve_value(v)

    return resolved


def parse_enums(raw_text: str):
    enums = {}
    enum_pattern = re.compile(r"enum\s*\{([^}]*)\}", re.MULTILINE | re.DOTALL)
    for block in enum_pattern.finditer(raw_text):
        enum_body = block.group(1)
        value = 0
        # Split by comma, remove empty entries
        entries = [e.strip() for e in enum_body.split(",") if e.strip()]
        
        for entry in entries:
            if '=' in entry:
                name, val_str = map(str.strip, entry.split("=", 1))
                value = int(val_str, 0)
            else:
                name = entry
            enums[name] = value
            value += 1
    return enums

def parse_defines(raw_text: str):
   
    defines = {}
    # Match #define NAME value (until newline or next #define)
    pattern = re.compile(r"#define\s+(\w+)\s+([^\n#]+)")
    for block in pattern.finditer(raw_text): #re.findall(pattern, raw_text):
        name = block.group(1)
        value = block.group(2).strip()

        # Remove surrounding parentheses
        if value.startswith("(") and value.endswith(")"):
            value = value[1:-1].strip()

        # If it's a quoted string → keep as str
        if (value.startswith('"') and value.endswith('"')) or \
        (value.startswith("'") and value.endswith("'")):
            defines[name] = value.strip('"').strip("'")
            continue

        # Remove F or f suffix from floats
        if value.upper().endswith("F"):
            value = value[:-1]
        
        # Remove U or u suffix from floats
        if value.upper().endswith("U"):
            value = value[:-1]

        # Convert to number if possible
        try:
            if "." in value or "e" in value.lower():
                value = float(value)
            else:
                value = int(value, 0)
        except ValueError:
            value = value.strip('"')  # maybe string literal

        defines[name] = value
    return defines

def parse_header(header_path):
    assert os.path.exists(header_path), f"Header file {header_path} does not exist"
    assert os.path.isfile(header_path), f"Path {header_path} is not a file"
    parsed_conf = {}
    with open(header_path) as f:
        text = f.read()
        # Remove C-style /* comments */ and // comments
        text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
        text = re.sub(r"//.*", "", text)

        defines = parse_defines(text)
        enums = parse_enums(text)
        parsed_conf.update(defines)
        parsed_conf.update(enums)

    return parsed_conf

def conf_init(
        header_files = ("Common/Inc/conf/conf.h", "Common/Inc/control.h", "Common/Inc/log/logger.h")
        ) -> Conf:
    
    conf = {}
    # for header in header_files:
    #     conf.update(parse_header(header))
    # conf = resolve_values(conf)

    # Dacite supports following features:
    # nested structures
    # (basic) type checking
    # optional fields (i.e. typing.Optional)
    # unions
    # generics
    # forward references
    # collections
    # custom type hooks
    # case conversion

    # return from_dict(data_class=Conf, data=conf)

    return Conf()

WRITE_CMD_RETURN_TYPE = Tuple[bool, Union[None, str]]

def write_cmd_packet(serial: QSerialPort, conf: Conf, packet: bytes) -> WRITE_CMD_RETURN_TYPE:
    # assert len(packet) == 8, "PID command packet must be exactly 8 bytes"
    if serial.isOpen() is False:
        return False, "Serial port is not open"
    try:
        serial.write(packet)
        return serial.flush(), None
    except Exception as e:
        print(f"Error writing command packet: {e}")
        return False, str(e)

def write_op_state_packet(serial: QSerialPort, conf: Conf, requested_state: int) -> WRITE_CMD_RETURN_TYPE:
    """
    Prepare a ChangeOpState command packet to send to the drone.
    """
    # TODO: get id for change op state command from conf
    # COMMAND_TYPE_CHANGE_OP_STATE = CONF.eCMD_TYPE_CHANGE_OP_STATE
    COMMAND_TYPE_CHANGE_OP_STATE = 0
    REQUESTED_STATE_START = requested_state

    # Create 8-byte packet: commandType(1) + requestedState(1) + padding(6)
    # B = uint8_t and 6x = 6 bytes of padding
    packet = struct.pack('<BB6x', COMMAND_TYPE_CHANGE_OP_STATE, REQUESTED_STATE_START)
    return write_cmd_packet(serial, conf, packet)

write_op_state_start = lambda serial, conf: write_op_state_packet(serial, conf, requested_state=1)
write_op_state_stop  = lambda serial, conf: write_op_state_packet(serial, conf, requested_state=0)

def write_velocity_packet(
        serial: QSerialPort,
        conf: Conf,
        forward: int,
        right: int,
        throttle: int
    ) -> WRITE_CMD_RETURN_TYPE:

    COMMAND_TYPE_VELOCITY = 2
    v_min = conf.CMD_TYPE_VELOCITY_CHANGE_MIN
    v_max = conf.CMD_TYPE_VELOCITY_CHANGE_MAX

    v_forward = max(v_min, min(v_max, forward))
    v_right = max(v_min, min(v_max, right))
    v_throttle = max(v_min, min(v_max, throttle))

    packet = struct.pack('<Bbbb4x', COMMAND_TYPE_VELOCITY, v_throttle, v_right, v_forward)
    return write_cmd_packet(serial, conf, packet)

def write_pid_packet(
        serial: QSerialPort,
        conf: Conf,
        axis: str,
        p_value: float,
        i_value: float,
        d_value: float
    ) -> WRITE_CMD_RETURN_TYPE:

    COMMAND_TYPE_PID_UPDATE = 4 # conf.eCMD_TYPE_CHANGE_PID
        
    axis_map = {
        "roll": 0,
        "pitch": 1,
        "yaw": 2
    }
    
    axis_code = axis_map.get(axis, 0)
    
    # Convert PID values (0.0 to 5.0) to uint16_t (0 to 65535)
    def map_range(value, in_min, in_max, out_min, out_max):
        return int(round((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, 0))

    p_min, p_max = conf.PID_MIN_VALUE, conf.PID_MAX_VALUE
    p_uint16 = map_range(p_value, p_min, p_max, 0, 65535)
    i_uint16 = map_range(i_value, p_min, p_max, 0, 65535)
    d_uint16 = map_range(d_value, p_min, p_max, 0, 65535)
    # Create 8-byte packet: commandType(1) + PID Axis(1) P(2) + I(2) + D(2)
    # H = uint16_t (2 bytes each)
    packet = struct.pack('<BBHHH4x', COMMAND_TYPE_PID_UPDATE, axis_code, p_uint16, i_uint16, d_uint16)
    return write_cmd_packet(serial, conf, packet)


_AXIS_MAP = {
    "roll":     flapjack_pb2.ROLL,
    "pitch":    flapjack_pb2.PITCH,
    "yaw":      flapjack_pb2.YAW,
    "throttle": flapjack_pb2.THROTTLE,
}

_GAIN_MAP = {
    "kp":             flapjack_pb2.KP,
    "ki":             flapjack_pb2.KI,
    "kd":             flapjack_pb2.KD,
    "integral_limit": flapjack_pb2.INTEGRAL_LIMIT,
}

def write_protobuf_command(serial: QSerialPort, cmd: flapjack_pb2.Command) -> WRITE_CMD_RETURN_TYPE:
    if not serial.isOpen():
        return False, "Serial port is not open"
    payload = cmd.SerializeToString()
    frame = bytes([len(payload)]) + payload
    try:
        serial.write(frame)
        return serial.flush(), None
    except Exception as e:
        return False, str(e)

def write_set_pid_cmd(serial: QSerialPort, axis: str, gain: str, value: float) -> WRITE_CMD_RETURN_TYPE:
    cmd = flapjack_pb2.Command()
    cmd.set_pid.axis = _AXIS_MAP[axis]
    cmd.set_pid.gain = _GAIN_MAP[gain]
    cmd.set_pid.value = value
    return write_protobuf_command(serial, cmd)

def write_arm_cmd(serial: QSerialPort, arm: bool) -> WRITE_CMD_RETURN_TYPE:
    cmd = flapjack_pb2.Command()
    cmd.arm_disarm.arm = arm
    return write_protobuf_command(serial, cmd)


if __name__ == "__main__":
    filenames = ("Common/Inc/conf/conf.h", "Common/Inc/control.h", "Common/Inc/log/logger.h")
    # filenames = ("Common/Inc/log.h",)
    conf = conf_init(filenames)
    print(conf)