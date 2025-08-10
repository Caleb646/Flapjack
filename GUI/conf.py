import re
import os
from dataclasses import dataclass
from dacite import from_dict

@dataclass
class Conf:
    MS_PER_LOG_DATA_UPDATE: int
    PRIMARY_LOGGER_ROLE: str
    LOGGER_SHOULD_BLOCK_ON_OVERWRITE: int
    PID_MIN_VALUE: float
    PID_MAX_VALUE: float
    PID_STARTING_ROLL_P: float
    PID_STARTING_ROLL_I: float
    PID_STARTING_ROLL_D: float
    PID_STARTING_PITCH_P: float
    PID_STARTING_PITCH_I: float
    PID_STARTING_PITCH_D: float
    PID_STARTING_YAW_P: float
    PID_STARTING_YAW_I: float
    PID_STARTING_YAW_D: float
    PID_STARTING_INTEGRAL_LIMIT: float
    LEFT_MOTOR_PWM_TIMER: str
    LEFT_MOTOR_PWM_CHANNEL: str
    LEFT_MOTOR_DMA_STREAM: str
    LEFT_MOTOR_DMA_REQUEST: str
    LEFT_SERVO_PWM_TIMER: str
    LEFT_SERVO_PWM_CHANNEL: str
    LEFT_SERVO_PWM_FREQUENCY: int
    MOTOR_MAX_THROTTLE: float
    MOTOR_STARTUP_THROTTLE: float
    MOTOR_MIN_THROTTLE: float
    MOTOR_PID_ROLL_MIX: float
    MOTOR_PID_PITCH_MIX: float
    MOTOR_PID_YAW_MIX: float
    LEFT_MOTOR_PID_ROLL_MIX_DIR: float
    LEFT_MOTOR_PID_PITCH_MIX_DIR: float
    LEFT_MOTOR_PID_YAW_MIX_DIR: float
    RIGHT_MOTOR_PID_ROLL_MIX_DIR: float
    RIGHT_MOTOR_PID_PITCH_MIX_DIR: float
    RIGHT_MOTOR_PID_YAW_MIX_DIR: float
    SERVO_PID_ROLL_MIX: float
    SERVO_PID_PITCH_MIX: float
    SERVO_PID_YAW_MIX: float
    LEFT_SERVO_PID_ROLL_MIX_DIR: float
    LEFT_SERVO_PID_PITCH_MIX_DIR: float
    LEFT_SERVO_PID_YAW_MIX_DIR: float
    RIGHT_SERVO_PID_ROLL_MIX_DIR: float
    RIGHT_SERVO_PID_PITCH_MIX_DIR: float
    RIGHT_SERVO_PID_YAW_MIX_DIR: float
    COMMAND_TOTAL_SIZE: int
    COMMAND_HEADER_SIZE: int
    COMMAND_DATA_SIZE: int
    eCMD_FLIGHT_MODE_HOVER: int
    eCMD_FLIGHT_MODE_AIRPLANE: int
    eCMD_OP_STATE_STOPPED: int
    eCMD_OP_STATE_RUNNING: int
    eCMD_OP_STATE_ERROR: int
    eNUMBER_OF_OP_STATES: int
    eCMD_TYPE_EMPTY: int
    eCMD_TYPE_CHANGE_OP_STATE: int
    eCMD_TYPE_CHANGE_FLIGHT_MODE: int
    eCMD_TYPE_CHANGE_VELOCITY: int
    eCMD_TYPE_CHANGE_PID: int
    eNUMBER_OF_CMD_TYPES: int
    eCMD_PID_ROLL: int
    eCMD_PID_PITCH: int
    eCMD_PID_YAW: int
    eCMD_PID_THROTTLE: int
    LOG_DATA_TYPE_ATTITUDE: str
    LOG_DATA_TYPE_PID_ATTITUDE: str
    LOG_DATA_TYPE_IMU_CALIB: str
    LOG_DATA_TYPE_IMU_DATA: str
    LOG_DATA_TYPE_RAW_IMU_DATA: str
    LOG_DATA_TYPE_ACTUATORS: str
    LOG_DATA_TYPE_DEBUG: str = "debug"
    CMD_TYPE_VELOCITY_CHANGE_MIN: int = -100
    CMD_TYPE_VELOCITY_CHANGE_MAX: int = 100

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
        header_files = ("Common/Inc/conf.h", "Common/Inc/control.h", "Common/Inc/log.h")
        ) -> Conf:
    conf = {}
    for header in header_files:
        conf.update(parse_header(header))
    conf = resolve_values(conf)
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
    return from_dict(data_class=Conf, data=conf)

if __name__ == "__main__":
    filenames = ("Common/Inc/conf.h", "Common/Inc/control.h", "Common/Inc/log.h")
    # filenames = ("Common/Inc/log.h",)
    conf = conf_init(filenames)
    print(conf)