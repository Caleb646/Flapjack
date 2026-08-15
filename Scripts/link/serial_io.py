"""Small pyserial helpers shared by the host-side tools (sim bridge, board test
reader). The GUI uses Qt's QSerialPort for its live link, but can still use
``list_ports`` for enumeration.
"""

import serial
import serial.tools.list_ports


def list_ports():
    """Return ``[(device, description), ...]`` for attached serial ports."""
    return [(p.device, p.description) for p in serial.tools.list_ports.comports()]


def open_port(port: str, baud: int, timeout: float = 0) -> serial.Serial:
    """Open a pyserial port. Raises ``serial.SerialException`` on failure.

    Accepts a device name (``COM7``, ``/dev/ttyUSB0``) or any pyserial URL. The
    URL form is what reaches an emulated target: Renode exposes the CM7's USART1
    as a TCP server, so the SIL link is ``socket://localhost:4000``.
    """
    return serial.serial_for_url(port, baudrate=baud, timeout=timeout)
