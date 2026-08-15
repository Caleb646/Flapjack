"""Sim-link wire framing, shared by the HIL bridge and any host-side tool that
speaks the firmware's binary sim protocol.

Wire frame:  [0xAA][0x55][msg_id][len][payload][crc8]
    crc8: poly 0x07, init 0x00, over (msg_id, len, payload).

Mirrors Firmware/drivers/sim_link/sim_link.c.
"""

MAGIC0 = 0xAA
MAGIC1 = 0x55


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def frame(msg_id: int, payload: bytes) -> bytes:
    body = bytes([msg_id, len(payload)]) + payload
    return bytes([MAGIC0, MAGIC1]) + body + bytes([crc8(body)])


class FrameParser:
    """Incremental deframer mirroring the FC RX state machine."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf += data
        while True:
            # find magic
            i = self.buf.find(b"\xAA\x55")
            if i < 0:
                # keep at most one trailing byte (possible split magic)
                self.buf = self.buf[-1:]
                return
            if len(self.buf) < i + 4:
                self.buf = self.buf[i:]
                return
            msg_id = self.buf[i + 2]
            length = self.buf[i + 3]
            end = i + 4 + length + 1
            if len(self.buf) < end:
                self.buf = self.buf[i:]
                return
            payload = bytes(self.buf[i + 4 : i + 4 + length])
            crc = self.buf[i + 4 + length]
            self.buf = self.buf[end:]
            if crc8(bytes([msg_id, length]) + payload) == crc:
                yield msg_id, payload
