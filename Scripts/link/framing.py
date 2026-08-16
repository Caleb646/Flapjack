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
    """Incremental deframer mirroring the FC RX state machine.

    The firmware interleaves two streams on one wire: framed binary, and raw
    ASCII debug log text. Bytes outside a frame are log text and nothing else -
    log output is 7-bit ASCII, so it can never contain the 0xAA that opens a
    frame. They accumulate in `text`; drain it with take_text().

    Bytes belonging to a frame that fails CRC are discarded, not treated as
    text: they were framed, they were just corrupt.
    """

    def __init__(self):
        self.buf = bytearray()
        self.text = bytearray()

    def take_text(self) -> bytes:
        """Return and clear the non-frame (log) bytes seen so far."""
        out = bytes(self.text)
        self.text.clear()
        return out

    def feed(self, data: bytes):
        self.buf += data
        while True:
            # find magic
            i = self.buf.find(b"\xAA\x55")
            if i < 0:
                # Keep a trailing 0xAA in case the magic is split across reads;
                # it cannot be text. Everything before it is.
                if self.buf[-1:] == b"\xAA":
                    self.text += self.buf[:-1]
                    self.buf = self.buf[-1:]
                else:
                    self.text += self.buf
                    self.buf = bytearray()
                return
            if i:
                self.text += self.buf[:i]
                self.buf = self.buf[i:]
            if len(self.buf) < 4:
                return
            msg_id = self.buf[2]
            length = self.buf[3]
            end = 4 + length + 1
            if len(self.buf) < end:
                return
            payload = bytes(self.buf[4 : 4 + length])
            crc = self.buf[4 + length]
            self.buf = self.buf[end:]
            if crc8(bytes([msg_id, length]) + payload) == crc:
                yield msg_id, payload
