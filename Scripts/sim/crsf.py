"""CRSF RC frame encoder — the host side of the SIL's RC link.

Emits exactly what a Crossfire receiver puts on the FC's RX UART, so the SIL
drives the real ``Rx_Task`` -> ``Crsf_ProcessFrame`` -> ``Rc_Update`` path instead
of writing ``g_Rx.channels`` behind its back.

Mirrors Firmware/drivers/rx/crsf.c and the TBS CRSF specification:

    [0xC8][len][0x16][22 bytes: 16 x 11-bit LSB-first][crc8]
    len  = type + payload + crc = 24
    crc8 = poly 0xD5 over (type, payload) - NOT the sim link's poly 0x07,
           and it covers neither the sync byte nor the length byte.

Channel conversion is the spec's, verbatim::

    TICKS_TO_US(x) = (x - 992) * 5 / 8 + 1500      # 992 ticks == 1500 us
    US_TO_TICKS(x) = (x - 1500) * 8 / 5 + 992

so 1000/1500/2000 us are ticks 192/992/1792.

Bit packing is **LSB first**, which is what a little-endian packed bitfield
struct produces — and ``crsf.c`` decodes by casting the payload straight to
``CrsfChannelsPayload_t``. Get this backwards and every channel is garbage, so
``Tests/UnitTest/test_crsf.c`` asserts the golden frame below byte for byte.
"""

SYNC_BYTE = 0xC8
FRAME_TYPE_RC_CHANNELS = 0x16

NUM_CHANNELS = 16
BITS_PER_CHANNEL = 11
RC_PAYLOAD_BYTES = 22

# Spec channel values. 172/1811 are what a transmitter actually swings between
# (988..2011 us); 192/992/1792 are exactly 1000/1500/2000 us.
TICKS_MIN = 172
TICKS_MID = 992
TICKS_MAX = 1811


def crc8(data: bytes) -> int:
    """CRSF CRC8, poly 0xD5. Covers frame type + payload only."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def us_to_ticks(us: float) -> int:
    """Pulse width (us) -> 11-bit channel value, clamped to a transmitter's range.

    Uses C's truncate-toward-zero division so the result matches the spec macro
    bit for bit on both sides of centre.
    """
    n = (int(us) - 1500) * 8
    q = -((-n) // 5) if n < 0 else n // 5
    return max(TICKS_MIN, min(TICKS_MAX, q + 992))


def ticks_to_us(ticks: int) -> int:
    """Inverse of :func:`us_to_ticks` — the firmware's view of a channel."""
    n = (int(ticks) - 992) * 5
    q = -((-n) // 8) if n < 0 else n // 8
    return q + 1500


def pack_channels(ticks) -> bytes:
    """16 x 11-bit values -> 22 bytes, LSB first."""
    if len(ticks) != NUM_CHANNELS:
        raise ValueError(f"need exactly {NUM_CHANNELS} channels, got {len(ticks)}")
    acc = 0
    for i, value in enumerate(ticks):
        if not 0 <= value < (1 << BITS_PER_CHANNEL):
            raise ValueError(f"channel {i} value {value} does not fit in {BITS_PER_CHANNEL} bits")
        acc |= value << (i * BITS_PER_CHANNEL)
    return acc.to_bytes(RC_PAYLOAD_BYTES, "little")


def frame(msg_type: int, payload: bytes) -> bytes:
    """Wrap a payload in [sync][len][type][payload][crc]."""
    length = len(payload) + 2  # type + payload + crc
    if not 2 <= length <= 62:
        raise ValueError(f"frame length {length} outside the spec's 2..62")
    body = bytes([msg_type]) + payload
    return bytes([SYNC_BYTE, length]) + body + bytes([crc8(body)])


def rc_frame(channels_us) -> bytes:
    """Build a 0x16 RC Channels frame from 16 pulse widths in microseconds."""
    return frame(FRAME_TYPE_RC_CHANNELS, pack_channels([us_to_ticks(us) for us in channels_us]))
