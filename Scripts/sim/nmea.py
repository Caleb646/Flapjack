"""NMEA 0183 sentence encoder — the host side of the SIL's GPS link.

Emits what a receiver puts on the FC's GPS UART, so the SIL drives the real
``Gps_DataReceivedHandler`` -> ``minmea`` -> ``Gps_Update_`` path instead of
injecting a decoded fix behind its back. Same reasoning that moved RC onto real
CRSF frames; see Scripts/sim/crsf.py.

Sentences emitted:

    $GPGGA,hhmmss.ss,ddmm.mmmmm,N,dddmm.mmmmm,E,q,nn,h.h,alt,M,0.0,M,,*CS
    $GPRMC,hhmmss.ss,A,ddmm.mmmmm,N,dddmm.mmmmm,E,knots,course,ddmmyy,,,A*CS

``CS`` is the XOR of every character between ``$`` and ``*``, exclusive.

**Five decimal places on the minutes field, not six.** minmea parses fields into
a fixed-point ``{int_least32_t value; int_least32_t scale}`` pair, so a longitude
of 179 deg at six decimals would need 17959.999999 -> value 17959999999, which
overflows int32. Five decimals peaks at 1795999999, just inside, and still
resolves ~1.9 cm. ``Tests/UnitTest/test_gps.c`` asserts the golden sentences
below against the firmware's parser so this file and minmea cannot drift apart.
"""

COORD_DECIMALS = 5
KNOTS_PER_MPS = 1.0 / 0.514444


def checksum(body: str) -> int:
    """XOR over the characters between '$' and '*'."""
    csum = 0
    for ch in body:
        csum ^= ord(ch)
    return csum


def _wrap(body: str) -> str:
    return f"${body}*{checksum(body):02X}\r\n"


def _coord(degrees: float, is_lat: bool) -> tuple:
    """Degrees -> (ddmm.mmmmm string, hemisphere char)."""
    hemi = ("N" if degrees >= 0 else "S") if is_lat else ("E" if degrees >= 0 else "W")
    degrees = abs(degrees)
    d = int(degrees)
    minutes = (degrees - d) * 60.0
    width = 2 if is_lat else 3
    # Zero-padded degrees then zero-padded minutes: ddmm.mmmmm / dddmm.mmmmm.
    return f"{d:0{width}d}{minutes:0{COORD_DECIMALS + 3}.{COORD_DECIMALS}f}", hemi


def coord_on_wire(degrees: float, is_lat: bool) -> float:
    """The value a sentence actually carries, after ddmm.mmmmm quantisation.

    Encoding to 5 decimal places on the minutes field loses ~1.9 cm, so the FC
    cannot echo back the degrees it was handed - it echoes what the wire could
    represent. This is that value, which is what a loopback check must compare
    against if it is to be tight rather than merely plausible.
    """
    coord, hemi = _coord(degrees, is_lat)
    width = 2 if is_lat else 3
    value = float(coord[:width]) + float(coord[width:]) / 60.0
    return -value if hemi in ("S", "W") else value


def gga(lat, lon, alt_m, sats=12, fix_quality=1, hhmmss="123519.00", hdop=0.9) -> str:
    """Position + altitude + satellite count. The FC's 3D-fix source."""
    lat_s, ns = _coord(lat, True)
    lon_s, ew = _coord(lon, False)
    body = (f"GPGGA,{hhmmss},{lat_s},{ns},{lon_s},{ew},{fix_quality},"
            f"{sats:02d},{hdop:.1f},{alt_m:.1f},M,0.0,M,,")
    return _wrap(body)


def rmc(lat, lon, speed_mps, course_deg, valid=True,
        hhmmss="123519.00", ddmmyy="230625") -> str:
    """Position + ground speed + course. Speed goes on the wire in KNOTS."""
    lat_s, ns = _coord(lat, True)
    lon_s, ew = _coord(lon, False)
    status = "A" if valid else "V"
    body = (f"GPRMC,{hhmmss},{status},{lat_s},{ns},{lon_s},{ew},"
            f"{speed_mps * KNOTS_PER_MPS:.3f},{course_deg:.2f},{ddmmyy},,,A")
    return _wrap(body)


def fix_sentences(lat, lon, alt_m, speed_mps, course_deg, sats=12) -> bytes:
    """One full update: GGA then RMC, as a receiver would send them."""
    return (gga(lat, lon, alt_m, sats) + rmc(lat, lon, speed_mps, course_deg)).encode("ascii")


if __name__ == "__main__":
    # Golden sentences - keep in step with Tests/UnitTest/test_gps.c.
    print(gga(37.6189, -122.3750, 100.0, sats=9), end="")
    print(rmc(37.6189, -122.3750, 5.0, 42.5), end="")
