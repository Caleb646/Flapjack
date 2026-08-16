# Flapjack tests

Two independent suites that share nothing but the vendored Unity in `Tests/unity/`.

| | `Tests/UnitTest/` | `Tests/HIL/` |
|---|---|---|
| Runs on | the host (x86) | the board |
| Built by | `Tests/CMakeLists.txt` (standalone CMake project) | the firmware build, `board.py build -f t` |
| Purpose | pure logic against controlled inputs | drivers against real peripherals |

---

## Host unit tests

```bash
cmake -S Tests -B Build/UnitTest -G "MinGW Makefiles"   # "Unix Makefiles" on Linux
cmake --build Build/UnitTest
ctest --test-dir Build/UnitTest --output-on-failure
```

No toolchain and no network needed — this compiles firmware sources with the host compiler
against `UnitTest/stubs/hal_stub.c`, and Unity is vendored rather than fetched.

### Current contents

`test_crsf.c` only. It covers the receiver path in `drivers/rx/crsf.c` against the TBS CRSF
specification: frame layout and the 2–62 length range, CRC8 (poly 0xD5, over type + payload),
the 16 × 11-bit LSB-first channel unpack, the `TICKS_TO_US` mapping, and the spec's rule that a
longer-than-expected frame must be accepted with its extra fields ignored. It also asserts a
**golden frame** produced by `Scripts/sim/crsf.py` byte for byte, so the host encoder used by the
SIL and the firmware decoder cannot drift apart.

### Adding a test

Add the source under `UnitTest/`, then in `Tests/CMakeLists.txt` put its firmware dependencies in
an `add_library(... OBJECT ...)` and register it:

```cmake
add_test_exe(test_thing OBJECTS drone_common_obj drone_thing_obj)
```

Two constraints worth knowing before you pick what to compile in:

- **`spi.c` and `uart.c` do not build against the stub.** `stubs/hal_stub.c` has no `LL_SPI_*` and
  none of the `__HAL_UART_*` macros. Either extend the stub, or do what `test_crsf.c` does and
  define the two or three functions your unit actually needs directly in the test file.
- **MinGW needs `-mno-ms-bitfields`** (the build sets it globally). Under MSVC bitfield layout
  `CrsfChannelsPayload_t` packs to 32 bytes instead of the 22 ARM EABI produces, and the host would
  silently exercise a wire format the firmware never emits. `test_crsf.c` carries a
  `_Static_assert` on that size; do the same for any other struct whose memory layout is the thing
  under test.

Note that `GetMicroseconds()` returns **0** under `UNIT_TEST` (`core/core_shared.c`), so anything
gated on elapsed time — the CRSF inter-frame timeout, the RC link timeout — cannot be reached from
here. Those need a time seam that does not exist yet.

### History

This suite previously held tests for queue, vector, umap, gpio, spi, uart, imu, mag, dshot and
filter. None of them had built for some time: they all include `"unity/unity.h"`, while
`CMakeLists.txt` fetched Unity to `_deps/unity-src/src/`, where that path cannot resolve — and
several of their dependencies no longer compile against the HAL stub. They were removed rather
than left as decoration. Restoring any of them means fixing the target against the two constraints
above; the sources are in git history.

---

## HIL tests

Firmware compiled with `HIL_TEST=True` runs Unity on the board at boot and streams results over
UART:

```bash
python Scripts/board.py build -b flapjack-v1 -f t
python Scripts/board.py flash -b flapjack-v1 -f t --port <PORT> --baud 230400
```

`flash -f t` parses the stream and exits 0 (all pass) or 1 (failures). Output format:

```
file.c:42:test_name:PASS
file.c:43:test_name:FAIL:Expected X Was Y
N Tests M Failures 0 Ignored
```

---

## The third kind: the SIL

Neither suite covers the GNC loop end to end. That is the SIL's job — Renode plus JSBSim, driven
by `board.py renode` and `board.py sim`. A flight plan run (`sim --plan …`) exits 0 or 1 on a set
of physics invariants, which makes it usable as a coarse regression gate. See `RcSilResearch.md`
and the `flapjack-sil-debugging` skill.
