# spangap/imu — is this device moving?

**imu** holds a **QMI8658** accelerometer in low power with **Wake-on-Motion**
armed and publishes one fact: whether the device is moving. That is the whole
scope — no orientation, no tilt, no step counting, no sample stream. A consumer
that wants those wants a different straddle; keeping this one to a single
question is what keeps its power cost at tens of microamps and its behaviour
testable.

Its first consumer is [gps](../gps), which stops tracking a device that is
standing still. The coupling is over the storage bus only — neither straddle
links against the other.

## Board wiring (`CONFIG_IMU_*`)

This straddle **defines** the knobs (`esp-idf/Kconfig`); a board sets their
values in its `straddle.yaml` `kconfig:` block, gated `when: spangap/imu`, and
lists `spangap/imu` in `additional_installs:`.

The part speaks the same registers over either bus, so a board states which one
it wired and then only that bus's pins. Everything above `regRead`/`regWrite` in
`imu.cpp` is transport-blind.

| Symbol | Default | Meaning |
|---|---|---|
| `CONFIG_IMU_BUS_SPI` / `CONFIG_IMU_BUS_I2C` | SPI | which bus the part is on; picks which block below applies |
| `CONFIG_IMU_INT_PIN` | `-1` | motion interrupt; `-1` = poll only |
| `CONFIG_IMU_INT_LINE` | `1` | which of the part's two interrupt outputs that pin is wired to |

SPI (`CONFIG_IMU_BUS_SPI`):

| Symbol | Default | Meaning |
|---|---|---|
| `CONFIG_IMU_SPI_HOST` | `-1` | SPI host the part is on; `-1` = not wired, service dormant |
| `CONFIG_IMU_SPI_SCK_PIN` / `_MOSI_PIN` / `_MISO_PIN` | `-1` | bus pins, used **only** when this straddle has to bring the host up itself |
| `CONFIG_IMU_CS_PIN` | `-1` | chip select |

I2C (`CONFIG_IMU_BUS_I2C`):

| Symbol | Default | Meaning |
|---|---|---|
| `CONFIG_IMU_I2C_SDA_PIN` / `_SCL_PIN` | `-1` | bus pins; `-1` = not wired, service dormant |
| `CONFIG_IMU_I2C_PORT` | `-1` | `-1` = create the bus; `0`/`1` = adopt the board's existing bus on that controller |
| `CONFIG_IMU_I2C_ADDR` | `0x6B` | the part's SA0 strapping (`0x6A` with SA0 low) |

**The bus is shared, and this straddle assumes it.** On SPI the IMU usually sits
on the SD card's host, which `spangapInit()` claims before any service runs, so
the driver initialises the host only if it finds it free and **adopts** it
otherwise — `ESP_ERR_INVALID_STATE` is success here, not failure. That is why
the SPI pin symbols above are qualified: on a board with an SD card they are
never used. On I2C the sharing is explicit instead: a board whose bus already
carries a touch controller, an RTC or an IO expander brings that bus up in its
own start-band service and names the controller in `CONFIG_IMU_I2C_PORT`, and
this straddle adopts it rather than putting a second master on the same two
wires.

**SPI mode is discovered, not configured.** The part accepts mode 0 and mode 3,
and vendor material does not commit to which a given board wants, so the driver
tries mode 0 and falls back to mode 3, deciding on the `WHO_AM_I` read.

**An interrupt on an IO expander is not an interrupt.** `CONFIG_IMU_INT_PIN`
names a real GPIO or nothing: a board that routes INT1/INT2 through an I2C
expander (the Waveshare 2.8B does) leaves it at `-1` and the service polls,
which costs nothing here because the poll is what clears the part's latch
anyway.

## Published keys

| Key | Type | Meaning |
|---|---|---|
| `imu.present` | 0/1 | the part answered at boot |
| `imu.model` | string | `QMI8658`, or empty |
| `imu.moving` | 0/1 | debounced: 1 on a motion event, 0 after `s.imu.still_s` without one |
| `imu.motion` | int | per-boot event counter |
| `imu.still_s` | int | seconds since the last motion event |
| `imu.state` | string | finished text: `moving`, `still 4m`, `asleep`, `not present` |

**Both a level and a counter, on purpose.** `imu.moving` answers "is it moving
now"; `imu.motion` answers "did it move since I last looked", which a consumer
that was asleep through the edge cannot get from a level. Both are published by
the same task from the same event.

**Boot counts as motion.** Something moved the device to switch it on, so the
service starts in `moving` — a consumer that parks on stillness begins awake
rather than from a still-timer that has been running since the epoch.

## Settings

| Key | Default | Meaning |
|---|---|---|
| `s.imu.enable` | `1` | staged only where the part exists, so on by default |
| `s.imu.threshold_mg` | `40` | motion threshold, **lower = more sensitive** (1 mg/LSB, 1–255) |
| `s.imu.odr` | `11` | accelerometer rate in Hz: 3 / 11 / 21 / 128 |
| `s.imu.still_s` | `30` | how long without an event counts as still |

Rate is a power dial as much as a responsiveness one — the data sheet's
low-power figures are 30 µA at 3 Hz, 35 at 11, 42 at 21 and 55 at 128 (typical,
1.8 V), and the pane's dropdown carries them in its labels. Disabled, the part
goes to its lowest state: engines off and the internal oscillator stopped.

## How it watches

Wake-on-Motion is armed with the data sheet's sequence — sensors off, range and
rate, threshold and interrupt selection in the `CAL1_*` registers, the `CTRL9`
command, accelerometer on. An event latches `STATUS1` bit 2 and toggles the
selected interrupt line; **reading `STATUS1` clears the latch** and releases the
line.

That latch is why **polling is a first-class mode**: a poll can be late but
cannot miss an event. The task polls at 1 Hz whether or not an interrupt is
wired, so a board that brings no line out — or brings it out on a pin that
cannot do what you need, such as a non-RTC GPIO on an ESP32-S3 that therefore
cannot wake deep sleep — works exactly the same, just with up to a second more
latency. The poll runs only while the sensor is armed: disabled or absent, the
task parks until a config change wakes it, so it adds no wakes to a sleeping
node.

## CLI

| Command | Effect |
|---|---|
| `imu` | sensor, state, still time, event count, threshold/rate, interrupt wiring |
| `imu on` / `imu off` | set `s.imu.enable` |

## Read next

- [gps](../gps) — the first consumer: `s.gps.imu_assist` parks the receiver
  while `imu.moving` is 0.
