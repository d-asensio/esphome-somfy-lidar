# esphome-somfy-lidar

[![CI](https://github.com/d-asensio/esphome-somfy-lidar/actions/workflows/ci.yml/badge.svg)](https://github.com/d-asensio/esphome-somfy-lidar/actions/workflows/ci.yml)

An ESPHome component for controlling a **Somfy RTS awning** with an ESP32-S3 and a
CC1101. The awning's position is **measured** by a TF-Luna lidar instead of estimated
from travel time.

- **Real position:** the position stays correct whether the awning was moved from Home Assistant, the original remote, or a Somfy wind sensor.
- **No sniffing:** the ESP doesn't listen to or copy your existing remotes, and their rolling codes are never touched.
- **One config block:** you set the bus pins and a single `somfy_lidar:` block. The radio, lidar, cover, buttons and sensors are set up for you.
- **Self-calibrating:** one button records the closed and open ends, and every move fine-tunes when to stop.

> **Status:** early. The firmware builds in CI and passes a closed-loop simulation, but
> it has not been tested on real hardware yet. Test it with the awning in view and the
> original remote at hand.

## Contents

- [Hardware](#hardware)
- [Wiring](#wiring)
- [Installation](#installation)
- [Pairing with the motor](#pairing-with-the-motor)
- [Calibration](#calibration)
- [Mounting the lidar](#mounting-the-lidar)
- [Configuration](#configuration)
- [How it works](#how-it-works)
- [Coming from ESPSomfy RTS](#coming-from-espsomfy-rts)
- [Development](#development)
- [Credits](#credits)

## Hardware

This is the hardware the example configuration is written for:

| Part | Example | Notes |
|---|---|---|
| ESP32-S3 board, 8 MB flash | [diymore ESP32-S3, 8 MB flash / 8 MB PSRAM, USB-C](https://www.amazon.es/dp/B0GZTF723K) | Any ESP32-S3 board works; check the pins below |
| CC1101 module, **433 MHz** | [Ebyte E07-M1101D-SMA](https://www.amazon.es/dp/B0FMR33GWZ) | 3.3 V only, SMA antenna jack |
| 433 MHz antenna | [Jopto 433 MHz 3 dBi](https://www.amazon.es/dp/B07T71H3MF) | Check the connector, see below |
| Lidar | [youyeetoo / Benewake TF-Luna](https://www.amazon.es/dp/B088BBJ9SQ) | Used in I2C mode |
| 5 V USB power supply | | Powers the board and the TF-Luna |

> [!IMPORTANT]
> **Notes on these parts**
>
> - **The E07-M1101D runs on 3.3 V only.** Never connect its VCC to 5 V.
> - **Antenna connector.** The E07-M1101D-SMA has a standard **SMA** jack. The Jopto antennas are sold for routers with **RP-SMA** connectors. An RP-SMA antenna screws onto an SMA jack, but the centre contacts don't meet, so almost no signal gets out. If the antenna's centre is a hole rather than a pin, put an RP-SMA-to-SMA adapter in between, or use an antenna with an SMA (male) plug. The U.FL pigtails in the antenna kit aren't needed for this module.
> - **Run the board from USB, not from a battery.** The TF-Luna needs 3.7–5.2 V on its 5 V pin. Most ESP32-S3 boards only provide 5 V on that pin while USB is connected. A single Li-ion cell is too close to the TF-Luna's lower limit.
> - **The TF-Luna starts in UART mode.** Tie its pin 5 to GND to put it in I2C mode. The cable in the youyeetoo kit leaves pin 5 unconnected, so you need to add that wire.
> - **I2C pull-ups.** The ESP32's internal pull-ups are enabled, which is fine for short wires. If the cable to the TF-Luna is longer than about 30 cm, add 4.7 kΩ resistors from SDA and SCL to 3.3 V. The ESP and the TF-Luna should sit close together in any case. I2C isn't meant for runs of several metres, so for that, move the ESP next to the sensor rather than extending the cable.
> - **Check the pins against your board.** I couldn't confirm the pin labels of this exact diymore board. The pins in the example are ones that are safe on any ESP32-S3 with octal PSRAM (see [Wiring](#wiring)). If your board doesn't break out one of them, any other free GPIO works.

## Wiring

| E07-M1101D (CC1101) | ESP32-S3 |
|---|---|
| VCC | **3V3** |
| GND | GND |
| SCK | GPIO12 |
| MOSI | GPIO11 |
| MISO | GPIO13 |
| CSN | GPIO10 |
| GDO0 | GPIO9 |
| GDO2 | not connected |

| TF-Luna | ESP32-S3 |
|---|---|
| 1 – 5V (red) | 5V |
| 2 – SDA (white) | GPIO4 |
| 3 – SCL (green) | GPIO5 |
| 4 – GND (black) | GND |
| 5 – CFG | **GND** (selects I2C) |
| 6 | not connected |

Pins to **avoid** on an ESP32-S3 with 8 MB octal PSRAM:

| Pins | Reason |
|---|---|
| GPIO0, 3, 45, 46 | Strapping pins |
| GPIO19, 20 | USB |
| GPIO26–37 | Flash and PSRAM |
| GPIO43, 44 | Serial console |

## Installation

You need ESPHome **2026.9 or newer**. The component is loaded straight from this
repository, so you only need a YAML file. Use either the ESPHome Device Builder (A)
or the command line (B).

### A. ESPHome Device Builder (Home Assistant add-on)

1. In the ESPHome Device Builder, click **+ New device**, give it a name (for example `awning`), choose **ESP32-S3**, and skip the install step.
2. Click **Edit** on the new device. Replace its YAML with the contents of [`example/awning.yaml`](example/awning.yaml), but keep the `api: encryption: key` the builder generated for you.
3. Open **Secrets** (top right) and make sure `wifi_ssid` and `wifi_password` exist.
4. Under `somfy_lidar:`, change `remote_address:` to a random 24-bit hex value, for example `0x5E3A91`. It only has to differ from every other remote you own.
5. Click **Install**:
   - **First install:** connect the board by USB to the computer running the browser and choose **Plug into this computer**, or choose **Manual download** and flash the file with [ESPHome Web](https://web.esphome.io). If the board isn't detected, hold **BOOT** while plugging it in.
   - **Later updates:** go over Wi-Fi.
6. Home Assistant discovers the device. Accept it under **Settings → Devices & services**.

### B. Command line

```sh
pip install esphome
mkdir awning && cd awning
curl -O https://raw.githubusercontent.com/d-asensio/esphome-somfy-lidar/main/example/awning.yaml
curl -o secrets.yaml https://raw.githubusercontent.com/d-asensio/esphome-somfy-lidar/main/example/secrets.yaml.example
# Edit secrets.yaml. Generate an API key with: openssl rand -base64 32
# Edit awning.yaml: set a random remote_address.
esphome run awning.yaml   # first time over USB, then over Wi-Fi
```

Then add the device in Home Assistant, as in step 6 above.

### Pinning a version

`github://d-asensio/esphome-somfy-lidar@main` always pulls the latest code. To stay on
a fixed version, replace `main` with a release tag or a commit hash.

## Pairing with the motor

You do this once. The buttons below are on the device page in Home Assistant, under
*Configuration*. Some are hidden by default; enable them from the entity list.

1. Enable the **Send PROG** button.
2. On a remote that already controls the awning, hold **PROG** (the small button on the back) until the awning jogs briefly.
3. Within a few seconds, press **Send PROG**. The awning jogs again; the ESP is now paired.
4. Check that **Open** and **Close** work.
   - If they are reversed, set `open_command: UP` and reinstall.

Repeating the procedure with **Send PROG** unpairs the ESP again.

## Calibration

Press **Calibrate**. The awning:

1. closes,
2. opens fully,
3. closes again.

At each end it waits until the reading has been stable for 2.5 s, then records the
distance. The calibration is stored in flash and survives reboots and updates.

Until it is calibrated, only fully open and fully closed work.

You can also set the ends by hand with **Set closed position here** and
**Set open position here** (hidden by default), or give starting values in YAML with
`closed_distance:` and `open_distance:`.

## Mounting the lidar

This is the part that determines whether the setup works well.

- **Where:** mount the sensor on the wall or under the cassette, pointing at the front bar along the direction the awning extends.
- **Target plate:** on folding-arm awnings the front bar drops as it extends. Put a flat, matte target plate on the bar, tall enough that the beam hits it over the whole travel. The beam is about 10 cm wide at 3 m.
- **Check the signal:** watch **Lidar signal strength** while the awning moves. It should stay well above 100 the whole way.
  - If it drops below 100, the reading is rejected and positioning moves stop.
  - If it reads 65535, the receiver is saturated. Avoid shiny or retroreflective targets and direct sun into the lens.
- **Weather:** shield the lens from rain with a small hood.

## Configuration

```yaml
spi:            # CC1101 bus
  clk_pin: GPIO12
  mosi_pin: GPIO11
  miso_pin: GPIO13

i2c:            # TF-Luna bus
  sda: GPIO4
  scl: GPIO5
  frequency: 100kHz

external_components:
  - source: github://d-asensio/esphome-somfy-lidar@main
    components: [somfy_lidar]

somfy_lidar:
  remote_address: 0x5E3A91
  cs_pin: GPIO10
  gdo0_pin: GPIO9
```

| Option | Default | Description |
|---|---|---|
| `remote_address` | **required** | 24-bit address of the ESP's virtual remote. Random, and unique among your remotes |
| `cs_pin` | **required** | CC1101 CSN |
| `gdo0_pin` | **required** | CC1101 GDO0 (transmit data) |
| `open_command` | `DOWN` | Button that extends the awning |
| `output_power` | `10` | Transmit power in dBm (-30 to 11) |
| `lidar_address` | `0x10` | TF-Luna I2C address |
| `min_signal_strength` | `100` | Lidar readings weaker than this are discarded |
| `closed_distance`, `open_distance` | | Starting calibration (e.g. `250cm`); ignored once a calibration is stored |
| `stop_latency` | `600ms` | Starting value for the time from MY to standstill; learned after each move |
| `position_tolerance` | `2%` | Positions this close to an end are shown as fully open/closed |
| `min_travel` | `3%` | Smaller moves are skipped |
| `max_travel_time` | `120s` | A move or calibration phase taking longer than this is aborted |
| `rolling_code` | `1` | Starting rolling code, used only while none is stored in flash |
| `repeat` | `2` | Extra radio frames per command |
| `id` | | ID of the cover, for automations |

### Entities

| Entity | Type | |
|---|---|---|
| *(device name)* | Cover | Open, close, stop, set position |
| Distance | Sensor | Measured distance, averaged over 5 s |
| Calibrate | Button | Automatic calibration |
| Set closed position here | Button | Manual calibration (hidden by default) |
| Set open position here | Button | Manual calibration (hidden by default) |
| Send PROG | Button | Pairing (hidden by default) |
| Lidar signal strength | Diagnostic | Averaged over 30 s |
| Lidar temperature | Diagnostic | Averaged over 60 s |
| Stop latency | Diagnostic | Current learned value |

### Automations

The cover works with the usual `cover.open`, `cover.close`, `cover.stop` and
`cover.control` actions. Raw RTS commands can be sent too, for example to send the
awning to the motor's own favourite (MY) position:

```yaml
somfy_lidar:
  id: awning
  # ...

button:
  - platform: template
    name: Favourite position
    on_press:
      - somfy_lidar.send_command:
          id: awning
          command: MY  # UP, DOWN, MY or PROG
```

### Safety behaviour

- If the lidar stops giving valid readings during a positioning move, the awning is stopped with MY.
- A positioning move that takes longer than `max_travel_time` is stopped.
- If something else stops the awning mid-move (the original remote, a wind sensor), the target is dropped and no MY is sent.
- MY is never sent to a motor that is standing still, because there it means "go to the favourite position", not "stop".
- Wind sensors and the original remote keep working, because they talk to the motor directly.

## How it works

The `somfy_lidar` component sets up four parts:

| Part | Role |
|---|---|
| ESPHome's `cc1101` and `remote_transmitter` drivers | Drive the CC1101 at 433.42 MHz in OOK mode. They are configured internally, so they don't appear in your YAML |
| Virtual RTS remote | Has its own address, and a rolling code saved to flash before every transmission |
| TF-Luna reader | Polls the lidar over I2C every 20 ms and publishes the median of each 100 ms window |
| Awning cover | Converts distance to 0–100 %, runs the closed loop and the calibration |

The ESP is paired with the motor as an **additional remote**. Somfy motors accept
about a dozen remotes, each with its own address and rolling code, so your existing
remotes keep working exactly as before.

Moving to a position:

- **Fully open or fully closed:** sends DOWN or UP, and the motor stops at its own end limits.
- **Any other position:**
  1. Sends UP or DOWN and watches the measured position.
  2. Sends MY (stop) when the remaining distance is less than *current speed × stop latency*.
  3. After each such move, compares where the awning stopped with the target and adjusts the stop latency.
- **Moves made with the original remote** are detected from the sensor and reported to Home Assistant.

## Coming from ESPSomfy RTS

You have two options.

**Pair a new remote** (recommended): follow [Pairing with the motor](#pairing-with-the-motor).
You can then remove the old ESPSomfy remote from the motor, or leave it.

**Take over the existing ESPSomfy remote:**

1. In ESPSomfy, note the shade's *remote address* and current *rolling code*.
2. Set `remote_address:` to that address.
3. Set `rolling_code:` to a value a little above that rolling code, for example 20 higher.
4. Flash.

The `rolling_code:` value is only used while nothing is stored in flash yet, so this works on the first flash only.

## Development

```
components/somfy_lidar/   The ESPHome component
example/                  Example device configuration
tests/run_sim.sh          Builds the component for ESPHome's host platform and runs
                          it against a simulated awning
```

In the simulation, the motor reacts to the RTS frames actually transmitted (decoded
back from the raw timings). It has start and stop delays, and it goes to its
favourite position if it ever gets MY while standing still. The scenarios cover:

- calibration;
- positioning accuracy;
- stop-latency learning;
- interference from the physical remote;
- sensor dropouts.

```sh
pip install esphome
tests/run_sim.sh   # about 3 minutes
```

To build the example against a local checkout, point `external_components` at it:

```yaml
external_components:
  - source:
      type: local
      path: ../components
```

CI validates and compiles the example firmware and runs the simulation on every push.

## Credits

This project builds on [**ESPSomfy RTS**](https://github.com/rstrouse/ESPSomfy-RTS) by
[@rstrouse](https://github.com/rstrouse). The RTS frame format and the transmit timings
in [`somfy_rts.cpp`](components/somfy_lidar/somfy_rts.cpp) are ported from its `Somfy.cpp`,
where they were worked out from real Somfy remotes. ESPSomfy RTS is a full-featured,
standalone controller for many shades; if you don't want a distance sensor, use it.

Somfy is a trademark of Somfy SAS. This project is not affiliated with or endorsed by Somfy.

## License

[MIT](LICENSE)
