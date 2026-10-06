# esphome-somfy-lidar

[![CI](https://github.com/d-asensio/esphome-somfy-lidar/actions/workflows/ci.yml/badge.svg)](https://github.com/d-asensio/esphome-somfy-lidar/actions/workflows/ci.yml)

ESPHome components for controlling a **Somfy RTS awning** with an ESP32 and a CC1101.
The awning's position is **measured** by a TF-Luna lidar instead of estimated from
travel time.

- **Real position:** the position stays correct whether the awning was moved from Home Assistant, the original remote, or a Somfy wind sensor.
- **No sniffing:** the ESP doesn't listen to or copy your existing remotes, and their rolling codes are never touched.
- **Lightweight firmware:** plain ESPHome talking to Home Assistant over the native API; no web UI runs on the device.
- **Self-calibrating:** one button records the closed and open ends, and every move fine-tunes when to stop.

> **Status:** early. The firmware builds in CI and passes a closed-loop simulation, but
> it has not been tested on real hardware yet. Test it with the awning in view and the
> original remote at hand.

## Contents

- [How it works](#how-it-works)
- [Hardware](#hardware)
- [Installation](#installation)
- [Pairing with the motor](#pairing-with-the-motor)
- [Calibration](#calibration)
- [Mounting the lidar](#mounting-the-lidar)
- [Configuration reference](#configuration-reference)
- [Coming from ESPSomfy RTS](#coming-from-espsomfy-rts)
- [Development](#development)
- [Credits](#credits)

## How it works

| Component | Role |
|---|---|
| `cc1101` + `remote_transmitter` (built into ESPHome) | Drive the CC1101 at 433.42 MHz in OOK mode |
| [`somfy_rts`](components/somfy_rts) | A virtual Somfy RTS remote with its own address and a rolling code kept in flash |
| [`tfluna`](components/tfluna) | Reads the TF-Luna over UART at 100 Hz and publishes the median of each 100 ms window |
| [`somfy_awning`](components/somfy_awning) | The cover entity: converts distance to 0–100 %, runs the closed loop and the calibration |

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

MY is never sent to a motor that is standing still, because there it means "go to the
favourite position", not "stop".

## Hardware

| Part | Notes |
|---|---|
| ESP32 dev board | Any classic ESP32 (`esp32dev`) |
| CC1101 module, **433 MHz** version, with antenna | 868 MHz modules don't work well at 433.42 MHz |
| [Benewake TF-Luna](https://en.benewake.com/TFLuna/index.html) lidar | 0.2–8 m range, about 2° beam |
| 5 V power supply | Powers both the ESP32 and the TF-Luna |

### Wiring

The CC1101 pins are the ESPSomfy RTS defaults, so an existing ESPSomfy board can be
reused as it is.

| CC1101 | ESP32 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SCK | GPIO18 |
| MISO (GDO1) | GPIO19 |
| MOSI | GPIO23 |
| CSN | GPIO5 |
| GDO0 | GPIO13 |
| GDO2 | not used |

| TF-Luna | ESP32 |
|---|---|
| 1 – 5V | 5V (VIN) |
| 2 – TX | GPIO16 |
| 3 – RX | GPIO17 |
| 4 – GND | GND |
| 5 – CFG | leave unconnected (UART mode). If you ground it, the sensor switches to I2C. |
| 6 | not used |

The TF-Luna's UART runs at 3.3 V logic, so it connects directly to the ESP32.

## Installation

You need ESPHome **2026.9 or newer**. The components are loaded straight from this
repository, so you only need a YAML file. Use either the ESPHome Device Builder (A)
or the command line (B).

### A. ESPHome Device Builder (Home Assistant add-on)

1. In the ESPHome Device Builder, click **+ New device**, give it a name (for example `awning`), choose **ESP32**, and skip the install step.
2. Click **Edit** on the new device and replace its YAML with the contents of [`example/awning.yaml`](example/awning.yaml). Keep the `api: encryption: key` the builder generated for you; that is the device's own key.
3. Open **Secrets** (top right) and make sure `wifi_ssid` and `wifi_password` exist. If you kept your own API key in step 2, you don't need `api_encryption_key` in secrets.
4. Change `address:` under `somfy_rts:` to a random 24-bit hex value, for example `0x5E3A91`. It only has to differ from every other remote you own.
5. Click **Install**:
   - **First install:** connect the ESP32 by USB to the computer running the browser and choose **Plug into this computer**, or choose **Manual download** and flash the file with [ESPHome Web](https://web.esphome.io).
   - **Later updates:** go over Wi-Fi.
6. Home Assistant discovers the device. Accept it under **Settings → Devices & services**.

### B. Command line

```sh
pip install esphome
mkdir awning && cd awning
curl -O https://raw.githubusercontent.com/d-asensio/esphome-somfy-lidar/main/example/awning.yaml
curl -o secrets.yaml https://raw.githubusercontent.com/d-asensio/esphome-somfy-lidar/main/example/secrets.yaml.example
# Edit secrets.yaml. Generate an API key with: openssl rand -base64 32
# Edit awning.yaml: set a random somfy_rts address.
esphome run awning.yaml   # first time over USB, then over Wi-Fi
```

Then add the device in Home Assistant, as in step 6 above.

### Pinning a version

`github://d-asensio/esphome-somfy-lidar@main` always pulls the latest code. To stay on
a fixed version, replace `main` with a release tag or a commit hash.

## Pairing with the motor

You do this once. The buttons below are in Home Assistant, on the device page under
*Configuration*. Some are hidden by default; enable them from the entity list.

1. Enable the **Send PROG** button.
2. On a remote that already controls the awning, hold **PROG** (the small button on the back) until the awning jogs briefly.
3. Within a few seconds, press **Send PROG**. The awning jogs again; the ESP is now paired.
4. Check that **Open** and **Close** work.
   - If they are reversed, set `open_command: UP` under the cover in YAML and reinstall.

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
**Set open position here** (hidden by default), or provide them in YAML:

```yaml
cover:
  - platform: somfy_awning
    # ...
    closed_distance: 250cm
    open_distance: 30cm
```

## Mounting the lidar

This is the part that determines whether the setup works well.

- **Where:** mount the sensor on the wall or under the cassette, pointing at the front bar along the direction the awning extends.
- **Target plate:** on folding-arm awnings the front bar drops as it extends. Put a flat, matte target plate on the bar, tall enough that the beam hits it over the whole travel. The beam is about 10 cm wide at 3 m.
- **Check the signal:** watch **Lidar signal strength** while the awning moves. It should stay well above 100 the whole way.
  - If it drops below 100, the reading is rejected and positioning moves stop.
  - If it reads 65535, the receiver is saturated. Avoid shiny or retroreflective targets and direct sun into the lens.
- **Weather:** shield the lens from rain with a small hood.

## Configuration reference

### `somfy_rts`

```yaml
somfy_rts:
  id: awning_remote
  transmitter_id: rf_tx  # the remote_transmitter wired to CC1101 GDO0
  address: 0x5E3A91      # required, unique 24-bit address of this virtual remote
  rolling_code: 1        # only used while nothing is stored in flash yet
  repeat: 2              # extra frames per command
```

The action `somfy_rts.send` sends a command (`UP`, `DOWN`, `MY` or `PROG`) from
automations:

```yaml
- somfy_rts.send:
    id: awning_remote
    command: MY
    repeat: 4  # optional
```

### `tfluna` sensor

```yaml
sensor:
  - platform: tfluna
    uart_id: lidar_uart       # 115200 baud
    update_interval: 100ms    # median of all frames in each interval
    min_signal_strength: 100  # weaker frames are discarded
    distance:                 # cm, NAN when no valid frame arrived
      id: awning_distance
    signal_strength:          # optional
      name: Lidar signal strength
    temperature:              # optional, chip temperature
      name: Lidar temperature
```

### `somfy_awning` cover

| Option | Default | Description |
|---|---|---|
| `somfy_rts_id` | the only `somfy_rts` | The remote paired with the motor |
| `distance_sensor` | **required** | The distance sensor (cm) |
| `open_command` | `DOWN` | Button that extends the awning |
| `closed_distance`, `open_distance` | | Starting calibration; ignored once a calibration is stored |
| `stop_latency` | `600ms` | Starting value for the time from MY to standstill; learned after each move |
| `position_tolerance` | `2%` | Positions this close to an end are shown as fully open/closed |
| `min_travel` | `3%` | Smaller moves are skipped |
| `max_travel_time` | `120s` | A move or calibration phase taking longer than this is aborted |

Methods you can call from lambdas: `start_calibration()`, `set_closed_here()`,
`set_open_here()`, `is_calibrated()`, `get_stop_latency()`.

### Safety behaviour

- If the lidar stops giving valid readings during a positioning move, the awning is stopped with MY.
- A positioning move that takes longer than `max_travel_time` is stopped.
- If something else stops the awning mid-move (the original remote, a wind sensor), the target is dropped and no MY is sent.
- Wind sensors and the original remote keep working, because they talk to the motor directly.

## Coming from ESPSomfy RTS

The CC1101 wiring is the same, so the same board works. You have two options.

**Pair a new remote** (recommended): follow [Pairing with the motor](#pairing-with-the-motor).
You can then remove the old ESPSomfy remote from the motor, or leave it.

**Take over the existing ESPSomfy remote:**

1. In ESPSomfy, note the shade's *remote address* and current *rolling code*.
2. Set `address:` to that address.
3. Set `rolling_code:` to a value a little above that rolling code, for example 20 higher.
4. Flash.

The `rolling_code:` value is only used while nothing is stored in flash yet, so this works on the first flash only.

## Development

```
components/        ESPHome external components
example/           Example device configuration
tests/run_sim.sh   Builds the components for ESPHome's host platform and runs
                   them against a simulated awning
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
in [`somfy_rts.cpp`](components/somfy_rts/somfy_rts.cpp) are ported from its `Somfy.cpp`,
where they were worked out from real Somfy remotes. ESPSomfy RTS is a full-featured,
standalone controller for many shades; if you don't want a distance sensor, use it.

Somfy is a trademark of Somfy SAS. This project is not affiliated with or endorsed by Somfy.

## License

[MIT](LICENSE)
