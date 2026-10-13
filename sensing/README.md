# Sensing Arduino — Hall sensors, LDR gate and local LEDs

[Shared contract](../CONTRACT.md) · [Build instructions](../README.md#platformio)

**The I2C contract is unchanged.** This board replaces the two reed switches with two linear Hall sensors and adds an optional LDR presence gate. The master still receives two debounced channels and their closing-edge counts, so no master, conveyor or `CONTRACT.md` change is needed. In the contract and code, "reed 1" and "reed 2" now mean channel 1 and channel 2; the names are kept so the wire format stays identical.

Channel 1 is puck arrival at the load/discharge end (Hall 1). Channel 2 is magnetic detection at the stop point (Hall 2, optionally gated by the LDR).

## Why Hall instead of reeds

A reed is a binary snap switch with mechanical hysteresis, and its trigger depends on gap and magnet orientation. A linear Hall sensor gives a graded reading, so the threshold is tuned from measured data. The front end adds:

| Stage | Purpose |
| --- | --- |
| Auto-zero at boot | The resting level differs per part, supply and mounting, so it is measured, not assumed. |
| Low-pass filter | ADC noise and coil pickup must not look like an event. |
| Absolute deviation | Magnet polarity and orientation stop mattering. |
| On/off hysteresis | A marginal signal cannot chatter. |
| Health checks | A pulled-out or shorted sensor latches a LOCAL fault. A disconnected normally-open reed just looks open. |

The existing 20 ms debounce and edge counting in `lib/sensing_logic` run unchanged on the output.

## Fusion gate (optional, off by default)

`USE_LDR_GATE` in `src/main.cpp`. When true, channel 2 closes only if Hall 2 is active **and** the LDR agrees a puck is physically there. This rejects a stray field at Hall 2 (stepper-coil pickup, a dropped magnet) that the Hall alone cannot tell from a puck. Host tests show the same transient stray pulse faulting the master with the gate off and being ignored with it on.

Limits, stated plainly:
- The gate cannot reject a stray field that coincides with a **non-magnetic** puck at the station, because the LDR sees "a puck" either way. It narrows the false-trigger window; it does not close it.
- It adds a failure mode: ambient light changes can hold the gate shut, so channel 2 never closes and the master latches a missing-arrival fault. Leave it off unless it has been bench-tested in the demo lighting.

## Wiring

| Item | Connection |
| --- | --- |
| Hall 1 (UGN3503UA) | Vcc to 5 V, GND to GND, output to **A0** |
| Hall 2 (UGN3503UA) | Vcc to 5 V, GND to GND, output to **A1** |
| LDR module | Power to 5 V and GND, analog output to **A2** |
| Green / amber / red LED | Pins **5 / 6 / 7**, each through a ~330 ohm resistor to GND (as in the seed) |
| I2C | A4 (SDA) and A5 (SCL) to the master; common ground. Not used for analog. |

Check the UGN3503UA pin order against its datasheet before powering it: wrong-way power can destroy the part. A 0.1 uF ceramic across each Hall's Vcc and GND is good practice. Check the LDR module's silkscreen for the signal pin.

## Calibration

Build the calibrate environment (`pio run -d sensing -e calibrate -t upload`) or use the standalone sketch. Open Serial at 9600 baud with the belt clear and magnets away. Keys: `r` resets the peak values, `z` re-zeros the baselines.

1. Press `r` with nothing near the sensors, wait 10 s. The `pk` values are the **noise floor**.
2. Press `r`, then pass the real puck by each sensor at the gap and speed it will see in the demo. The `pk` value is the **signal peak**. Repeat for the worst case (largest gap, fastest belt, weakest orientation).
3. Set `onDelta` to roughly half the weakest signal peak, but at least 3x the noise floor. Set `offDelta` to about half of `onDelta`.
4. Check the baselines (`b1`, `b2`) sit near 512. If not, adjust `baselineMin/Max` or check the wiring.
5. For the LDR, do the same with and without a puck at the station. Only enable the gate if the swing is several times larger than ambient fluctuation.

`sensing/src/main.cpp` ships with **starting values** (`onDelta` 12, `offDelta` 6) that have not been tuned on the assembled belt.

## Code layout

- `lib/analog_channel`: baseline, filter, deviation, hysteresis and health. Pure C++, host-tested.
- `lib/sensing_logic`: debounce, counters, LED decision, reply encoder. One addition to the original: `raiseLocalFault()`, which latches the contract's existing LOCAL fault and keeps the first fault reason.
- `src/main.cpp`: pins, tuning, sampling loop, fusion, LED output, Wire link.
- `src/calibrate.cpp`: the bench measuring tool, built by the `calibrate` environment.
- `../shared/slave_link.h`: Wire slave boilerplate, unchanged.

Boot behaviour: all three LEDs light for 300 ms (lamp test), then the baselines are captured over about 0.26 s. **Keep the belt clear and no magnet near either sensor at power-up**, or the baseline is wrong. A baseline outside the plausible band latches a LOCAL fault rather than running with bad data.

## LEDs

Green while channel 1 reads closed. Amber while channel 2 reads closed. Red while the board is in FAULT. Off otherwise. The LEDs show the local reading only.

## Verification

Recorded on 13 October 2026, host and compiler only:
- `pio test -d sensing -e native` runs 19 tests: 11 unit tests of the front end and 8 end-to-end tests that drive the **unmodified** master and conveyor logic against a noisy Hall model. Covered: boot zeroing, noise rejection, either polarity, one edge per pass at 10 to 150 mm/s, hysteresis, bad baseline, stuck-at-rail, LOCAL fault latching, unchanged wire bytes, full cycles with noise across 8 seeds, a non-magnetic puck, an unplugged sensor, and the stray-field case with and without the gate.
- The master's own 41 tests still pass against the modified `sensing_logic`.
- `sensing` and `calibrate` compile for the ATmega328P with avr-gcc 7.3 and the Arduino AVR core, no warnings in project files under `-Wall -Wextra`. Firmware uses about 21% of flash and 32% of RAM.

**Not verified:** any hardware upload, real Hall or LDR readings, the starting thresholds on a real belt, I2C with real boards, or mechanical operation. The Hall model in the tests is an assumed bell-shaped field, not a measurement.

## Known gaps and improvements

- No baseline drift compensation: a long run with temperature change can shift the resting level. A slow tracker that only updates while the channel is inactive would fix this.
- Fixed sensitivity (about 0.27 counts per gauss at 10-bit): a weak or distant magnet may be invisible. An op-amp stage or a 16-bit ADC would help.
- Calibration is manual, and the starting thresholds are unmeasured.
- Single LDR at one position; it cannot verify channel 1.
