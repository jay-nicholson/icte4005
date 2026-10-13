# Master Arduino — process coordination and system andon

[Shared contract](../CONTRACT.md) · [Build instructions](../README.md#platformio)

Implemented and host-tested against the real slave logic. Not yet exercised on hardware. The master owns every phase, baseline, allowance, acknowledgement and system fault. It does not step the motor or debounce channels.

## Code layout

- `../shared/contract.h`: every number from CONTRACT.md as named constants. All three boards include it.
- `lib/feedback`: decodes and validates each reply, tracks freshness, counter regression, uptime stall and reboot. Pure C++.
- `lib/process`: the state machine below. Takes the two slave views, the operator command and the time; returns the commands to send. Pure C++.
- `src/config.h`: Serial baud, Wire timeout, andon pins and the calibration constants.
- `src/wire_link`: bounded Wire send and read.
- `src/operator_serial`: Serial command parsing and the status report.
- `src/main.cpp`: the 100 ms poll loop, Stop priority, andon output.
- `test/`: host tests. `test_process` feeds scripted contract bytes. `test_end_to_end` runs this master against the real conveyor and sensing logic on a simulated belt.

The pure modules compile unchanged for the Uno and for the host, so the tests cannot drift from the firmware.

```sh
pio run -d master
pio test -d master -e native
pio device monitor -d master --port /dev/cu.YOUR_BOARD --baud 115200
```

## Process rules in the code

State handlers never read the slave views. Each tick computes one `Observation` from the views and the saved baselines: arrived, magnetic, channel order, overdue and approach complete. Progress checks run on that observation in every cycle state, including STOPPING and PAUSED, before any operator command is handled. Resume uses the same observation to pick the next motion. This is what keeps pause, stop and resume from disagreeing.

A motion command is confirmed only by a reply received after it was sent. Step baselines are taken only from a conveyor reply in the same poll as the channel event. Stop has priority over any other operator command received in the same poll interval and discards it. A slave reporting CONTACT_LOST in INITIALISING, READY or BENCH only blocks progress, since its next reads clear it; any other slave fault, in any state, faults the master. Notices print when they change, not every poll.

## Serial commands

Commands end with newline. `start` is accepted only in READY, `resume` only in PAUSED, `ack` only in WAIT_ACK and only while channel 1 reads open. `stop` pauses any moving phase. `status` prints the report on demand. `help` lists the commands. There is no fault-clear command; reset the master.

`bench` enters BENCH from INITIALISING or READY. In BENCH the master keeps polling and reporting but runs no process logic and raises no faults. `send c g` sends `g` to the conveyor. `stop` still sends `s`. Leave BENCH by resetting the master. Use it to test one slave at a time and to measure calibration.

The report prints on every state change and once a second. Each slave shows current state, fault, feedback age and UNKNOWN or INVALID when feedback is missing or fails validation. The conveyor also shows its target state. A channel line with the current step count prints whenever a live bit or an edge count changes. Travel shows estimated steps against the active allowance.

## Calibration

Start is refused until `FORWARD_ALLOWANCE_STEPS` and `PADDLE_STEPS` in `src/config.h` are nonzero. Measure them in BENCH on the assembled belt: run the conveyor with raw sends, read the step count printed at each channel change for the allowance, and the step count from the channel 2 stop point to the paddle for the paddle distance. Add margin for polling, debounce and stopping distance. `STEPS_PER_MM` only adds millimetres to the report.

## Andon

Amber blinking in INITIALISING while waiting for the slaves. Off in READY and BENCH. Green while a cycle is moving or stopping. Amber steady in WAIT_ACK and PAUSED. Red in FAULT. Pins and blink period are in `src/config.h`; set the pins to the wiring.

## States

| State | Responsibility |
| --- | --- |
| INITIALISING | Request stop; require fresh STOPPED, sensing OK and both channels open. Absent slaves wait here without fault. |
| READY | Wait for manual loading and Start. Start saves both channel edge counts and sends `g`. |
| FORWARD | Run until a channel 1 edge. No arrival has no deadline. |
| TRACKING | Step baseline saved at arrival; check travel to channel 2. |
| STOPPING | Send `s` each poll; confirm STOPPED; then WAIT_ACK, CLEARING or PAUSED from the observation. |
| WAIT_ACK | Magnetic detected and stopped; wait for Ack. Ack saves the approach baseline and sends `g`. |
| APPROACH | Run forward the calibrated paddle distance from the stop point, then send `e`. |
| EJECTING | Confirm SWEEPING, wait for STOPPED within the sweep timeout. Stop is refused here. |
| CLEARING | Stopped; require both channels open within 300 ms; then READY. |
| PAUSED | Keep polling; retain phase and baselines. |
| BENCH | Poll and report only; forward raw operator sends. Reset to leave. |
| FAULT | Send `s` until a post-fault reply confirms rest; retain reason until master reset. A master reset recovers, because slaves clear CONTACT_LOST themselves once polling resumes. |

```mermaid
stateDiagram-v2
    [*] --> INITIALISING
    INITIALISING --> READY: healthy stopped boards
    INITIALISING --> BENCH: bench
    READY --> BENCH: bench
    READY --> FORWARD: start / g
    FORWARD --> TRACKING: channel 1 edge / save steps
    FORWARD --> STOPPING: magnetic / s
    TRACKING --> STOPPING: magnetic / s
    STOPPING --> WAIT_ACK: STOPPED and magnetic
    WAIT_ACK --> APPROACH: ack / g
    APPROACH --> EJECTING: paddle distance reached / e
    EJECTING --> CLEARING: SWEEPING then STOPPED
    STOPPING --> EJECTING: STOPPED and paddle distance reached
    CLEARING --> READY: both channels open
    FORWARD --> STOPPING: stop
    TRACKING --> STOPPING: stop
    APPROACH --> STOPPING: stop
    STOPPING --> PAUSED: STOPPED, nothing else due
    PAUSED --> FORWARD: resume / g
    PAUSED --> TRACKING: resume / g
    PAUSED --> APPROACH: resume / g
    PAUSED --> WAIT_ACK: resume with magnetic seen
    TRACKING --> FAULT: missing forward arrival
    EJECTING --> FAULT: sweep not confirmed or not finished
    FAULT --> [*]: master reset required
    note right of FAULT
        Invalid feedback, lost feedback, slave fault, channel order,
        unexpected motion or held channel enters FAULT from any cycle state.
    end note
```

## Verification

Host tests cover the complete cycle through approach and eject, both channels in one poll, channel order, missing forward arrival, event priority over the travel limit, pause in each phase including approach, events during stopping, stale replies that must not confirm a stop, stop retry and STOP UNCONFIRMED, sweep confirmation and sweep timeout, stop refused during a sweep, fault requests that continue without feedback, lost and invalid feedback, counter regression, slave faults, confirmation timeouts, reboot handling, held channels after eject, Ack blocked by channel 1, unexpected motion, limits exceeded during stopping, and bench mode. The end-to-end tests run the master against the real slave logic for a full cycle with a simulated sweep, pause and resume, a non-magnetic puck, slave watchdogs, master reset recovery and a short allowance.

Still to record on hardware: calibrated allowances, actual stopping distance, Wire timing with real boards, and the full demonstration sequence. Host tests and compilation do not verify mechanical behaviour.
