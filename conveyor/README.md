# Conveyor Arduino — motion

[Shared contract](../CONTRACT.md) · [Build instructions](../README.md#platformio)

The proven standalone sketch's motor driver is integrated: a 28BYJ-48 on a ULN2003, half-stepped directly from four pins, no library. Set the pins and `MM_PER_HALF_STEP` in `src/main.cpp` to the assembled rig.

## What this board owns

Stepping, direction, a cumulative step count and its own contact watchdog. It accepts `g`, `b` and `s`. It reports STOPPED, FORWARD, REVERSE or FAULT, a fault reason, uptime and the step count. It holds no process state. The master decides when movement starts and ends.

## Code layout

- `lib/conveyor_logic`: the state machine and the reply encoder. Pure C++, so the master's host tests run it directly.
- `src/main.cpp`: the half-step driver, fixed-speed stepping, bench keys, the Wire link and the publish loop. The ejector paddle code is present behind `EJECTOR_ENABLED`, off by default, for the planned ejection cycle.
- `../shared/slave_link.h`: Wire slave boilerplate shared with the sensing board.

A step is emitted only when `motor()` asks for one, and every emitted half-step calls `countStep()` once. One contract step is one half-step. The coils are released whenever the belt is not stepping. Do not reset the count on stop or reversal.

## Wiring and settings

| Item | Value |
| --- | --- |
| Belt motor | ULN2003 IN1 to IN4 on D4, D5, D6, D7 |
| Step unit | half-step, 4096 per revolution |
| `MM_PER_HALF_STEP` | 0.0307 from the standalone sketch; re-measure with `c` |
| Belt speed | 15 mm/s fixed; the driver caps at about 1000 half-steps per second |
| Direction | flip `FORWARD_DIRECTION` if the belt runs backwards |

## Bench keys

Serial at 9600. `g`, `b` and `s` send the same commands the master sends and pass the same guards. `c` runs exactly 4096 half-steps then stops; measure the travel and set `MM_PER_HALF_STEP` to travel in millimetres divided by 4096. Do not type bench keys while the master is running a cycle.

## States keyed to commands

```mermaid
stateDiagram-v2
    [*] --> STOPPED: boot
    STOPPED --> FORWARD: g
    STOPPED --> REVERSE: b
    FORWARD --> FORWARD: g / no restart
    REVERSE --> REVERSE: b / no restart
    FORWARD --> STOPPED: s
    REVERSE --> STOPPED: s
    STOPPED --> STOPPED: s
    STOPPED --> FAULT: contact lost
    FORWARD --> FAULT: contact lost / stop output
    REVERSE --> FAULT: contact lost / stop output
    FAULT --> FAULT: commands / retain fault
    FAULT --> STOPPED: reads resume and reason was contact lost
    FAULT --> [*]: LOCAL fault, board reset required
```

Direct reversal is rejected: the master confirms STOPPED before the opposite command. There is no speed or reset command. Existing Serial `g/s/f/l` may remain in an explicit standalone bench mode; it must not compete with master commands, and bus loss must not select bench mode.

## Verification

Demonstrate both directions, responsive stop, repeated-command behaviour, rejected direct reversal, continuous step count, contact timeout, recovery to STOPPED when reads resume, and no motion without a new command. Measure stop latency under load and half-steps per millimetre on the assembled belt with the `c` key. No encoder or stall detection is assumed.
