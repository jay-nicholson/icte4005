# Shared contract — v3 design

This file defines the external interface. `shared/contract.h` carries the same numbers as named constants for all three boards; change both together. Subsystem READMEs explain local work; they do not override these rules. Use Arduino Wire without another messaging library, CRC, sessions or command IDs.

## Ownership

Slaves report what they measure and what they are doing. The master gives those readings their meaning. No slave holds process state, so no slave is told about the process.

| Board | Owns | Reports | Accepts |
| --- | --- | --- | --- |
| Conveyor | Stepping, the ejector paddle, cumulative step count, its contact watchdog | Motion state, fault, uptime, steps | `g`, `s`, `e`; `b` for bench use |
| Sensing | Debounce, cumulative edge count per channel, its contact watchdog | Health, fault, uptime, live mask, two edge counts | Nothing |
| Master | Every phase, baseline, allowance, acknowledgement and system fault; operator Serial; the only andon | Serial report and andon | `start`, `stop`, `resume`, `ack` |

## Agreed operation

Load one puck before channel 1. Run forward. A channel 1 edge records arrival; a channel 2 edge completes magnetic detection. Stop and wait for operator acknowledgement. Then run forward a calibrated distance to the ejector paddle, eject the puck with one paddle sweep and stop. Prepare the next cycle only after both channels read open.

No channel activity before arrival is normal. There is no no-detection deadline or non-magnetic classification. Operator Stop pauses the current phase. Resume preserves the phase and travel progress. Missing an expected channel causes a latched master FAULT. Remove all pucks and reset the master to clear it. Reset a slave only if it shows LOCAL. Never resume a faulted cycle.

## Wire format

The master initiates every transaction. A command is exactly one ASCII byte, without newline or NUL. Status is a separate read. Addresses are 7-bit. Unknown commands, wrong-length writes and failed guards have no effect. An ACK confirms delivery, not action completion. Confirm actions by polling.

| Board | Address | Reply length | State byte values |
| --- | --- | --- | --- |
| Conveyor | `0x10` | 10 bytes | 0 STOPPED, 1 FORWARD, 2 REVERSE, 3 FAULT, 4 SWEEPING |
| Sensing | `0x11` | 15 bytes | 0 OK, 1 FAULT |

| Offset | Field | Definition |
| --- | --- | --- |
| 0, both | state: uint8 | Board-specific value above. |
| 1, both | fault: uint8 | 0 NONE, 1 CONTACT_LOST, 2 LOCAL. |
| 2–5, both | uptime_ms: uint32 | Main-loop snapshot of milliseconds since boot. |
| 6–9, conveyor | issued_steps: uint32 | Total emitted library steps in either direction since boot. |
| 6, sensing | live_mask: uint8 | Debounced input bits: bit 0 channel 1, bit 1 channel 2. Other bits zero. |
| 7–8, sensing | channel1_edges: uint16 | Debounced closing edges on channel 1 since boot. |
| 9–10, sensing | channel2_edges: uint16 | Debounced closing edges on channel 2 since boot. |
| 11–12, sensing | channel1_level: uint16 | Diagnostic: channel 1 signal level in the board's own units. The master only displays it. |
| 13–14, sensing | channel2_level: uint16 | Diagnostic: channel 2 signal level. The master only displays it. |

Multi-byte fields are little-endian. Send numeric bytes, not ASCII digits. Serialize explicitly; do not transmit a C++ struct. Publish coherent snapshots from the main loop at least every 20 ms. FAULT requires a nonzero fault; healthy states require zero. Preserve the first fault reason.

Counters only increase and wrap naturally. The master compares them with saved baselines using unsigned subtraction and keeps every limit below half the counter range. Step delta divided by calibrated steps per millimetre estimates travel; it is not position feedback. Slips, stalls and actual ejection are not verified.

## Conveyor commands

| Command | Legal behaviour |
| --- | --- |
| `g` | STOPPED → FORWARD. Already FORWARD: no-op. REVERSE or FAULT: ignore. |
| `b` | STOPPED → REVERSE. Already REVERSE: no-op. FORWARD or FAULT: ignore. |
| `s` | Stop issuing steps. STOPPED, FORWARD or REVERSE → STOPPED. SWEEPING and FAULT: ignore. |
| `e` | STOPPED or FORWARD → SWEEPING: stop the belt and run one paddle sweep out and back. REVERSE, SWEEPING or FAULT: ignore. Back to STOPPED when the paddle has returned. |

A direction change requires confirmed STOPPED first. The master never sends `b` in v3; it stays for bench use. No speed command exists. Standalone Serial bench controls are ignored while a master is in contact, so they cannot compete with master commands.

## Slave watchdog and faults

After the first status read, no status read for 1,000 ms causes a local FAULT with CONTACT_LOST: the conveyor stops stepping and reports FAULT, the sensing board reports FAULT. When reads resume, the board clears CONTACT_LOST on its own: the conveyor returns to STOPPED and never resumes motion by itself; the sensing board returns to OK. This lets a master reset recover the system. LOCAL covers any fault the board detects in its own hardware; it latches until that board is reset. The master never writes a fault into a slave.

## Master rules

1. Boot: request stop. Require fresh conveyor STOPPED, sensing OK and both channels open before READY. Absent slaves wait without fault.
2. Poll sensing then conveyor every 100 ms in every master state.
3. Send a motion command once. Confirm it only from a reply received after it was sent, within 300 ms. Retry only `s`, once per poll, until confirmed. Never extend the original deadline. `e` is confirmed by SWEEPING within 300 ms; the sweep must end in STOPPED within 6,000 ms.
4. Mark feedback UNKNOWN after 300 ms without a valid reply. Invalid length, values or reserved bits cause FAULT during a cycle; in INITIALISING, READY or BENCH they only block progress, because a bus being wired produces them. A counter that decreases without a detected reboot counts as invalid. UNKNOWN during a cycle causes FAULT; UNKNOWN in the idle states only blocks progress. A slave reporting CONTACT_LOST in the idle states also only blocks progress, because the next reads clear it; any other slave fault, in any state, causes FAULT.
5. Fault if returned uptime does not advance for 300 ms. A slave reboot during a cycle causes FAULT; in INITIALISING or READY it returns the master to INITIALISING.
6. Start saves both channel edge counts. Arrival is a channel 1 count above that baseline. Magnetic is a channel 2 count above its baseline. A channel 2 rise with no channel 1 rise causes REED_ORDER. Both rising in the same poll counts as magnetic.
7. At arrival, save the step baseline from the same poll. Apply the forward allowance to reach channel 2. Exceeding it without the expected event causes FAULT in every state, including paused. An expected event seen in the same poll wins over the limit.
8. Acknowledge is accepted whatever the live channels read; it saves the approach step baseline and sends `g`. When the step delta reaches the calibrated paddle distance, send `e`. Confirm SWEEPING, wait for STOPPED, then require both channels open within 300 ms. A held channel causes FAULT. Then READY.
9. Stop saves the active phase and sends `s` each poll until STOPPED is confirmed. Then the destination is WAIT_ACK if magnetic, EJECTING if the approach distance is complete, otherwise PAUSED. Stop during a sweep is refused; the sweep is short and the belt is already stopped. Resume re-evaluates the same facts before requesting motion. Baselines survive pauses.
10. Stop has priority over any other operator command received in the same poll interval.
11. On system fault: send `s` each poll until a post-fault reply confirms STOPPED, show red on the master andon, latch, and report STOP UNCONFIRMED until the confirmation arrives. No command clears a master fault.
12. Full reset means stop the drive as needed, remove all pucks and reset the master. Reset a slave only if it reports LOCAL.

Timing values are bench defaults, not safety guarantees. Software stop is not an emergency-stop system. Step allowances are based on emitted motion; they cannot diagnose a motor that receives steps but does not move.

## Andon

The master drives the only andon. Amber blinking in INITIALISING while it waits for the slaves. Off in READY and BENCH. Green while a cycle is moving or stopping. Amber steady in WAIT_ACK and PAUSED. Red in FAULT, including a slave fault. Slaves have no status lights.

## Normal sequence

```mermaid
sequenceDiagram
    actor O as Operator
    participant M as Master
    participant S as Sensing
    participant C as Conveyor
    O->>M: Load / Start
    Note over M,S: Save channel edge baselines
    M->>C: g / confirm FORWARD
    S-->>M: channel 1 edges rose
    Note over M,C: Save step baseline
    S-->>M: channel 2 edges rose
    M->>C: s / confirm STOPPED
    O->>M: Acknowledge
    Note over M,C: Save approach step baseline
    M->>C: g / confirm FORWARD
    Note over M,C: Step delta reaches the paddle distance
    M->>C: e / confirm SWEEPING
    C-->>M: STOPPED after the sweep
    S-->>M: both channels open
    Note over O,M: Ready for next puck
```

## Calibration and integration inputs

Measure fixed speed, step unit, steps/mm, channel spacing, the forward allowance, the stop-to-paddle distance, the sweep duration, debounce, sample interval and stopping/polling margin. Master-observed events are delayed by polling; account for this in calibration. Never reset a baseline on repeated replies.

The conveyor drives a 28BYJ-48 through a ULN2003 by direct half-stepping; one contract step is one half-step and the coils are released when stopped. Confirm channel/LED wiring and polarity. Use common ground, Uno SDA/A4 and SCL/A5, suitable pull-ups and a compatible motor driver/supply. Never drive coils from GPIO. Screen, buzzer, Teleplot formatting and speed control are stretch work. Actual ejection remains unverified without an additional sensor; both channels reading open after the sweep is the only evidence.
