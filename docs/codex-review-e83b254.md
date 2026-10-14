# Adversarial firmware review

Repository: `/Users/jaynicholson/Projects/icte4005`  
Reviewed commit: `e83b254b30f7e31cb774b72cc83d4b1b06591ca7` (`main` at the time of review)  
Review date: 13 October 2026

This report applies only to that reviewed commit. The repository has since moved on; findings and file:line references describe the reviewed snapshot, not the current checkout. Later changes have not been reviewed here.

I would block hardware integration on findings 1–5 and 9–13.

Both requested host suites ran first: **45 master tests and 20 sensing tests passed**. PlatformIO initially hit sandbox restrictions; successful runs used a temporary repository copy. Tracked files were unchanged; the pre-existing untracked `handover.md` remained.

**Confirmed** means a running host probe demonstrated the behavior, or—where explicitly stated—source inspection established a standards/test issue. Hardware timing probes use the actual firmware functions with Arduino/Wire stubs, not physical Unos. Physical consequences that were not tested on hardware are marked plausible. Probe sources and executables remain in `/tmp/icte-review.Kz0LNa`: `probe.cpp`, `hardware_probe.cpp`, `operator_probe.cpp`, `edge_probe.cpp`, and `boot_probe.cpp`.

## Findings

### 1. Alternating missed polls disable travel limits indefinitely

- **Severity:** High
- **Status:** Confirmed by running process probes
- **Location:** `master/lib/process/process.cpp:146`, `master/lib/process/process.cpp:172`

Both slaves must reply in the same poll before the master saves an arrival or overrun baseline. State transitions do not require that baseline.

**Reproduction:** Confirm forward motion, receive a reed 1 edge while the conveyor read fails, then alternate successful conveyor and sensing reads. Each slave replies every 200 ms, so neither becomes UNKNOWN. The master remains TRACKING with no entry baseline. The probe reached 1,100 issued steps against a 100-step allowance without faulting. The corresponding reverse sequence remained OVERRUN indefinitely because no overrun baseline existed.

Even a short interruption loses progress: arrival during a failed conveyor read, Stop during the next failed sensing read, then successful stop confirmation saved the entry baseline at the stopped position. `travel()` returned zero despite intervening travel. This violates contract rule 7 and the promise that pauses preserve progress.

### 2. Stop already buffered alongside Start does not get priority

- **Severity:** High
- **Status:** Confirmed by running the actual master loop and Serial parser with stubs
- **Location:** `master/src/operator_serial.cpp:47`, `master/src/main.cpp:110`

**Reproduction:** In READY, place `start\nstop\n` in the Serial receive buffer immediately before a due poll. `serialPoll()` returns after parsing Start; `loop()` immediately polls and sends `g`. Stop remains unread. The probe sent `s` at the next poll, 100 ms later.

The stop-priority flag works only for commands already parsed. Contract rule 10 applies to commands received in the interval, including the Stop already waiting in the UART buffer.

### 3. A faulted master stops enforcing rest after one stop confirmation

- **Severity:** High
- **Status:** Confirmed by running a process probe
- **Location:** `master/lib/process/process.cpp:272`

**Reproduction:** Trigger REED_ORDER, return a post-fault STOPPED reply, then return FORWARD with increasing steps. The master remains FAULT but sends no further stop commands and leaves `stopUnconfirmed()` false.

This combines directly with finding 11: a conveyor Serial `g` can restart the belt after fault-stop confirmation. Continued master polling keeps the conveyor watchdog satisfied, so motion can continue indefinitely under a red andon.

There is also a literal contract mismatch at line 274: conveyor FAULT is accepted as stop confirmation, whereas rule 11 specifies STOPPED. FAULT currently implies motor-off in conveyor logic, but the contract should describe what the master accepts. This secondary mismatch was established by inspection rather than a separate probe.

### 4. BENCH bypasses mandatory fault handling and stop confirmation

- **Severity:** High
- **Status:** Confirmed by running process probes
- **Location:** `master/lib/process/process.cpp:282`

**Reproduction:** Enter BENCH, report sensing LOCAL, then issue `send c g`. The master sends `g`, stays BENCH, and leaves the andon off. Invalid state bytes, stalled uptime and reboot notifications also bypass `checkFeedback()` entirely.

A BENCH Stop emits one `s`; failure to act causes neither retry nor confirmation timeout. This contradicts rules 3–5: BENCH exempts UNKNOWN and CONTACT_LOST from faulting, not invalid replies, LOCAL or uptime stalls.

### 5. Missed polls erase expired deadlines and feedback-loss intervals

- **Severity:** High
- **Status:** Confirmed by running process probes
- **Location:** `master/lib/process/process.cpp:95`, `master/lib/feedback/feedback.cpp:118`, `master/lib/process/process.cpp:254`

**Reproductions:**

1. Send `g`; make the next poll occur 500 ms later with FORWARD feedback. Confirmation succeeds because matching state is checked before the expired 300 ms deadline.
2. Send `s`; return STOPPED 500 ms later. The master enters PAUSED without STOP_UNCONFIRMED.
3. In CLEARING with a held reed, skip 500 ms and return clear reeds. The master enters READY rather than REED_HELD.
4. After 600 ms without polling in READY, return valid replies and Start. New replies overwrite the evidence of the gap and motion starts.

These violate rules 3, 4 and 8. Receiving good feedback now does not establish that confirmation or clearance occurred before the original deadline.

### 6. Contradictory STOPPED replies can conceal motion and corrupt progress

- **Severity:** Medium
- **Status:** Confirmed by running a process probe
- **Location:** `master/lib/process/process.cpp:190`, `master/lib/feedback/feedback.cpp:90`

**Reproduction:** Reach PAUSED with confirmed STOPPED. Keep reporting STOPPED while increasing `issued_steps` by 100. The master accepts the replies, remains PAUSED and subsequently sends `g` on Resume.

State and monotonic counters are validated independently, but steps are never checked for remaining unchanged across stopped observations. Actual movement between polls—for example, competing bench commands—can consume allowances or overrun distance without producing UNEXPECTED_MOTION.

### 7. Some slave reboots are undetectable under the current protocol

- **Severity:** High
- **Status:** Confirmed by running a process/feedback probe; protocol limitation
- **Location:** `master/lib/feedback/feedback.cpp:89`

**Reproduction:** Start a cycle with sensing uptime 220 ms and both counters zero. Reset sensing shortly afterwards and miss polls until 250 ms after the previous reply. The rebooted slave can legitimately report uptime 230 ms and zero counters. The master sees increasing uptime, unchanged counters and no reboot; the probe remained FORWARD without fault.

Rule 5's unconditional “slave reboot during a cycle causes FAULT” is stronger than this interface can guarantee. Uptime rollback detects many reboots, not all. The prohibition on sessions/boot identifiers makes this an explicit contract limitation.

### 8. Calibration accepts limits forbidden by the contract

- **Severity:** Medium
- **Status:** Confirmed by running a process probe
- **Location:** `master/lib/process/process.h:74`

**Reproduction:** Supply `{0x80000000, 0xffffffff, 0xffffffff}` as calibration. `ready()` returns true and Start sends `g`.

The contract requires every limit below half the counter range. Only nonzero values are enforced. In particular, a `UINT32_MAX` allowance can never be exceeded by a `uint32_t` delta.

### 9. An unknown command can overwrite a queued Stop

- **Severity:** High
- **Status:** Confirmed by running actual SlaveLink callbacks with Wire stubs
- **Location:** `shared/slave_link.h:46`

**Reproduction:** While conveyor logic is FORWARD, invoke the receive callback for one-byte `s`, then one-byte `?`, before `takeCommand()`. The mailbox returns `?`; conveyor logic ignores it and keeps moving.

Unknown commands therefore have an effect: they destroy a valid pending command. This violates the wire contract. The mailbox also collapses multiple meaningful commands into the last byte rather than preserving their guards and ordering.

### 10. A successful status read can manufacture CONTACT_LOST

- **Severity:** Medium
- **Status:** Confirmed by running a SlaveLink timestamp-interleaving probe; interrupt occurrence on hardware remains plausible
- **Location:** `shared/slave_link.h:37`, `conveyor/src/main.cpp:158`, `sensing/src/main.cpp:52`

**Reproduction:** Main captures `now = 100`. Before it calls `contactLost(now)`, the Wire ISR records a read at 101. Unsigned `100 - 101` becomes `UINT32_MAX`, so contact is declared lost immediately.

The conveyor stops spuriously. If that iteration publishes, CONTACT_LOST can become a latched master cycle fault. Protecting the timestamp copy does not fix comparing it against an older caller timestamp.

### 11. Conveyor Serial controls compete with master commands in normal firmware

- **Severity:** High
- **Status:** Confirmed by running the actual conveyor loop with stubs
- **Location:** `conveyor/src/main.cpp:130`, `conveyor/src/main.cpp:159`

**Reproduction:** Queue Wire `s` and Serial `g` before one loop. The loop applies `s`, immediately applies `g`, and issues a step. The probe finished FORWARD.

There is no standalone-mode guard or ownership arbitration. The comment asking operators not to type is the only protection. This directly violates the conveyor-command contract and enables finding 3.

The Serial `c` command also arms `calibrationRemaining` even when its `g` is rejected. The hidden countdown can later stop unrelated motion. This secondary countdown behavior was established by inspection, not a separate execution probe.

### 12. The half-step scheduler emits compressed catch-up steps

- **Severity:** High
- **Status:** Confirmed by running the actual motor service with stubbed time; rotor consequences remain plausible
- **Location:** `conveyor/src/main.cpp:62`

**Reproduction:** Set the real 2,046 µs interval and `lastStepMicros = 0`. Service at 4,091 µs, then 4,092 µs. Both calls emit and count a half-step: **1 µs spacing**.

The first call advances the scheduled timestamp only to 2,046. Its resynchronisation test misses being almost a full interval late. The next call immediately emits another step. This contradicts “no burst” and bypasses the intended rate cap. Whether the rotor follows these compressed transitions was not hardware-confirmed; the emitted-step count nevertheless credits both.

### 13. LOCAL arising during CONTACT_LOST is discarded rather than latched

- **Severity:** High
- **Status:** Confirmed by running sensing-logic calls
- **Location:** `sensing/lib/sensing_logic/sensing_logic.cpp:33`

**Reproduction:** Call `serviceWatchdog(true)`, `raiseLocalFault()`, then `serviceWatchdog(false)`. The resulting state is OK/NONE without reset.

Preserving the first displayed reason must not discard the existence of a later LOCAL fault. In actual sensing firmware, a persistent analog fault is eventually raised again, but read recovery can publish an intervening healthy snapshot because local-fault processing precedes watchdog clearing. A transient local condition is lost completely.

### 14. Debug output can violate the 20 ms snapshot guarantee and create sampling blind spots

- **Severity:** Medium
- **Status:** Plausible; UART implementation inspected and line length calculated, no hardware timing probe
- **Location:** `sensing/src/main.cpp:70`

**Reproduction:** With large edge counts and CONTACT_LOST, the enabled debug line can reach 87 bytes, for example:

```text
hall1=511 hall2=511 light=993 live=3 edges=65535/65535 state=FAULT fault=CONTACT_LOST
```

Including CR/LF, this is 87 bytes. At 9,600 baud, even allowing approximately 65 bytes of initial buffering, writing the remainder takes about 23 ms. No sampling or publishing occurs during that blocking print. The installed AVR HardwareSerial implementation uses a 64-byte transmit ring and blocks when it fills.

This exceeds the contract's maximum snapshot interval and the configured 2 ms sampling interval. Pulses inside the blind interval can be missed. Existing host models omit UART blocking entirely.

### 15. SlaveLink does not preserve the caller's interrupt state

- **Severity:** Low
- **Status:** Confirmed with interrupt-state stubs; latent in current call sites
- **Location:** `shared/slave_link.h:23`, `shared/slave_link.h:31`, `shared/slave_link.h:37`

**Reproduction:** Disable interrupts, then call `takeCommand()`. Interrupts are enabled on return. `publish()` and `contactLost()` have the same pattern.

Current callers are ordinary main-loop code, so no present nested-ISR failure was established. The shared API cannot safely be called inside an existing critical section; it should restore the previous interrupt state.

### 16. Sensing reports healthy, clear inputs before the front end is ready

- **Severity:** High
- **Status:** Confirmed by running the actual sensing loop with stubs
- **Location:** `sensing/src/main.cpp:58`, `sensing/lib/sensing_logic/sensing_logic.cpp:7`

**Reproduction:** Run the sensing loop for 220 ms with normal ADC values. Hall baseline capture is still incomplete, but the Wire snapshot reports OK, live mask zero and zero edges.

The master cannot distinguish “both channels measured open” from “neither channel has completed zeroing.” A master starting or resetting at a different time can use these replies to authorise motion during calibration. Delayed sampling extends the window.

### 17. A stable magnetic field at boot becomes the baseline; removing it creates a false arrival

- **Severity:** Medium
- **Status:** Confirmed by running analog-channel and sensing-logic probes
- **Location:** `sensing/lib/analog_channel/analog_channel.cpp:31`

**Reproduction:** Zero with 128 samples at 560. Baseline 560 passes plausibility and spread checks. Holding 560 reports inactive and healthy. Returning to the normal 512 level produces a debounced closing edge and remains active.

A real puck can be invisible while present, then “arrive” when removed. The README's clear-at-boot instruction acknowledges the precondition; firmware does not enforce it. The same fixed-baseline mechanism turns sufficiently large later drift into an event rather than a health fault.

### 18. Rail faults first produce valid-looking edges

- **Severity:** Medium
- **Status:** Confirmed by running analog-channel and sensing-logic probes
- **Location:** `sensing/lib/analog_channel/analog_channel.cpp:24`

**Reproduction:** After normal zeroing, feed 1023 at 2 ms intervals. By sample 31, sensing logic has counted a closing edge while `fault()` is still false; the rail fault requires 50 consecutive samples.

A short-to-rail lasting less than the fault threshold can be reported solely as arrival or magnetic detection. A longer short can cause process transitions before LOCAL appears. The existing rail test checks fault timing but never checks the edge count during that window.

### 19. The optional light gate accepts brightening and can duplicate detection

- **Severity:** Medium
- **Status:** Confirmed input traces; occurrence from real lighting remains plausible
- **Location:** `sensing/lib/analog_channel/analog_channel.cpp:17`, `sensing/lib/analog_channel/analog_channel.h:45`

**Reproductions:**

1. Zero the light channel at 600, then feed 750. Absolute deviation declares presence, just as shading does. A stray Hall field plus ambient brightening passes the gate.
2. Hold Hall 2 continuously active while alternating light samples between 450 and 600 for 200 ms segments. One continuous Hall detection produces two channel-2 edges.

The gate detects departure from boot illumination, not specifically puck presence. It is disabled by default, reducing current exposure.

### 20. “One edge per pass” depends on the assumed unipolar field shape

- **Severity:** Medium
- **Status:** Confirmed input trace; occurrence with the physical magnet arrangement remains plausible
- **Location:** `sensing/lib/analog_channel/analog_channel.cpp:58`, `sensing/test/test_analog_channel/test_main.cpp:26`

**Reproduction:** After zeroing at 512, apply 572, then 512, then 452 for 200 ms each. The positive and negative lobes produce two closing edges.

Absolute deviation makes either polarity detectable, but does not merge lobes separated by a release interval into one passage. Existing pass tests only use a single positive or negative Gaussian bump. An extra forward reed-1 edge arriving late can subsequently satisfy the reverse “new reed 1 edge” condition.

### 21. Tests encode a contract violation and omit the layers responsible for several failures

- **Severity:** Medium
- **Status:** Confirmed by source inspection, passing suites and independent probes
- **Locations and reproductions:**

1. `master/test/test_process/test_main.cpp:320`: deliberately injects invalid state 9 and asserts BENCH remains BENCH. Run the suite: it passes. That asserts the wrong behavior under contract rule 4.
2. `sensing/test/test_analog_channel/test_main.cpp:124`: checks CONTACT_LOST remains the first reason after LOCAL, but never resumes contact and checks that LOCAL remains latched. Run the suite, then the finding-13 probe: the suite passes while the fault disappears.
3. `master/test/test_process/test_main.cpp:354`: endorses a delayed overrun baseline after missed reads. It never tests indefinitely alternating reads or measures extra travel discarded before baseline capture. The suite passes while finding 1 reproduces.
4. `master/test/test_end_to_end/test_main.cpp:54` and `sensing/test/test_hall_end_to_end/test_main.cpp:87`: encode snapshots immediately, deliver commands synchronously, and replace the motor scheduler with one step per millisecond. SlaveLink, Serial arbitration, publication delay and half-step timing failures cannot fail these tests because those implementations are not executed.
5. `sensing/test/test_hall_end_to_end/test_main.cpp:171`: the test named “runs until operator stop” never issues Stop or verifies stopping. Run it: the asserted behavior ends with continuing forward motion.

### 22. Project identifier and comment standards are violated

- **Severity:** Low
- **Status:** Confirmed by inspection; execution probe not applicable
- **Locations and reproduction:**

Inspect `i` at `master/lib/process/process.cpp:126`, `ret` at `master/test/test_end_to_end/test_main.cpp:30`, and `ms`, `i`, `w`, `s` at `sensing/test/test_hall_end_to_end/test_main.cpp:124` and `:130`. These conflict with “no abbreviated identifiers.”

Comments at `shared/slave_link.h:22`, `sensing/lib/sensing_logic/sensing_logic.cpp:14`, and `sensing/test/test_analog_channel/test_main.cpp:110` describe directly visible behavior rather than non-obvious reasoning, conflicting with “comments only where non-obvious.”

## Review limits

No hardware upload, electrical integration, rotor response, actual Hall field shape or measured lighting behavior was tested. Confirmed host failures stand independently of those unverified hardware effects. The shipped master calibration values are zero and deliberately refuse Start; motion probes supplied nonzero calibration. This expected calibration gate was not reported as a defect.

## What did not break

- **Natural uptime, issued-step and reed-edge counter wrap:** accepted correctly in explicit probes. The step-wrap probe started with a legitimate near-wrap initial counter rather than injecting an invalid half-range jump.
- **Ordinary pause/resume with uninterrupted polls:** existing tests preserve progress.
- **Expected events arriving in the same poll as allowance exhaustion:** existing tests confirm the event wins.
- **Ordinary counter rollback, detectable reboot, invalid masks and stalled uptime:** existing tests fault correctly.
- **Normal contact-loss recovery and LOCAL-first fault preservation:** existing tests pass. This does not cover LOCAL arising second, as in finding 13.
- **Snapshot byte copying:** main-loop publication is interrupt-protected, and the installed AVR Wire implementation copies the reply into its transmit buffer. No torn-frame reproduction was found.
