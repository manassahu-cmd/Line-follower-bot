# 🤖 AUTOBOT — AXIS'26 | VNIT Nagpur

> Autonomous Line-Following & Maze-Solving Robot built for the AXIS'26 robotics competition at VNIT Nagpur.

---

## Components Used

| Component | Specification |
|-----------|---------------|
| Microcontroller | ESP32 WROOM DevKit V1 — 30-pin |
| IR Sensor Array | Pololu QTR-8A — 8-channel Analog |
| Motor Driver | TB6612FNG Dual H-bridge |
| Drive Motors | 2× N20 Micro Gear Motor |
| Wheels | Rubber wheels matched to N20 shaft |
| Battery | 3S LiPo — 800 mAh |
| Buck Converters | 2× DC-DC step-down modules |
| Build | All components soldered on a custom PCB |

---

## System Architecture

```
┌─────────────────────────────────────────────────┐
│                   AUTOBOT FLOW                  │
└─────────────────────────────────────────────────┘

  [QTR-8A 8-channel Analog IR]
         │
         │  8 analog readings (0–4095 per sensor)
         ▼
  [ESP32 — Decision Core]
         │
         ├── Calibrated thresholds → BLACK or WHITE per sensor
         │
         ├── countActive()    → how many sensors see black
         ├── leftActive()     → edge sensors S6, S7 on line
         ├── rightActive()    → edge sensors S0, S1 on line
         ├── centerActive()   → center sensors S3, S4 on line
         │
         ├── isDeadEnd?  → all sensors white  → pivot to recover
         ├── isJunction? → edge + center both active → decide turn
         └── else        → followLinePID()
                                │
                                ▼
                    [TB6612FNG Motor Driver]
                         │           │
                    [N20 Left]   [N20 Right]
```

### Sensor Zone Layout

```
  [S7][S6]  [S5][S4][S3][S2]  [S1][S0]
  LEFT EDGE     CENTER ZONE    RIGHT EDGE
```

The 8 sensors together give a positional reading from 0 (far right) to 7000 (far left), with 3500 being perfectly centered on the line.

---

## The Journey — From PID Setup to Priority-Based Running

### Step 1 — Calibration

Before the bot moves an inch, it sweeps itself left and right over the track surface for ~4 seconds, recording the minimum and maximum analog value each sensor sees. From this, a per-sensor threshold is computed as the midpoint between the two extremes. Everything above the threshold = black line; below = white surface. This ensures reliable detection regardless of track color variation or ambient lighting.

---

### Step 2 — PID Line Following

Once calibrated, the bot computes a weighted position of the line across all 8 sensors:

```
position = weighted average of active sensor indices × 1000
error    = 3500 − position
```

A positive error means the robot has drifted right; negative means left. The PID controller then calculates a correction output:

```
output = Kp × error  +  Ki × integral  +  Kd × derivative
```

This output is added to the left motor and subtracted from the right (or vice versa), smoothly steering the robot back to center without jerky movements.

**Tuning was done in this order:**

- **Kp first** — increased until the bot followed the line but started oscillating (wiggling).
- **Kd next** — increased to dampen that oscillation and make the response crisp.
- **Ki last and tiny** — added only to correct a persistent one-sided drift; too much causes slow wobble.

Final values settled at `Kp = 0.052`, `Ki = 0.0001`, `Kd = 0.416` for maze mode and slightly softer values for the high-speed line follow round.

---

### Step 3 — Junction Detection & Priority Selection

Pure PID handles straight lines and gentle curves well, but at intersections the bot needs to make a conscious decision. A junction is detected when the edge sensors (left or right) AND the center sensors are simultaneously active — meaning paths branch off while the main line continues.

Once a junction is identified, the bot nudges forward slightly to fully clear the intersection body, then re-reads the sensors and applies the selected priority:

| Priority Mode | Decision Order |
|---------------|---------------|
| Straight first | S → L → R |
| Left first | L → S → R |
| Right first | R → S → L |

The turn itself is a two-phase pivot: the bot first rotates away from the incoming line until the center sensor goes white, then continues rotating until it finds the exit line (center goes black again). This cleanly handles all junction geometries — T-junctions, crossroads, and angled branches — without false triggers.

---

### Step 4 — Maze Solving (Round 2)

For the maze round, the priority logic was extended into a full explore-then-replay system.

**Dry Run:** The bot explores the maze using Left-first (LSRB) or Right-first (RSLB) priority, recording every decision (`L`, `R`, `S`, `U`) into a path array. After each U-turn, an optimizer checks if the last three moves can be collapsed into a single equivalent turn using angle arithmetic — dead ends are erased from the path in real time.

**Final Run:** With the optimized path stored, the bot replays it decision-by-decision at full speed, ignoring the priority logic entirely and following the pre-computed shortest route straight to the finish.

The finish line is a wide black bar that activates six or more sensors at once — the bot detects this after its junction nudge and halts, blinking both LEDs to signal completion.

---

*Built for AXIS'26 — VNIT Nagpur*
