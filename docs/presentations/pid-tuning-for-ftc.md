---
marp: true
theme: uncover
class: invert
paginate: true
style: |
  section {
    font-family: 'Segoe UI', Arial, sans-serif;
  }
  code {
    font-size: 0.75em;
  }
  h1 {
    color: #4FC3F7;
  }
  h2 {
    color: #81D4FA;
  }
  strong {
    color: #FFD54F;
  }
  em {
    color: #A5D6A7;
  }
  .columns {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 1rem;
  }
  table {
    font-size: 0.8em;
  }
  .red { color: #EF5350; }
  .green { color: #66BB6A; }
  .yellow { color: #FFD54F; }
  .blue { color: #42A5F5; }
  .small { font-size: 0.7em; }
---

<!-- _class: invert -->

# PID Tuning for FTC 🎯
## Making Your Shooter Accurate & Your Robot Precise

**BBTC Robotics — 2025 Off-Season Workshop**

---

# What We'll Cover Today

1. 🔍 **Your Shooter Today** — What those mystery numbers do
2. 📐 **What is PID?** — The concept behind the math
3. 🔧 **Your Shooter's PIDF** — Mapping theory to your code
4. 🎮 **Tuning Step-by-Step** — Hands-on with FTC Dashboard
5. 🚗 **Drive Controller Tuning** — RoadRunner gains explained
6. 📦 **Consolidating Values** — Stop copy-pasting numbers!

---

<!-- _class: invert -->

# Part 1
## 🔍 Your Shooter Today

---

# The Mystery Numbers in Your Code

You have **at least 5 different PIDF settings** scattered across your code:

| File | P | I | D | F | TPS |
|------|---|---|---|---|-----|
| `BaseTeleOp` Long Shot | 75 | 0 | 0 | 6.5 | 1000 |
| `BaseTeleOp` Short Shot | 28 | 0 | 0 | 10.5 | 900 |
| `BaseTeleOp` Mid Shot | 28 | 0 | 0 | 13 | 900 |
| `BaseTeleOp` Emergency | 80 | 0 | 0 | 20 | 1000 |
| `BaseAutonomus` Far Shot | 75 | 0 | 0 | 6.5 | 1000 |
| `BaseAutonomus` Close Shot | 28 | 0 | 0 | 9 | 900 |
| `MecanumDriveClose` Goal | 30 | 0 | 0.25 | 10.5 | 900 |
| `MecanumDriveClose` Straight | 65 | 0 | 0 | 6.5 | 1000 |

**Question:** Can anyone explain what P=75, F=6.5 actually means? 🤔

---

# What Happens With These Numbers?

Your code calls this every loop:

```java
shooter_1.setVelocityPIDFCoefficients(75, 0.0, 0.0, 6.5);
shooter_1.setVelocity(1000);  // target: 1000 ticks per second
```

The motor controller uses these 4 numbers to decide **how much power** to send to the motor.

**Without understanding what P, I, D, and F do**, you're just guessing numbers until it "works." 

Let's fix that. 💡

---

# A Real-World Analogy 🚿

Imagine you're adjusting a **shower temperature**:

- The water is **currently 60°F** (cold!)
- You **want 100°F** (perfect)
- The **error** = 100 - 60 = **40°F**

How do you adjust the knob?

That's exactly what a PID controller does — but for your **shooter motor's velocity**.

---

<!-- _class: invert -->

# Part 2
## 📐 What is PID?
### (Actually PIDF — there are 4 terms)

---

# The 4 Letters: P, I, D, F

Each letter is a **strategy** for reducing error:

| Letter | Name | Plain English |
|--------|------|--------------|
| **F** | Feedforward | "I know roughly how much power this needs" |
| **P** | Proportional | "Push harder when I'm far away, softer when I'm close" |
| **I** | Integral | "If I've been off for a while, push a little more" |
| **D** | Derivative | "I'm approaching fast — ease off so I don't overshoot" |

Let's understand each one. We'll start with **F** because it's the most important for motors.

---

# F = Feedforward 🎯

**"I know roughly how much power this needs to reach the target speed."**

```
Without F:   Motor starts at 0 power, PID has to figure out everything
             Result: Slow start, lots of correction needed

With F:      Motor immediately gets ~80% of needed power
             Result: Fast start, PID only needs small corrections
```

**In your code:** `F = 6.5` means:
```
Motor power = F × target velocity = 6.5 × (1000 / maxTPS)
```

F gets you **close to the answer**. P, I, D handle the fine-tuning.

---

# How to Think About F

Imagine you need to throw a ball to someone 20 feet away.

```
Without Feedforward:
  You: "Hmm, how hard? Let me try..."  → too short
  You: "A little more..."              → too short  
  You: "More..."                       → still short
  You: "MORE..."                       → overshot!!
      (This is P alone — slow and oscillating)

With Feedforward:
  You: "20 feet? I know roughly how hard that is."  → close!
  You: "Just a tiny adjustment..."                   → perfect!
      (F gets you close, P fine-tunes)
```

---

# P = Proportional 💪

**"The farther I am from my target, the harder I push."**

```
Error = Target Velocity - Current Velocity

Motor correction = P × Error
```

**Example** with P = 75, target = 1000 TPS:

| Current TPS | Error | P × Error | Push |
|-------------|-------|-----------|------|
| 0 | 1000 | 75,000 | 🔥 FULL POWER |
| 500 | 500 | 37,500 | 💪 Half power |
| 950 | 50 | 3,750 | 👌 Gentle push |
| 1000 | 0 | 0 | ✋ Stop correcting |

**Higher P** = more aggressive corrections.
**Too high** = oscillation (bounces back and forth around target).

---

# Visualizing P

```
Target: 1000 TPS
                                    ┌─── Oscillation (P too high)
                                    ▼
Velocity  ·····/\/\/\/\/\/\/\/\─────── 
         ·····/                         
1000 ──────────── ─ ─ ─ ─ ─ ─ ─ ─ ─── Target
         ···/            ╱─────────── 
        ··/            ╱               
       ·/           ╱                  Just right (P correct)
      ·/         ╱
     ·/       ╱─── ── ── ── ── ── ─── 
    ·       ╱                           Too slow (P too low)
    ·     ╱
    ·   ╱
    · ╱
    ╱
0───┴──────────────────────────────── Time
```

**Your goal:** Fast response, minimal overshoot, no oscillation.

---

# I = Integral 📚

**"I've been below target for a while — I need to push a little extra."**

```
I accumulates error over time:

Total correction = I × (sum of all past errors)
```

**When is I useful?**
- When there's a **constant force** fighting you (friction, gravity)
- When P alone gets close but never quite reaches the target
- The motor sits at 980 TPS but never hits 1000

**When is I dangerous?**
- It builds up and causes massive overshoot (**integral windup**)
- That's why your code has `I = 0` — it's the safest default

---

# Visualizing I (Integral Windup)

```
Without I:                          With I (tuned well):
         ┌── Steady-state error          ┌── No error!
         ▼                               ▼
980 ─────────────────────    1000 ──────────────────
                                    ╱
    ╱                            ╱
   ╱                           ╱
──┘                          ─┘


With I (too high):
1050 ─── overshoot!     ← I pushed too hard because it
         ╲                  remembered all the past error
1000 ────────────────
           ╱
          ╱
       ──┘
```

**Rule of thumb:** Only add I if P + F leaves a consistent gap.

---

# D = Derivative 🛑

**"I'm approaching the target fast — slow down so I don't overshoot!"**

```
D looks at how fast the error is CHANGING:

Correction = D × (current error - previous error)
```

Think of it like **brakes on a car**:
- P says "we're 100 feet from the stop sign, accelerate!"
- D says "we're going 60mph toward it — HIT THE BRAKES!"

**When is D useful?**
- When you're overshooting the target
- When the system oscillates (bounces back and forth)

**Your code:** `D = 0` everywhere except one place (`D = 0.25` in `MecanumDriveClose`)

---

# How P, I, D, F Work Together

```
Total motor power = F × target  +  P × error  +  I × Σerror  +  D × Δerror
                    ↑               ↑              ↑              ↑
                  "Base power"  "Push when   "Fix lingering  "Brake when
                  to reach      far away"     offset"         approaching
                  target"                                     fast"
```

**For your shooter (velocity control):**
1. **F** gets the motor spinning at roughly the right speed
2. **P** corrects the remaining error
3. **D** prevents overshoot and oscillation
4. **I** eliminates any tiny remaining steady-state error

**Tuning order: F → P → D → I** (always!)

---

<!-- _class: invert -->

# Part 3
## 🔧 Your Shooter's PIDF
### Let's Decode Your Current Numbers

---

# Understanding Your Motor

Your shooter uses a **DcMotorEx** in `RUN_USING_ENCODER` mode.

Key specs you need to know:
- **Maximum TPS** (ticks per second): This depends on your specific motor
- Your targets: **900 TPS** (short/mid shot) or **1000 TPS** (long shot)

The FTC SDK's PIDF controller runs **on the REV Hub**, not your phone.
When you call `setVelocityPIDFCoefficients()`, you're configuring that controller.

```java
// This configures the REV Hub's internal PID loop
shooter_1.setVelocityPIDFCoefficients(P, I, D, F);

// This sets the target for that PID loop
shooter_1.setVelocity(targetTPS);
```

---

# Let's Decode: Long Shot (P=75, F=6.5)

```java
shooter_1.setVelocityPIDFCoefficients(75, 0.0, 0.0, 6.5);
shooter_1.setVelocity(1000);
```

**F = 6.5:**
- The hub computes: `F × target_tps = 6.5 × 1000 = 6500`
- This is scaled internally to a power value
- It provides the **base power** to get the motor near 1000 TPS

**P = 75:**
- If error is 100 TPS: `75 × 100 = 7500` additional correction
- If error is 10 TPS: `75 × 10 = 750` small correction
- This is **very aggressive** — the motor corrects hard

**I = 0, D = 0:** No error accumulation, no braking. Raw P + F only.

---

# Why Different Shots Have Different Values

| Shot Type | Distance | TPS | P | F | Why? |
|-----------|----------|-----|---|---|------|
| Long | ~8 ft | 1000 | 75 | 6.5 | High speed, needs aggressive P |
| Short | ~3 ft | 900 | 28 | 10.5 | Lower speed, higher F compensates |
| Mid | ~5 ft | 900 | 28 | 13 | Same speed, even higher F |
| Emergency | Any | 1000 | 80 | 20 | Max everything, accuracy doesn't matter |

**Notice:** These were likely found by trial and error.
*"It worked at that distance"* — but **why** does it work? Let's find out.

---

# The Problem With Your Current Approach

```java
// In handleMechanisms(), this runs EVERY LOOP:
if (longShotOn) {
    shooter_1.setVelocityPIDFCoefficients(75, 0.0, 0.0, 6.5);
    tps = 1000;
}
```

**Issues:**
1. ⚠️ Calling `setVelocityPIDFCoefficients()` every loop is **wasteful** — the hub stores these values
2. ⚠️ Different P values for the same motor at slightly different speeds = **inconsistency**
3. ⚠️ No I or D term = the motor may never reach **exact** target velocity
4. ⚠️ No explanation of **how** these numbers were chosen = hard to tune later

---

# What Good PIDF Looks Like

```java
// Set ONCE during initialization — not every loop
private static final double SHOOTER_P = 40;
private static final double SHOOTER_I = 0.5;
private static final double SHOOTER_D = 5;
private static final double SHOOTER_F = 8;

public void initializeHardware() {
    // ...
    shooter_1.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, 
                                           SHOOTER_D, SHOOTER_F);
}
```

**One set of PIDF values** that works for your motor at **any** velocity.
The target velocity changes — the tuning stays the same.

But how do we find those values? 👉 That's the tuning process!

---

<!-- _class: invert -->

# Part 4
## 🎮 Tuning Step-by-Step
### Hands-On With FTC Dashboard

---

# What You Need

1. **FTC Dashboard** — already in your project (via Road Runner)
   - Connect to `192.168.43.1:8080/dash` when robot is running
   - Shows **real-time graphs** of motor velocity

2. **Your `PIDF_Testing` OpMode** — already exists in `testing/`!
   
3. **A notebook** — write down what values you try and what happened

4. **Patience** — this takes 30-60 minutes to do well!

---

# Your PIDF_Testing OpMode

You already have this! Let's look at it:

```java
// From testing/PIDF_Testing.java
double F = 0;
double P = 0;
double D = 0;

// D-pad controls:
// Up/Down = adjust P
// Left/Right = adjust F  
// Bumpers = adjust D
// B = change step size (10, 1, 0.1, 0.001, 0.0001)
```

You can adjust values **live** while the motor runs.
This is how we'll tune! 🎮

---

# Step 1: Find F (Feedforward)

**Goal:** Find the F value that gets the motor to ~80% of target speed with zero P, I, D.

```
Start: P=0, I=0, D=0, F=0, Target=1000 TPS

1. Increase F slowly (use step size 1)
2. Watch the velocity on telemetry
3. Stop when velocity reaches ~750-800 TPS

Example progression:
  F=2  → 200 TPS  (too low)
  F=5  → 520 TPS  (getting there)  
  F=7  → 740 TPS  (close!)
  F=8  → 810 TPS  (✓ good enough)
```

**Write down your F value!** ✏️

---

# Step 1: What You Should See

```
With only F, the motor gets close but not exact:

Target: 1000 TPS
                                ┌── There's a gap (that's OK!)
                                ▼
 800 ────────────────────────────── Actual velocity
                                   
1000 ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─── Target

  ╱
 ╱
╱
─────────────────────────────────── Time
```

**F doesn't try to close the gap.** That's P's job.

---

# Step 2: Add P (Proportional)

**Goal:** Close the gap between actual and target velocity.

```
Keep your F value. Start adding P.

1. Use step size 10 to start
2. Increase P until velocity hits 1000 TPS
3. Watch for OSCILLATION — velocity bouncing up and down

Example:
  P=10  → 920 TPS  (getting closer)
  P=20  → 960 TPS  (almost there)
  P=30  → 1000 TPS (on target!)
  P=50  → velocity bounces 950↔1050 (too high! Go back!)
  P=35  → 1000 TPS, stable (✓ 
```

**Write down your P value!** ✏️

---

# Step 2: What You Should See

```
Good P (velocity reaches target, no bouncing):

1000 ──────────────────────────────── Target
              ╱──────────────────── Actual (locked on!)
             ╱
            ╱
           ╱
──────────╱────────────────────────── Time


Bad P — too high (oscillating!):

1050 ──  ╱╲    ╱╲    ╱╲
1000 ──╱────╲╱────╲╱────── Target
       ╱
      ╱
──── ╱─────────────────────────────── Time
   ↑
   Motor is "hunting" — overshooting then undershooting
```

---

# Step 3: Add D (Derivative) If Needed

**Goal:** Stop oscillation — only add D if P causes bouncing.

```
If your P value causes slight oscillation:

1. Use step size 1
2. Add D slowly
3. D should be much smaller than P (usually 1/10 to 1/5 of P)
4. Stop when velocity is smooth

Example (if P=35 oscillates slightly):
  D=2  → still some bounce
  D=5  → smooth! ✓
  D=10 → sluggish response (too high, go back)
  D=5  → smooth approach to target ✓
```

**If there's no oscillation with your P, skip D (leave at 0).**

---

# Step 4: Add I (Integral) Only If Necessary

**Goal:** Fix any remaining steady-state error.

```
Look at your velocity graph on FTC Dashboard:

If velocity sits at 995 TPS but never reaches 1000:
  → You have steady-state error
  → Add a TINY amount of I

I should be VERY small (start at 0.1):
  I=0.1  → 998 TPS (closer)
  I=0.5  → 1000 TPS ✓
  I=2.0  → OVERSHOOT! (too high)
```

⚠️ **Warning:** I is the most dangerous term. It accumulates over time.
Too much I = the motor winds up and overshoots badly.

**Most FTC teams leave I at 0 and that's fine!**

---

# The Complete Tuning Recipe

```
┌─────────────────────────────────────────┐
│         PIDF TUNING RECIPE              │
│                                          │
│  1. Set P=0, I=0, D=0, F=0             │
│                                          │
│  2. INCREASE F until ~80% of target     │
│     └─ Write down F = ____              │
│                                          │
│  3. INCREASE P until at target          │
│     └─ If oscillating, back off P       │
│     └─ Write down P = ____              │
│                                          │
│  4. If oscillating, ADD D               │
│     └─ Keep D small (1/10 of P)         │
│     └─ Write down D = ____              │
│                                          │
│  5. ONLY if there's steady-state error: │
│     └─ Add TINY I (start at 0.1)        │
│     └─ Write down I = ____              │
│                                          │
│  6. TEST at different velocities!        │
└─────────────────────────────────────────┘
```

---

# Common Mistakes ⚠️

| Mistake | What Happens | Fix |
|---------|-------------|-----|
| P way too high | Motor oscillates wildly | Reduce P by 50% |
| F too high | Motor overspeeds immediately | Reduce F, it should only get ~80% |
| Adding I first | Massive overshoot | Always tune F → P → D → I (in order!) |
| Different PIDF per shot | Inconsistent behavior | One good set of values works for all speeds |
| Tuning with battery low | Values don't work later | Charge battery fully before tuning |
| Not writing down values | Forget what worked | ALWAYS write down what you try! |

---

# FTC Dashboard Pro Tips

**Seeing real-time graphs:**
1. Connect phone to WiFi
2. Go to `192.168.43.1:8080/dash`
3. Run your `PIDF_Testing` OpMode
4. Click the **Graph** tab
5. Add `velocity` and `targetVelocity` to the graph

**What to look for:**
- 📈 **Response time** — How fast does velocity reach target? (< 0.5 seconds = good)
- 📉 **Overshoot** — Does velocity go ABOVE target? (< 5% overshoot = good)
- 📊 **Steady state** — Does velocity stay at target? (< 2% error = good)

---

<!-- _class: invert -->

# Part 5
## 🚗 Drive Controller Tuning
### RoadRunner Gains Explained

---

# Your Drive Has PID Too!

In `MecanumDriveClose.java`, you have these gains:

```java
// Gains — these are PID values for DRIVING, not the shooter
public double axialGain = 1.8;     // Forward/backward correction
public double lateralGain = 1.5;   // Left/right (strafe) correction
public double headingGain = 6.8;   // Rotation correction

public double axialVelGain = 0.0;  // Forward velocity dampening
public double lateralVelGain = 0.0; // Strafe velocity dampening
public double headingVelGain = 0.0; // Rotation velocity dampening
```

These work exactly like P and D for your **drivetrain position**.

---

# What Each Gain Does

| Gain | Acts Like | Controls |
|------|-----------|----------|
| `axialGain` | **P** for forward/back | "Push harder when far from target X position" |
| `lateralGain` | **P** for left/right | "Push harder when far from target Y position" |
| `headingGain` | **P** for heading | "Rotate harder when pointing wrong direction" |
| `axialVelGain` | **D** for forward/back | "Brake when approaching target X too fast" |
| `lateralVelGain` | **D** for left/right | "Brake when approaching target Y too fast" |
| `headingVelGain` | **D** for heading | "Brake when rotating too fast toward target" |

**Your velocity gains are all 0** — meaning no dampening → potential overshoot.

---

# You Also Have Feedforward!

```java
// Feedforward parameters in MecanumDriveClose
public double kS = 0.12;   // Static friction compensation
public double kV = 0.012;  // Velocity feedforward
public double kA = 0;      // Acceleration feedforward (unused)
```

| Parameter | Shower Analogy |
|-----------|---------------|
| **kS** | The minimum knob turn to get ANY water flowing (overcomes friction) |
| **kV** | How much more to turn per unit of desired flow rate |
| **kA** | Extra boost when you need to change flow rate quickly |

These ensure the motors get the **right base power** before correction kicks in.

---

# Tuning Drive Gains with ManualFeedbackTuner

You already have `tuning/ManualFeedbackTuner.java`!

**Steps:**
1. Place robot on field with space to move
2. Run `ManualFeedbackTuner` OpMode
3. Open FTC Dashboard (192.168.43.1:8080/dash)
4. The robot will drive a specific path repeatedly

**What to adjust:**
```
If robot undershoots (doesn't reach target):
  → Increase axialGain / lateralGain

If robot overshoots (goes past target):
  → Decrease gains OR add velocity gains (axialVelGain)

If robot oscillates at the end:
  → Decrease gains AND add velocity gains
```

---

# Drive Tuning Recipe

```
┌────────────────────────────────────────────────┐
│         DRIVE GAIN TUNING RECIPE               │
│                                                 │
│  1. Tune feedforward FIRST (kS, kV)            │
│     └─ Use Road Runner's built-in tuner        │
│                                                 │
│  2. Start with position gains only:            │
│     └─ axialGain = 2.0                         │
│     └─ lateralGain = 2.0                       │
│     └─ headingGain = 4.0                       │
│                                                 │
│  3. Run ManualFeedbackTuner                    │
│     └─ Increase gains if undershooting         │
│     └─ Decrease if overshooting/oscillating    │
│                                                 │
│  4. If oscillating at end of paths:            │
│     └─ Add velocity gains (start at 0.1)       │
│     └─ These act as D term — dampening         │
│                                                 │
│  5. Test on ACTUAL autonomous paths!            │
└────────────────────────────────────────────────┘
```

---

# Your Two Drive Classes

⚠️ You have **two** mecanum drive classes with **different** values:

| Parameter | `MecanumDriveClose` | `MecanumDriveFar` |
|-----------|---------------------|-------------------|
| `axialGain` | 1.8 | (may differ) |
| `lateralGain` | 1.5 | (may differ) |
| `headingGain` | 6.8 | (may differ) |
| `kS` | 0.12 | (may differ) |
| `kV` | 0.012 | (may differ) |
| `maxWheelVel` | 25 | (may differ) |

**Ideally these should be the same!** Your robot's physical properties don't change between autonomous modes. The *paths* change, not the *tuning*.

---

<!-- _class: invert -->

# Part 6
## 📦 Consolidating Your Values
### Stop Copy-Pasting Numbers!

---

# The Problem Today

Your PIDF values are **hardcoded** in at least 4 different files:

```java
// In BaseTeleOp.java line 197:
shooter_1.setVelocityPIDFCoefficients(75, 0.0, 0.0, 6.5);

// In BaseTeleOp.java line 201:
shooter_1.setVelocityPIDFCoefficients(28, 0.0, 0, 10.5);

// In BaseAutonomus.java line 276:
shooter_1.setVelocityPIDFCoefficients(75, 0.0, 0.0, 6.5);

// In MecanumDriveClose.java line 321:
shooter_1.setVelocityPIDFCoefficients(65, 0.0, 0.0, 6.5);
```

**If you re-tune the shooter**, you have to find and change every copy.
You'll miss one. You always miss one. 😅

---

# The Fix: A Constants Class

```java
public final class ShooterConstants {
    private ShooterConstants() {} // Can't instantiate
    
    // PIDF Coefficients (tuned 2025-03-29)
    public static final double SHOOTER_P = 35;
    public static final double SHOOTER_I = 0;
    public static final double SHOOTER_D = 5;
    public static final double SHOOTER_F = 8;
    
    // Target velocities (ticks per second)
    public static final double LONG_SHOT_TPS = 1000;
    public static final double SHORT_SHOT_TPS = 900;
    public static final double MID_SHOT_TPS = 900;
}
```

**One place to change. One source of truth.** ✅

---

# Using the Constants

**Before (scattered magic numbers):**
```java
shooter_1.setVelocityPIDFCoefficients(75, 0.0, 0.0, 6.5);
shooter_1.setVelocity(1000);
```

**After (self-documenting constants):**
```java
import static org.firstinspires.ftc.teamcode.ShooterConstants.*;

// Set once during init
shooter_1.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, 
                                       SHOOTER_D, SHOOTER_F);
// Set target velocity based on shot type
shooter_1.setVelocity(LONG_SHOT_TPS);
```

Now anyone reading the code can understand what these numbers mean.

---

# Same Approach for Drive Constants

You already have `RR_RobotConstants.java` — expand it!

```java
public final class DriveConstants {
    private DriveConstants() {}
    
    // Feedforward (tuned with RR FeedforwardTuner)
    public static final double DRIVE_KS = 0.12;
    public static final double DRIVE_KV = 0.012;
    public static final double DRIVE_KA = 0;
    
    // Position gains (tuned with ManualFeedbackTuner)
    public static final double AXIAL_GAIN = 1.8;
    public static final double LATERAL_GAIN = 1.5;
    public static final double HEADING_GAIN = 6.8;
    
    // Velocity gains (dampening)
    public static final double AXIAL_VEL_GAIN = 0.1;
    public static final double LATERAL_VEL_GAIN = 0.1;
    public static final double HEADING_VEL_GAIN = 0.1;
}
```

---

# Your Tuning Log Template

**Keep a log!** When you tune, record:

```
┌─────────────────────────────────────────────────────┐
│  SHOOTER TUNING LOG — Date: ___________             │
│  Battery Voltage: _______ V                         │
│                                                      │
│  Target TPS: _______                                │
│                                                      │
│  Attempt | F    | P    | I    | D    | Result       │
│  ────────┼──────┼──────┼──────┼──────┼──────────    │
│  1       |      |      |      |      |              │
│  2       |      |      |      |      |              │
│  3       |      |      |      |      |              │
│  4       |      |      |      |      |              │
│  5       |      |      |      |      |              │
│                                                      │
│  Final values: P=___ I=___ D=___ F=___              │
│  Response time: ____ms  Overshoot: ____%            │
└─────────────────────────────────────────────────────┘
```

---

# Action Items for Next Season

1. **Tune your shooter** using the F → P → D → I recipe
   - Use `PIDF_Testing` OpMode + FTC Dashboard
   - Find **one good set** of PIDF values for your motor
   - Record values in your engineering notebook

2. **Tune your drive** using ManualFeedbackTuner
   - Add velocity gains to prevent overshoot
   - Merge `MecanumDriveClose` and `MecanumDriveFar` into one class

3. **Create Constants classes**
   - `ShooterConstants.java` — all shooter PIDF + TPS values
   - Expand `RR_RobotConstants.java` — all drive tuning values

4. **Stop calling `setVelocityPIDFCoefficients` every loop**
   - Set once in `initializeHardware()`, not in `handleMechanisms()`

---

# Key Takeaways 🧠

- **PIDF** = 4 strategies working together to hit a target
- **F** (Feedforward) = base power, gets you 80% there — tune first!
- **P** (Proportional) = error correction — tune second
- **D** (Derivative) = braking — only add if oscillating
- **I** (Integral) = memory — only add if there's steady-state error
- **Tuning order matters:** F → P → D → I, always!
- **One set of values** should work for all velocities
- **Constants classes** prevent copy-paste bugs
- **FTC Dashboard** is your best friend for tuning — use the graphs!

---

<!-- _class: invert -->

# Questions? 💬

**Resources:**
- [CTRL ALT FTC — PID Tuning](https://www.ctrlaltftc.com/the-pid-controller)
- [Game Manual 0 — PID Controllers](https://gm0.org/en/latest/docs/software/concepts/pid-controllers.html)
- [FTC Dashboard](https://acmerobotics.github.io/ftc-dashboard/)
- [Road Runner Tuning Guide](https://rr.brott.dev/docs/v1-0/tuning/)

**Your PIDF_Testing OpMode:** `testing/PIDF_Testing.java`
**Your ManualFeedbackTuner:** `tuning/ManualFeedbackTuner.java`
