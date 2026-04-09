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

# Multi-Threading for FTC 🤖
## Making Your Robot Do Two Things at Once

**BBTC Robotics — 2025 Off-Season Workshop**

---

# What We'll Cover Today

1. 🔍 **Your Robot Today** — How your code actually runs
2. 🚨 **The Problem** — Why your robot seems "slow"
3. 🔄 **State Machines** — The safe way to multitask
4. 🏗️ **Refactoring Autonomous** — No more `sleep()` wasted time
5. 🧵 **Java Threads** — Real multi-threading (advanced)
6. 🎯 **Putting It Together** — Architecture for next season

---

<!-- _class: invert -->

# Part 1
## 🔍 Your Robot Today

---

# How Does Your TeleOp Actually Run?

Open `TeleOpRed.java` — here's your main loop:

```java
while (opModeIsActive()) {
    handleDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x);
    handleMechanisms();      
    camera.update();          
    // ... LED logic ...
    telemetry.update();       
}
```

This runs **one line at a time**, top to bottom, then repeats.

**Question for you:** What happens if `camera.update()` takes a long time?

---

# The Answer: Everything Waits

Your loop runs like a **single conveyor belt**:

```
┌──────────────────────────────────────────────────────┐
│                    ONE LOOP CYCLE                     │
│                                                       │
│  handleDrive() ──→ handleMechanisms() ──→ camera ──→ LED ──→ telemetry
│     5ms                 8ms               12ms      3ms      2ms
│                                                       │
│              Total: ~30ms = ~33 loops/second          │
└──────────────────────────────────────────────────────┘
```

Nothing can start until the previous thing finishes.
If camera takes **50ms** instead of 12ms → your **drive feels laggy**.

---

# What's Inside handleMechanisms()?

Your `BaseTeleOp.java` does **everything** in one method:

```java
protected void handleMechanisms() {
    camera.update();                    // Read camera again
    
    // Toggle buttons
    if (gamepad1.x && !xWasPressed) intakeOn = !intakeOn;
    if (gamepad1.b && !bWasPressed) shooterOn = !shooterOn;
    
    // Set PIDF coefficients (every single loop!)
    shooter_1.setVelocityPIDFCoefficients(75, 0.0, 0.0, 6.5);
    
    // Calculate powers, set motors...
    shooter_1.setVelocity(finalShooterVel);
    intakeServo.setPower(finalServoPower);
    intakeTransfer.setPower(finalTransferPower);
}
```

**6 different responsibilities** crammed into 1 method.

---

# Your Autonomous Is Worse

Look at `AutonomusRedGoal.java`:

```java
closeShot();                    // Set shooter speed
moveForward(57, 0.7);          // ⏳ BLOCKS until done driving
sleep(100);                     // ⏳ BLOCKS for 100ms doing nothing
intakeServo.setPower(1.0);
intakeTransfer.setPower(1.0);
sleep(2500);                    // ⏳ BLOCKS for 2.5 SECONDS doing nothing
shooter_1.setPower(0);
stopShootSequence();
rotate(-142, DRIVE_SPEED);     // ⏳ BLOCKS until done turning
```

<span class="red">**Every `sleep()` and `moveForward()` = your robot is FROZEN**</span>

Nothing else can happen during those calls.

---

# Let's Map the Timeline

**Your current autonomous does this:**

```
TIME ──────────────────────────────────────────────────────→

Drive:    [====DRIVING 57in====]...[======ROTATING======]...
Shooter:  .............[ON].......................
Intake:   ........................[===FEEDING===]...........
Robot:    ................[SLEEP 100ms][SLEEP 2500ms].......
                              ↑               ↑
                          DOING NOTHING    DOING NOTHING
```

All those flat lines = **wasted time** your robot could be doing something.

---

<!-- _class: invert -->

# Part 2
## 🚨 The Problem

---

# The Single-Thread Problem

Your robot has **one brain** trying to do everything:

```
One Thread = One Thing at a Time

  "Drive forward"     → Can't shoot
  "Wait for shooter"  → Can't drive  
  "Read camera"       → Can't move intake
  "Sleep 2500ms"      → Can't do ANYTHING
```

In a **30-second autonomous**, you might waste **8-10 seconds** on `sleep()` calls.

That's **25-33% of your auto** doing literally nothing! 😱

---

# Real Example From Your Code

In `AutonomusRedGoal.java`, you do this **three times**:

```java
closeShot();                    // Start shooter
// ... drive somewhere ...
intakeServo.setPower(1.0);
intakeTransfer.setPower(1.0);
sleep(2500);                    // Wait for balls to feed
shooter_1.setPower(0);
```

**3 × 2.5 seconds = 7.5 seconds** of sleeping.

What if you could drive to the next position **while** feeding balls? 🤔

---

# What We Want Instead

```
TIME ──────────────────────────────────────────────────────→

Drive:    [====DRIVING====][==ROTATING==][====DRIVING====]──
Shooter:  [===SPINNING UP===][SHOOTING][===SPINNING UP===]─
Intake:   ......[==FEEDING==]............[==FEEDING==].....─
Camera:   [==SCANNING==][==SCANNING==][==SCANNING==]────────

              ↑ Everything overlaps! No wasted time!
```

**This is what multi-tasking looks like.** But how do we get there?

---

<!-- _class: invert -->

# Part 3
## 🔄 State Machines
### The Safe Way to Multitask

---

# What Is a State Machine?

Think of your **phone's alarm**:

| State | What It Does | What Changes It |
|-------|-------------|-----------------|
| **OFF** | Nothing | You set an alarm → goes to WAITING |
| **WAITING** | Counts down | Time hits 0 → goes to RINGING |
| **RINGING** | Makes noise | You tap dismiss → goes to OFF |
| **SNOOZING** | Counts 5 min | Time hits 0 → goes to RINGING |

The alarm doesn't **block** your phone. Your phone checks: *"Is it time to ring?"* — if no, it moves on to other things.

---

# Your Intake as a State Machine

Right now your intake logic is scattered through `handleMechanisms()`.
Let's think about its **states**:

```
          ┌──────────┐
     ┌───→│   OFF    │←──────────────┐
     │    └────┬─────┘               │
     │         │ Press X             │ Press X
     │         ▼                     │
     │    ┌──────────┐          ┌────┴─────┐
     │    │ RUNNING  │          │ REVERSE  │
     │    └────┬─────┘          └──────────┘
     │         │ Press A             ▲
     │         └─────────────────────┘
     │ Press B (shooter takes over)
     └───────────────────────────────
```

Each state knows: **what to do** and **what makes it change**.

---

# Let's Code It: Step 1 — Define States

```java
public enum IntakeState {
    OFF,
    RUNNING,
    REVERSE
}
```

That's it. An **enum** is just a list of named options.
Java knows there are only these 3 possibilities.

---

# Step 2 — Track the Current State

```java
private IntakeState intakeState = IntakeState.OFF;
```

Instead of juggling `intakeOn`, `intakeReverseOn`, `servoOn`... 
you have **one variable** that tells you everything.

**Before:** 3 booleans = 8 possible combinations (most are invalid!)
**After:** 1 enum = 3 possible states (all are valid!)

---

# Step 3 — Write the State Machine

```java
private void updateIntake() {
    // Check for state transitions
    if (gamepad1.x && !xWasPressed) {
        intakeState = (intakeState == IntakeState.RUNNING) 
            ? IntakeState.OFF : IntakeState.RUNNING;
    }
    if (gamepad1.a && !aWasPressed) {
        intakeState = (intakeState == IntakeState.REVERSE) 
            ? IntakeState.OFF : IntakeState.REVERSE;
    }
    
    // Act on current state
    switch (intakeState) {
        case OFF:     intakeTransfer.setPower(0); intakeServo.setPower(0); break;
        case RUNNING: intakeTransfer.setPower(0.75); intakeServo.setPower(0); break;
        case REVERSE: intakeTransfer.setPower(-0.3); intakeServo.setPower(-0.75); break;
    }
}
```

---

# What Makes This Better?

<div class="columns">
<div>

### ❌ Before
```java
// Scattered booleans
boolean intakeOn = false;
boolean intakeReverseOn = false;
boolean servoOn = false;

// Tangled if/else chains
if (intakeReverseOn && !shooterOn) {
    finalTransferPower = -0.3;
    finalServoPower = -0.75;
    intakeOn = false;
} else if (intakeOn && !shooterOn) {
    // ...
}
```

</div>
<div>

### ✅ After
```java
// One clear variable
IntakeState intakeState;

// Clean switch
switch (intakeState) {
    case OFF:     /* ... */ break;
    case RUNNING: /* ... */ break;
    case REVERSE: /* ... */ break;
}
```

</div>
</div>

**Easier to read. Easier to debug. Impossible to be in an invalid state.**

---

# Now Do the Same for the Shooter

```java
public enum ShooterState {
    OFF,
    SPINNING_UP,    // Motor on, not at speed yet
    READY,          // At target velocity
    SHOOTING         // Feeding balls
}
```

The key insight: **SPINNING_UP** doesn't block anything!

```java
case SPINNING_UP:
    shooter_1.setVelocity(targetTPS);
    // Check if we've reached target speed
    if (Math.abs(shooter_1.getVelocity() - targetTPS) < 50) {
        shooterState = ShooterState.READY;
    }
    break;
```

You can drive, move intake, read camera — all while the shooter spins up! 🎉

---

# The Shooter State Machine

```java
private void updateShooter() {
    switch (shooterState) {
        case OFF:
            shooter_1.setVelocity(0);
            break;
            
        case SPINNING_UP:
            shooter_1.setVelocity(targetTPS);
            double error = Math.abs(shooter_1.getVelocity() - targetTPS);
            if (error < 50) shooterState = ShooterState.READY;
            break;
            
        case READY:
            shooter_1.setVelocity(targetTPS);  // Maintain speed
            // LED shows green — ready to fire!
            break;
            
        case SHOOTING:
            intakeServo.setPower(1.0);
            intakeTransfer.setPower(1.0);
            if (shootTimer.seconds() > 2.5) shooterState = ShooterState.OFF;
            break;
    }
}
```

---

# Your New TeleOp Loop

```java
while (opModeIsActive()) {
    // Each method checks its state and acts — never blocks!
    handleDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x);
    updateShooter();     // ← State machine (never sleeps)
    updateIntake();      // ← State machine (never sleeps)
    updateLED();         // ← State machine (never sleeps)
    camera.update();
    telemetry.update();
}
```

**Every method returns immediately.** 
No method waits for anything.
The loop runs **fast** — 50-100+ times per second.

---

<!-- _class: invert -->

# Part 4
## 🏗️ State Machines in Autonomous
### No More `sleep()` Wasted Time

---

# The Autonomous Problem

Your `AutonomusRedGoal.java` calls `moveForward()` which contains:

```java
protected void moveForward(double inches, double speed) {
    // ... set target ...
    while (opModeIsActive() && flmotor.isBusy()) {   // ← BLOCKS!
        // Can't do anything else while driving!
        setDrivePower(currentPower);
    }
    stopDrive();   // calls sleep(250) !
}
```

The `while` loop **traps** execution until driving is done.
Then `stopDrive()` **sleeps** for 250ms on top of that!

---

# State Machine Approach for Driving

```java
public enum DriveState {
    IDLE,
    DRIVING,
    STRAFING,
    ROTATING,
    SETTLING    // Replaces sleep(250)
}
```

```java
case DRIVING:
    int remaining = Math.abs(target - flmotor.getCurrentPosition());
    double inchesLeft = remaining / COUNTS_PER_INCH;
    
    if (!flmotor.isBusy()) {
        settleTimer.reset();
        driveState = DriveState.SETTLING;
    } else {
        double power = (inchesLeft < 20) 
            ? Range.clip((inchesLeft / 15.0) * speed, 0.15, speed) 
            : speed;
        setDrivePower(power);
    }
    break;
```

---

# Non-Blocking Autonomous Sequence

Instead of linear code, use a **step counter**:

```java
int step = 0;

while (opModeIsActive()) {
    updateDrive();      // State machine
    updateShooter();    // State machine
    updateIntake();     // State machine
    
    switch (step) {
        case 0:  // Start shooter AND drive at the same time!
            startShooter(CLOSE_SHOT);
            startDriveForward(57, 0.7);
            step++;
            break;
        case 1:  // Wait for BOTH to finish
            if (driveState == IDLE && shooterState == READY) step++;
            break;
        case 2:  // Fire!
            startShooting();
            step++;
            break;
        // ...
    }
}
```

---

# The Timeline Improvement

**Before (blocking):**
```
[===DRIVE 57in===][sleep][===SPIN UP===][sleep 2.5s][===ROTATE===]
                                                Total: ~9 seconds
```

**After (state machines):**
```
[===DRIVE 57in===][SHOOT]...[===ROTATE===]
[===SPIN UP======][     ]...[===SPIN UP==]  ← Overlapped!
                        Total: ~5.5 seconds
                        
               ⚡ ~40% FASTER ⚡
```

The shooter spins up **while** driving. No wasted time!

---

# Quick Exercise 🧠

Look at these three lines from your autonomous:

```java
closeShot();              // Start shooter
moveForward(57, 0.7);     // Drive 57 inches
sleep(100);               // Wait
```

**Question:** With state machines, how would you rewrite this?

**Answer:** Start both at the same time, advance when both are done!
```java
case 0:
    startShooter(CLOSE_SHOT);     // Non-blocking: sets state to SPINNING_UP
    startDriveForward(57, 0.7);   // Non-blocking: sets state to DRIVING
    step++;
    break;
case 1:
    if (driveIdle() && shooterReady()) step++;  // Wait for both
    break;
```

---

<!-- _class: invert -->

# Part 5
## 🧵 Java Threads
### Real Multi-Threading (Advanced)

---

# What Is a Thread?

A **thread** is a separate "brain" that runs code independently.

```
Your Robot Controller (the phone/hub) has multiple CPU cores

   Core 1                    Core 2
   ┌─────────────┐          ┌─────────────┐
   │ Main Thread  │          │ Your Thread  │
   │             │          │              │
   │ Drive loop  │          │ Camera       │
   │ Mechanisms  │          │ processing   │
   │ Telemetry   │          │              │
   └─────────────┘          └─────────────┘
        ↕ Both run at the same time! ↕
```

Unlike state machines (which take turns), threads run **truly simultaneously**.

---

# When Threads Make Sense in FTC

| ✅ Good Use Cases | ❌ Dangerous Use Cases |
|---|---|
| Camera/vision processing | Controlling motors |
| Complex math calculations | Reading sensors used by main loop |
| Logging/recording data | Anything touching `hardwareMap` from 2 threads |
| Dashboard telemetry | Multiple threads calling `setPower()` |

**Golden Rule:** Hardware access from multiple threads = 💥 crashes.

The FTC SDK is **not thread-safe** for hardware operations.

---

# Creating a Thread in Java

```java
// Define what the thread will do
Thread cameraThread = new Thread(() -> {
    while (opModeIsActive()) {
        camera.update();           // Process camera frame
        
        // Store results in a thread-safe variable
        synchronized (lock) {
            latestTx = camera.getTx();
            latestDistance = camera.getDistance();
        }
        
        try {
            Thread.sleep(20);      // Don't hog the CPU - run at 50Hz
        } catch (InterruptedException e) {
            break;                 // Clean exit
        }
    }
});

// Start it!
cameraThread.start();
```

---

# What Is `synchronized`?

When two threads access the same variable, bad things happen:

```
Thread 1 (writes):  latestTx = 5.3
Thread 2 (reads):   value = latestTx   ← might get half-written garbage!
```

`synchronized` creates a **lock** — only one thread at a time:

```java
private final Object lock = new Object();

// Writing (camera thread)
synchronized (lock) {
    latestTx = camera.getTx();        // Thread 2 waits here
}

// Reading (main thread)
synchronized (lock) {
    double tx = latestTx;              // Thread 1 waits here
}
```

Think of it like a bathroom lock 🚪 — one person at a time!

---

# Safe Camera Thread for Your Robot

```java
public class ThreadedCamera {
    private final Camera camera;
    private final Object lock = new Object();
    private volatile boolean running = true;
    
    private double tx, distance;
    private boolean hasTarget;
    
    public void start() {
        new Thread(() -> {
            while (running) {
                camera.update();
                synchronized (lock) {
                    tx = camera.getTx();
                    distance = camera.getDistance();
                    hasTarget = camera.hasTarget();
                }
                try { Thread.sleep(20); } 
                catch (InterruptedException e) { break; }
            }
        }).start();
    }
    
    public double getTx() { synchronized (lock) { return tx; } }
    public void stop() { running = false; }
}
```

---

# Using It In Your TeleOp

```java
ThreadedCamera threadedCamera;

@Override
public void runOpMode() {
    initializeHardware();
    
    // Start camera on its own thread
    threadedCamera = new ThreadedCamera(camera);
    threadedCamera.start();
    
    waitForStart();
    
    while (opModeIsActive()) {
        handleDrive(...);
        updateShooter();
        updateIntake();
        
        // Camera data is always fresh — never slows down your loop!
        double tx = threadedCamera.getTx();
        telemetry.addData("Target Tx", tx);
        telemetry.update();
    }
    
    threadedCamera.stop();  // Clean up!
}
```

---

# Thread Safety Checklist ✅

Before using threads in FTC, ask yourself:

- [ ] Does this thread touch **any hardware** (`setPower`, `setPosition`)?
  - ⚠️ If yes, **DON'T use a thread** — use a state machine instead
- [ ] Am I sharing variables between threads?
  - ⚠️ If yes, use `synchronized` or `volatile`
- [ ] Do I stop the thread when the OpMode ends?
  - ⚠️ If no, add `running = false` and `thread.interrupt()`
- [ ] Could this crash and leave hardware in a bad state?
  - ⚠️ If yes, wrap in `try/catch` and set motors to 0 on error

**When in doubt, use a state machine. Threads are a power tool — respect them.**

---

<!-- _class: invert -->

# Part 6
## 🎯 Putting It Together

---

# Your Improved Architecture

```
┌─────────────────────────────────────────────────┐
│               MAIN THREAD (OpMode Loop)          │
│                                                   │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐       │
│  │  Drive   │  │ Shooter  │  │  Intake  │       │
│  │  State   │  │  State   │  │  State   │       │
│  │ Machine  │  │ Machine  │  │ Machine  │       │
│  └──────────┘  └──────────┘  └──────────┘       │
│       ↑              ↑             ↑              │
│       └──────────────┴─────────────┘              │
│              Autonomous Sequencer                 │
│         (step counter coordinates all)            │
└──────────────────────────────────────────────────┘
                       ↕ shared data (synchronized)
┌──────────────────────────────────────────────────┐
│              CAMERA THREAD                        │
│       Limelight processing at 50Hz               │
└──────────────────────────────────────────────────┘
```

---

# Before vs After: Summary

| Aspect | Before | After |
|--------|--------|-------|
| **TeleOp loop** | Everything sequential | State machines, never blocks |
| **Autonomous** | `sleep()` everywhere | Overlapping actions |
| **Shooter control** | Blocking spin-up | Non-blocking state machine |
| **Camera** | Runs in main loop | Background thread |
| **Code structure** | Boolean spaghetti | Clean enums + switch |
| **Auto time used** | ~60-70% (rest is sleeping) | ~90%+ |
| **Debugging** | "Which boolean is wrong?" | "What state am I in?" |

---

# Action Items for Next Season

1. **Start with state machines** — Refactor `BaseTeleOp` first
   - Create `IntakeState`, `ShooterState` enums
   - Replace boolean flags with enum variables
   - Write `updateIntake()`, `updateShooter()` methods

2. **Then fix autonomous** — Make `BaseAutonomus` non-blocking
   - Remove all `sleep()` calls
   - Add step-based sequencing
   - Overlap shooter spin-up with driving

3. **Then add camera thread** — Only after state machines work
   - Create `ThreadedCamera` class
   - Test extensively before competition

---

# Key Takeaways 🧠

- Your robot runs **one line at a time** — every `sleep()` = wasted time
- **State machines** make each mechanism independent — they never block
- An **enum** is safer than multiple booleans — impossible to have invalid states
- **Threads** give true parallelism but are **dangerous** with hardware
- **Always** use state machines first, threads only when needed
- Your autonomous could be **~40% faster** with overlapping actions

---

<!-- _class: invert -->

# Questions? 💬

**Resources:**
- [Game Manual 0 — State Machines](https://gm0.org/en/latest/docs/software/concepts/finite-state-machines.html)
- [FTC Docs — Threading](https://ftc-docs.firstinspires.org/)
- Your code: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`

**Next up: PID Tuning Deep Dive 🎯**
