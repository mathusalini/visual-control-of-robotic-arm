
# SSC-32U Robotic Arm Control (Python + PySerial)

This project controls a robotic arm connected to a **Lynxmotion SSC-32U** servo controller using **Python** and **pyserial**.

The script:
1. Connects to SSC-32U through a COM port
2. Moves all servos to a **HOME / CENTER position (1500µs)**
3. Runs a simple **sequential movement task** (base → shoulder → elbow → wrist)
4. Closes the serial connection safely

---

## Requirements

- Windows (or Linux/Mac with correct port name)
- Python 3.x
- SSC-32U connected via USB
- External servo power supply connected to SSC-32U (USB alone is not enough to power servos)

---

## Install Dependencies

Install pyserial:

```bash
python -m pip install pyserial
````

---

## Hardware Setup

1. Connect SSC-32U to your PC using USB.
2. Power the servos using an external **5V–6V** supply (high current recommended).
3. Connect servos to SSC-32U channels:

* Channel 0 → Base
* Channel 1 → Shoulder
* Channel 2 → Elbow
* Channel 3 → Wrist

> Make sure servo plug orientation is correct: **GND / V+ / Signal**.

---

## Find Your COM Port (Windows)

1. Open **Device Manager**
2. Go to **Ports (COM & LPT)**
3. Identify SSC-32U port (example: **COM7**)

Update this in the script:

```python
controller = SSC32U(port="COM7", baudrate=9600)
```

---

## Run the Code

From your project folder:

```bash
cd Robotic_arm
```

```bash
python main.py
```

You should see output like:

* Connected to SSC-32U...
* Step 0) Going to HOME...
* Step 1) Base move...
* ...
* Done ✅
* Connection closed

---

## How the Motion Works

The SSC-32U is controlled using commands like:

```
#0 P1500 T2000
```

Meaning:

* `#0` → channel 0
* `P1500` → move to pulse 1500µs (center)
* `T2000` → take 2000ms (2 seconds) to reach the target
  ✅ Larger `T` = slower motion
  ✅ Smaller `T` = faster motion

---

## Modify Servo Channels

If your arm uses different channels, change these:

```python
CH_BASE = 0
CH_SHOULDER = 1
CH_ELBOW = 2
CH_WRIST = 3
```

---

## Modify Home Position (Recommended)

**1500µs is center**, but for robotic arms it may not always be safe.

If your arm hits the table or frame, use a custom home like:

```python
controller._send("#0 P1500 #1 P1700 #2 P1300 #3 P1500 T2000")
```

---

## Troubleshooting

### Arm works in FlowBotics but not in Python

* Close FlowBotics completely (it may lock the COM port).
* Try baudrates:

  * `9600` (common)
  * `115200` (also common)

### No servo movement

* Check external servo power (5–6V, enough current)
* Check common ground
* Check servo plug direction
* Check baudrate

### COM Port error

* Update `COM7` to your actual port from Device Manager

---

## Safety Notes

* Start with small moves and slow speeds (`T2000`)
* Avoid extreme ranges until you confirm the servo limits
* Robotic arm servos can draw high current — ensure your power supply is adequate

---

## File

* `main.py` → Main script
* `README.md` → This guide

flowchart TD
    A(Start)
    B(Input: pick_xyz, place_xyz, object_width_cm)
    C(Go HOME - arm HOME pulses - gripper OPEN)
    D{Is PICK reachable?}
    E(Solve IK for PICK)
    F{IK success?}
    G(Convert angles to PWM)
    H(Send PWM via Serial)
    I(Move arm to PICK)
    J(Close gripper to object width)
    K(Lift UP from pick)
    L{Is PLACE reachable?}
    M(Solve IK for PLACE)
    N{IK success?}
    O(Convert angles to PWM)
    P(Send PWM via Serial)
    Q(Move ABOVE PLACE)
    R(Lower to PLACE)
    S(Open gripper - release object)
    T(Lift UP from place)
    U(Return HOME - gripper OPEN)
    X(End)

    A --> B --> C --> D
    D -- No --> U --> X
    D -- Yes --> E --> F
    F -- No --> U --> X
    F -- Yes --> G --> H --> I --> J --> K --> L
    L -- No --> U --> X
    L -- Yes --> M --> N
    N -- No --> U --> X
    N -- Yes --> O --> P --> Q --> R --> S --> T --> U --> X