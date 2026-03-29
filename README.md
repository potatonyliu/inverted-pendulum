# Results
[`records/dancing-queen-03_20260329_105457.csv`](https://github.com/potatonyliu/inverted-pendulum/blob/main/records/dancing-queen-03_20260329_105457.csv):
Same spec, removed limit switch and instead use phi for crash detection. Ran for 5 minutes and lost balance. Motor constantly saturates, we might need lower cart mass or stronger motor. Right now the jittering is caused by continued saturation of PWM and over-compensation. We have an issue with the gain values from MATLAB, it is consistently giving forces over the max force producable by our motor (around 6.5-10N) for diviations within 1 std of a typical run. The easiest fix right now is to reduce cart mass. I already tested that adjusting Q and R to make max force fall below 8N in 1 std of phi in a typical run did not work (it requires higher force later, which is a problem, std changes). I haven't figured this out yet.
<img width="1947" height="1395" alt="dancing-queen-03_fig1_overview" src="https://github.com/user-attachments/assets/17ec2942-8454-4993-9b94-957f3dc323ca" />


[`records/dancing-queen-02_20260328_211616.csv`](https://github.com/potatonyliu/inverted-pendulum/blob/main/records/dancing-queen-02_20260328_211616.csv):

Same spec, this time for 4 minutes! Again stopped at false crash detection.
<img width="1947" height="1395" alt="dancing-queen-02_fig1_overview" src="https://github.com/user-attachments/assets/d3b598c8-a910-4b71-88c8-e671d5893fa8" />



[`records/dancing-queen_20260328_204710.csv`](https://github.com/potatonyliu/inverted-pendulum/blob/main/records/dancing-queen_20260328_204710.csv):

Balanced for 69.64s - The limit switch pin might've deteched or touched something by accident, it did not hit the edge but detected crash - it wouldn've kept going!
<img width="1948" height="1394" alt="image" src="https://github.com/user-attachments/assets/a54409ec-c78d-4f7f-9408-f69a101e3c89" />


# How We Can Collaborate

## Setup

### Clone the repo

```bash
git clone https://github.com/potatonyliu/inverted-pendulum
cd inverted-pendulum
```

### Install PlatformIO (or you can install VSCode extension platformio)

```bash
brew install pipx
pipx install platformio
```

## When you want to make changes to the code:

### Get a new branch

```bash
 git checkout main                 # checkout main branch
 git pull                          # get latest changes
 git checkout -b your-branch-name  # create your branch
```

Example branch naming:

- feature/swing-up-controller
- experiment/PWM-to-force
- fix/encoder-misalignment

### Make changes

- Make changes to the code (in `inverted-pendulum/firmware/src`)
- Commit regularly with good commit messages

### Push and open PR (If the changes should be permanent)

```bash
git push -u origin branch-name
```

Then on GitHub:

- Click "Compare & pull request"
- Set base branch to main
- Describe your changes and why briefly
- Tag me so I can merge it into main branch

## Compiling

Check connection: 

```bash
ls /dev/cu.usb*
```

Compile to pico depending on which file you are running (update `platformio.ini` if you add other files):

```bash
pio run -e pico_main
pio run -e pico_exp_b
```

Serial out/in:

```bash
 pio device monitor -b 115200
```

### Logging

We log data from experiments for analysis.

```cpp
bool csv_mode = true;
```

Flip `csv_mode` to true if not already (false for more readable terminal debugging).

```bash
pio device monitor -b 115200 | tee "../logs/EXPERIMENT_NAME_$(date +%Y%m%d_%H%M%S).csv"
```

This command pipes the entire serial output into a .csv file in `logs/`. If the log is significant for analysis, copy the .csv file to `records/` in its own folder named `YYMMDD_experiment_title/` and add a `README.md` in that folder to explain what it is about or any significant results.

# **Code Architecture**

The firmware is split into three layers:

- **hardware.h / hardware.cpp** — pin definitions, system parameters, encoder reading, motor control, interrupt handlers, setup
- **control.h / control.cpp** — LQR gain and control computation
- **main.cpp / exp_b.cpp / whatever other ones you may want to create** — state machine, loop logic, serial logging

If you are tuning gains or changing control law, go to `control.cpp`. If you are changing motor behavior, encoder resolution, or pin mapping, go to `hardware.cpp`. The experiment files just orchestrate everything.

# **hardware.cpp**

## **Pin definitions**

```
 #define XA_PIN 1
 #define XB_PIN 2
 #define PA_PIN 3
 #define PB_PIN 9
 #define IN1_PIN 5
 #define IN2_PIN 6
 #define ENA_PIN 7
```

## **System parameters**

```
 const float MAX_FORCE = 3;
 const float PULLEY_DIAMETER = 0.0225;
 const float METERS_PER_TICK = PI * PULLEY_DIAMETER / 403.2;
 const float RADIANS_PER_TICK = 2.0 * PI / 2400.0;
```

`MAX_FORCE` is used for now without proper force-PWM translation. `METERS_PER_TICK` and `RADIANS_PER_TICK` translate encoder ticks to actual position and angle:

```
 float read_position(){
     return cart_ticks * METERS_PER_TICK;  // meters
 }
 
 float read_angle(){
     return pendulum_ticks * RADIANS_PER_TICK;  // radians
 }
```

## **Motor control**

```
 void update_motor(float force);       // takes Newtons, converts to PWM
 void update_motor_directly();         // writes ENA (global int) directly as PWM
 void coast_motor();                   // cuts power, no braking
```

`update_motor()` detects direction from sign, then maps magnitude linearly to PWM as a ratio of `MAX_FORCE`. `coast_motor()` sets both IN pins LOW to let the motor freewheel instead of braking.

## **Encoder handlers**

```
 void pendulum_A_handler();
 void pendulum_B_handler();
 void cart_A_handler();
 void cart_B_handler();
```

4 interrupt handlers (rising and falling edge on both A and B channels) for maximum encoder resolution.

## **hardware_setup()**

Called in `setup()` of each experiment file. Sets pin modes, attaches all 4 interrupts.


NOTE🤨:
`pinMode(PA_PIN, INPUT_PULLUP)` and `pinMode(PB_PIN, INPUT_PULLUP)` are set **after** `attachInterrupt` — the rotary encoder is open collector and needs pull-ups, but attaching the interrupt overwrites the pin mode for some reason so the order matters.

# **control.cpp**

```
 float K[4] = {-1.0000, -3.1708, 80.2158, 24.9766};
 float state[4] = {0.0, 0.0, 0.0, 0.0};  // x, xdot, phi, phidot
 
 float compute_control(){
     float u = -K[0]*state[0] - K[1]*state[1] - K[2]*state[2] - K[3]*state[3];
     u = constrain(u, -MAX_FORCE, MAX_FORCE);
     return u;
 }
```

`K` is the LQR gain calculated in MATLAB. `state` is updated every 1ms in the main loop and shared here. Note that `state` initializes at 0 — meaning the system assumes the cart starts centered with the pendulum upright. We can later make phi = pi at startup.

---

# **main.cpp**

## **Variables**

```
 float K[4] = {-1.0000, -3.1708, 80.2158, 24.9766};  // in control.cpp
 float state[4];       // x, xdot, phi, phidot — shared with control
 float force_out;
 float x, phi, xdot, phidot;
 float prev_x, prev_phi;
 unsigned long t0;     // system start timestamp
 unsigned long t1;     // previous control loop timestamp
 unsigned long last_print;
```

## **State machine**

```
 enum SystemState { IDLE, RUNNING };
 SystemState currentState = IDLE;
```

- `IDLE` — motor coasts, no control
- `RUNNING` — LQR control active

Serial input to transition:

- `w` → IDLE to RUNNING
- `s` → RUNNING to IDLE

## **Crash detection**

```
 if (currentState == RUNNING && (x > 0.6 || x < -0.6)) {
     currentState = IDLE;
     event = "crash";
 }
```

Cart going out of bounds forces IDLE and coasts the motor. The `event` field in the CSV log will say `crash` on that row. We can add Soren’s limiter later.

## **Main loop**

### **Every 1ms**

```
 if (micros()-t1 >= 1000){
     x = read_position();
     phi = read_angle();
     xdot = (x-prev_x)/0.001;
     phidot = (phi-prev_phi)/0.001;
     prev_x = x;
     prev_phi = phi;
     state[0] = x;
     state[1] = xdot;
     state[2] = phi;
     state[3] = phidot;
     force_out = compute_control();
     // serial input for state transitions
 }
```

Reads encoders, calculates velocities by finite difference, updates `state`, computes control force.

### **Serial logging**

```
 bool csv_mode = true;
```

- `csv_mode = true` — 100Hz, outputs: `time_us, state, force, x, xdot, phi, phidot, event`
- `csv_mode = false` — 10Hz, human-readable labeled output

`event` is normally empty. Gets set to `start`, `manual_stop`, or `crash` on transitions, printed on the next log row, then cleared.

---

# **exp_b.cpp (under development)**

This file is for force-PWM experiments. Main stuff is similar to main.cpp. Changes are:

## **Additional variables**

```
 int ENA;  // global PWM value, written directly
 
 float xddot;
 float phiddot;
 float prev_xdot, prev_phidot;
 
 float acc_threshold = 0.01;
 int consecutive_count_below_threshold = 0;
```

`xddot` and `phiddot` are computed by finite difference each 1ms and included in the CSV. `acc_threshold` and `consecutive_count_below_threshold` are used to detect when velocity has become constant.

## **State machine**

```
 enum SystemState { IDLE, RUNNING, ACCELERATING, TESTING };
 SystemState currentState = IDLE;
```

- `IDLE` — motor coasts
- `RUNNING` — motor running at constant PWM, but test not started yet (first 1 second)
- `ACCELERATING` — motor running, watching for velocity to converge
- `TESTING` — velocity converged, motor coasting, test in progress

Serial input:

- `w` → IDLE to RUNNING (also resets `t0`)
- `s` → anything to IDLE

## **Main test loop**

```
 // After 1 second, allow test to start
 if (currentState == RUNNING && micros()-t0 >= 1000000){
     currentState = ACCELERATING;
     event = "accelerating";
 }
```

```
 // Every 1ms: check if velocity has converged
 if (consecutive_count_below_threshold >= 10){
     currentState = TESTING;
     event = "test_start";
 }
```

Run at constant PWM until acceleration drops below threshold for 10 consecutive ms. Then coast motor and start test.

## **Crash detection**

```
 if (x > 1.2 || x < -0.1) {
     currentState = IDLE;
     event = "crash";
 }
```

Crash x detection is asymmetrical because exp_b should start from an end and run to the other end.

## **Serial logging**

```
 bool csv_mode = true;
```

- `csv_mode = true` — 100Hz, outputs: `time_us, state, ENA, x, xdot, xddot, phi, phidot, phiddot, event`
- `csv_mode = false` — 10Hz, human-readable labeled output

`event` values: `experiment_start`, `accelerating`, `test_start`, `manual_stop`, `crash`
