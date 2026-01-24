# Aquila: Fixed-Wing UAS 6-DOF Digital Twin & Flight Software Stack

Aquila is a small fixed-wing UAS “digital twin” and flight software stack that mirrors the kinds of tools used in modern defense/autonomy startups:

- A 6-DOF nonlinear dynamics model of a small fixed-wing UAV
- A modular C++ flight software (FSW) core for estimation, guidance, control, and mode management
- A Python-based simulation and Monte Carlo harness for performance and robustness evaluation
- An architecture deliberately compatible with PX4 / ROS 2 / SITL/HIL-style workflows

---

<details>
<summary><strong>1. Goals & Scope</strong></summary>

<br/>

### 1.1 Objectives

Aquila is intended to:

- Demonstrate system-level thinking for fixed-wing UAS:
  - Dynamics, sensors, estimation, guidance, control, and modes
- Provide a sandbox for:
  - Flight control and navigation logic
  - Sensor fusion experiments
  - Fault tolerance and mission-level behaviors
- Serve as a portfolio artifact that looks and feels like a stripped-down internal tool:
  - Clear architecture and modules
  - Tests, logging, and analysis
  - Containerized dev environment

### 1.2 Target Use Cases

- Validate flight control and basic navigation logic before hardware
- Explore estimation strategies for IMU + GPS + baro
- Run Monte Carlo campaigns to study robustness to environment and model uncertainty
- Provide concrete technical material for discussions with autonomy / flight software / GNC teams

### 1.3 Out-of-Scope (Initial Version)

- High-fidelity aerodynamic identification for a specific airframe  
  - Initial model will use a simplified but structurally correct 6-DOF aero model
- Real hardware-in-the-loop (HIL) with flight controllers  
  - Architecture will be HIL-ready, but initial work focuses on software-only SIL/SITL-style simulations
- Full ROS 2 or PX4 integration  
  - Initial implementation uses a custom IPC link between sim and FSW, with ROS 2 / PX4 integration left as a future extension

</details>

---

<details>
<summary><strong>2. System Architecture</strong></summary>

<br/>

### 2.1 High-Level Components

1. **Simulation Layer (Python)**
   - 6-DOF rigid-body dynamics model for a small electric fixed-wing UAV
   - Environmental models: gravity, atmosphere, wind, gusts
   - Sensor models: IMU, GPS, barometer, magnetometer (optional), camera stub (optional)
   - Scenario runner and Monte Carlo harness

2. **Flight Software Core (C++17)**
   - State estimation layer (IMU + GPS + baro fusion) with an EKF-style interface
   - Guidance and control (cascaded loops: position → attitude → rates)
   - Mode manager (STANDBY, TAKEOFF, CRUISE, RTL, FAILSAFE)
   - Fault detection / health checks and basic FDIR hooks
   - Structured logging of states, commands, and residuals

3. **Interfaces & Middleware**
   - Lightweight IPC between simulation and FSW:
     - Initial: simple TCP or stdin/stdout-based message exchange
     - Future: ROS 2 topics/services or PX4-compatible messaging
   - Config-driven parameters via YAML/TOML for:
     - Vehicle parameters
     - Controller gains and constraints
     - Sensor noise and bias

4. **Analysis & Tooling (Python)**
   - Log parsing utilities
   - Visualization scripts for trajectories, residuals, and control usage
   - Monte Carlo result summaries and simple reports

### 2.2 Architectural Principles

- **Modularity**: Decoupled dynamics, sensors, estimation, control, and mission logic
- **Replaceability**: Estimator and controller modules can be swapped out or upgraded without changing the rest of the system
- **Observability**: Logging and analysis tools are first-class citizens, not afterthoughts
- **Reproducibility**: Single Docker-based dev environment; build and tests runnable via CI

### 2.3 Innovation Hooks

- Estimator interface designed so that:
  - A simple filter can be used initially
  - More advanced EKF/IEKF/UKF variants can be dropped in later
- Controller interface designed to support:
  - PID loops initially
  - MPC, optimal control, or RL-based policies in future iterations
- Simulation designed to:
  - Start from a generic UAS model
  - Later accept higher-fidelity aerodynamic models from literature or CFD/ID work

</details>

---

<details>
<summary><strong>3. Flight Dynamics & Environment Modeling</strong></summary>

<br/>

### 3.1 Coordinate Frames

- **Inertial / Navigation frame**: Local NED (North-East-Down)
- **Body frame**: x-forward, y-right, z-down
- Attitude representation: quaternions for numerical robustness
- Utility functions for conversions:
  - Body ↔ NED
  - Euler angles ↔ quaternions (for initialization and debugging)

### 3.2 State Definition

The core state vector will include:

- Position in NED: \( \mathbf{p}_n \)
- Velocity in NED: \( \mathbf{v}_n \)
- Attitude: quaternion \( q_{nb} \) (body relative to NED)
- Body angular rates: \( \boldsymbol{\omega}_b \)
- Optional: estimator-specific states (sensor biases, wind estimates, etc.)

### 3.3 Equations of Motion

- **Translational dynamics**:
  - Forces: aerodynamic, thrust, gravity, disturbance
  - Integration in body or NED frame with proper frame transforms
- **Rotational dynamics**:
  - Rigid-body moment equations using inertia tensor and applied moments
- **Integrator**:
  - Explicit Runge-Kutta (e.g., RK4) for clarity and stability in the sim

### 3.4 Aerodynamic Model

- Start with a simplified fixed-wing model:
  - Lift, drag, and moments as functions of:
    - Angle of attack, sideslip, and control surface deflections
  - Coefficients defined in `configs/vehicle_params.yaml`
- Allow configuration of:
  - Reference area, wingspan, chord, mass, inertia tensor
  - Coefficient curves or lookup tables
- Document how this model could be replaced by:
  - A published model (e.g., Skywalker X8)
  - A custom ID’d model from flight data

### 3.5 Environment Model

- Gravity: constant 9.81 m/s²
- Atmosphere:
  - ISA-based density vs. altitude (simplified)
- Wind:
  - Steady wind vector in NED
  - Optional gust model (band-limited noise or simple stochastic process)
- Disturbances:
  - Hooks for injecting specific disturbances (e.g., gusts at certain waypoints)

</details>

---

<details>
<summary><strong>4. Sensor & Actuator Modeling</strong></summary>

<br/>

### 4.1 Sensor Models

All sensor parameters (noise, bias, update rate) will be defined in `configs/sensor_noise.yaml`.

**IMU**

- Accelerometer:
  - Measures specific force in body frame
  - Additive white noise and bias
- Gyroscope:
  - Measures body angular rates
  - Additive white noise and bias
- Optional: slow random-walk bias drift

**GPS**

- Position and velocity in NED
- Lower update rate (e.g., 5–10 Hz)
- Additive noise and optional latency
- Optional: configurable dropouts to test robustness

**Barometer**

- Altitude derived from pressure with:
  - Noise
  - Simple bias

**Magnetometer (Optional)**

- Heading information based on a nominal Earth magnetic field model
- Adds observability for yaw in the estimator

### 4.2 Actuator Models

**Control Surfaces**

- Aileron, elevator, rudder
- Saturation limits (min/max deflection)
- First-order actuator dynamics:
  - Commanded deflection vs actual deflection (time constant)

**Throttle**

- Command mapped to thrust:
  - Simple static mapping for initial version
  - Potential upgrade to more realistic thrust model later

### 4.3 Sensor Health & FDIR Hooks

- Each sensor model will output:
  - Measurement
  - Quality flags (e.g., valid, stale, saturated)
- FSW will maintain simple health checks:
  - IMU saturation detection
  - GPS jump / dropout detection
- These hooks will feed into FAILSAFE mode triggers and basic FDIR logic

</details>

---

<details>
<summary><strong>5. Estimation & Navigation</strong></summary>

<br/>

### 5.1 Estimator Architecture

The estimator will be structured around an EKF-style interface:

- `predict(dt, imu_data)`
- `update_gps(gps_data)`
- `update_baro(baro_data)`
- Optionally: `update_mag(mag_data)`

Initial implementation:

- Simple filter (e.g., complementary or reduced-order EKF)
- Focus on:
  - Fusing IMU with GPS & baro
  - Producing a consistent navigation state for the controller

The interface will be designed so that:

- Internals can be upgraded to a more sophisticated EKF without changing callers
- Advanced features like wind estimation and bias estimation can be added later

### 5.2 Navigation State

The estimator will provide:

- Position and velocity in NED
- Attitude quaternion and derived Euler angles
- Optional estimates:
  - Biases (gyro and accelerometer)
  - Wind components

### 5.3 Frame Transform Handling

- Centralized utilities for:
  - NED ↔ body transformations
  - Quaternion ↔ Euler conversions
- This provides a place to demonstrate:
  - Clear understanding of frame transformations
  - Handling of singularities and numerical stability concerns

### 5.4 Error Budgets & Performance Metrics

Key metrics to track:

- RMS position error vs. reference trajectory
- RMS attitude error (roll/pitch/yaw)
- Convergence time after disturbances or sensor dropouts
- Estimator residuals (measurement vs predicted measurement)

These metrics will be computed by the analysis tools based on logs, and summarized for single runs and Monte Carlo campaigns.

</details>

---

<details>
<summary><strong>6. Guidance, Control & Modes</strong></summary>

<br/>

### 6.1 Mode Manager

Finite-state machine with the following modes:

- `STANDBY`:
  - Pre-flight state, motors off
- `TAKEOFF`:
  - Climb to a safe altitude and airspeed
- `CRUISE`:
  - Waypoint-following mission
- `RTL` (Return-to-Launch):
  - Navigate back to launch point and loiter
- `FAILSAFE`:
  - Triggered by critical faults (e.g., GPS loss, estimator divergence)

Mode transitions will be driven by:

- Mission timeline
- Health flags (sensor health, estimator status)
- Simple safety conditions (minimum altitude, minimum airspeed, etc.)

### 6.2 Guidance Logic

- Waypoint-based navigation:
  - List of waypoints in NED
  - Lateral guidance:
    - Initial: heading hold towards next waypoint
    - Future: L1 / lookahead-based lateral path tracking
  - Vertical guidance:
    - Altitude tracking with climb and descent rate limits

### 6.3 Control Architecture

Cascaded control loops:

- Outer loop:
  - Position / path error → desired track, altitude, and speed
- Middle loop:
  - Attitude commands (roll/pitch/yaw) from guidance laws
- Inner loop:
  - Angular rate controllers → actuator commands (surfaces, throttle)

All controller parameters will be:

- Configurable via `configs/controller_gains.yaml`
- Subject to unit tests for basic expected behavior (e.g., step response, saturation)

### 6.4 Future Control Extensions

The design will accommodate more advanced controllers, e.g.:

- MPC (model predictive control) for attitude or trajectory
- RL-based controllers for specific phases (e.g., gust rejection, landing flare)
- Gain scheduling across flight conditions

</details>

---

<details>
<summary><strong>7. Software Architecture & Interfaces</strong></summary>

<br/>

### 7.1 C++ Flight Software Layout

Namespace: `aquila::`

Key modules (headers under `fsw/include/aquila/`):

- `config.hpp`  
- `state.hpp`  
- `sensors.hpp`  
- `estimator.hpp`  
- `controller.hpp`  
- `modes.hpp`  
- `logger.hpp`

Key source files (under `fsw/src/`):

- `main_loop.cpp` – orchestrates the FSW loop
- Implementations for config, sensors, estimator, controller, modes, logger

FSW responsibilities:

- Receive sensor messages from sim (via IPC)
- Update estimator and navigation state
- Run mode manager and guidance logic
- Compute actuator commands and send them back to the sim
- Log relevant data each cycle

### 7.2 Simulation–FSW Interface (Initial Version)

Initial implementation will use a simple, inspectable IPC mechanism:

- Option A:
  - Python sim launches FSW as a subprocess
  - Sends sensor messages over stdin
  - Receives actuator commands over stdout
- Option B:
  - TCP or UDP socket between sim and FSW

Messages:

- Structured as compact text or binary (e.g., CSV lines or simple JSON)
- Designed so that they can later be replaced with ROS 2 messages without changing core logic

### 7.3 Future ROS 2 / PX4 Style Integration

Planned future extension:

- ROS 2 node wrapping the FSW core:
  - Subscribes to sensor topics
  - Publishes actuator command topics
- Sim publishes synthetic sensor data as ROS 2 topics
- This mirrors PX4–ROS 2 integration patterns and makes the project feel “drop-in” for robotics systems

</details>

---

<details>
<summary><strong>8. Testing & Verification</strong></summary>

<br/>

### 8.1 Testing Levels

**Unit Tests (C++ & Python)**

- Controllers:
  - Step response behavior
  - Saturation and limits
- Estimator:
  - Convergence in noiseless scenarios
  - Stability under simple disturbances
- Dynamics and sensors:
  - Basic sanity checks (e.g., free-fall under gravity when thrust = 0)

**Scenario Tests**

- Nominal mission:
  - Takeoff → waypoint mission → RTL under moderate wind
- Disturbance scenario:
  - Gust injections at specific waypoints
- Failure scenario:
  - GPS dropout for a window of time
  - Baro bias jump

### 8.2 Monte Carlo Campaigns

Monte Carlo runs will vary:

- Initial position/attitude
- Wind speed and direction
- Mass and inertia (within bounds)
- Sensor noise parameters

Metrics:

- Mission success/failure
- Max and RMS tracking errors
- Control saturation frequency and duration

Results will be summarized by:

- Python scripts under `tools/`
- Optional Jupyter notebooks for visual exploration

### 8.3 Continuous Integration

A simple CI pipeline (e.g., GitHub Actions) will:

- Build the C++ code and run `ctest`
- Run Python unit tests (e.g., `pytest`)
- Optionally run a “smoke test” scenario end-to-end in Docker

</details>

---

<details>
<summary><strong>9. Extensibility & Future Work</strong></summary>

<br/>

Planned extension areas:

- **Higher-fidelity aerodynamics**
  - Replace the initial aero model with:
    - A published UAV model (e.g., Skywalker X8)
    - A custom ID’d model from wind-tunnel or flight test data

- **ROS 2 / gazebo-class Integration**
  - Wrap FSW as a ROS 2 node
  - Drive sim and FSW via ROS 2 topics/services
  - Optionally visualize in a 3D sim environment

- **Detect-and-Avoid / BVLOS Logic**
  - Add synthetic intruder aircraft and airspace constraints
  - Implement simple detect-and-avoid behaviors
  - Tie into FAILSAFE and RTL logic

- **System-of-Systems Scenarios**
  - Model multiple UAS and ground assets
  - Explore simple system-of-systems performance questions

- **RL / Advanced Control Experiments**
  - Use the simulation as a training environment for:
    - RL policies on specific tasks (e.g., landing, gust rejection)
    - Hybrid classical + learning-based controllers

Each of these extensions can be scoped as an independent “project chapter” that you can reference in conversations with hiring managers and technical leads.

</details>