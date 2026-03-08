# System Architecture

## 1. Overall Software Architecture
This project is a medical FUE (Follicular Unit Extraction) micromotor controller running on an STM32F103C8T6 microcontroller. The software is designed around a decoupled, modular architecture using a global shared memory model (`m_SharedMemory.c / .h`) to pass data between various subsystems.

The system is organized into the following key modules:
- **`main.c` (Application & State Management):** Implements the non-blocking main loop (`while(1)`). It acts as the high-level coordinator, processing serial communications, evaluating the watchdog, handling low-priority tasks (LED blink, IO scanning), and executing the high-level Operation State Machine.
- **`m_IO.c` (Hardware Abstraction):** Interfaces directly with peripherals. It initializes and manages ADC (via DMA), TIM1 (PWM generation), TIM2 (Control Loop time base), and TIM4 (Encoder reading). It contains functions to filter ADC inputs (Median + LPF) and compute actual velocity and position.
- **`m_MotorControl.c` (Real-Time Control):** Implements trajectory generators and PID control loops for both position and velocity. It runs the Motor Control State Machine based on high-level commands.
- **`m_SerialComm.c` (Communication):** Handles USART3 communication with the graphical user interface. Commands are parsed and placed into shared memory.
- **`m_SharedMemory.c` (Data Exchange):** Provides a globally accessible `sm` (Shared Memory) structure. This decouples the real-time control loop from the high-level logic, avoiding direct function calls between the Main Loop and Control Loop. It also manages non-volatile parameters using the MCU's internal flash.

## 2. Interaction Between Control Loop (TIM2) and Main Loop
The architecture heavily relies on the separation of real-time critical tasks and non-critical application tasks.

### Control Loop (TIM2 ISR)
Timer 2 (TIM2) generates periodic interrupts that serve as the fundamental time base for the control loop (`HAL_TIM_PeriodElapsedCallback`). Because it runs in an ISR, it preempts the Main Loop to guarantee hard real-time performance.
At every interrupt tick, it executes:
1. **`UpdateADC_FromDMA_Task()`:** Applies median and low-pass filters to the raw ADC values (potentiometer, motor voltage, motor current) acquired by DMA.
2. **`UpdateEncoder_Task()`:** Reads TIM4 to calculate the accumulated position and instantaneous velocity (RPM), incorporating a silent overflow protection to maintain long-term accuracy.
3. **`MotorControl_Task()`:** Executes the selected control law (Velocity PI Control, Position P Control, Trajectory Generation) and computes the new PWM duty cycle, which is applied immediately.

### Main Loop (`while(1)`)
The Main Loop runs purely in user context without blocking. It relies on `HAL_GetTick()` to execute tasks at defined intervals:
- **Serial Transmission (50ms):** Sends real-time telemetry (velocity, position, PWM, current) to the GUI.
- **State Loop (`STATE_LOOP_PERIOD_MS`):** Runs the high-level `operation_state` machine, deciding which mode the system should be in based on inputs (pedal, GUI commands).
- **Watchdog:** Constantly monitors communication. If the motor is active and communication is lost for more than 1 second, it forces an emergency stop.
- **Data Race Protection:** When a serial packet is ready, it briefly disables interrupts (`__disable_irq()`) to process the packet, ensuring the Control Loop does not read partially updated parameters.

### Interaction
The two loops communicate **exclusively through the `sm` (Shared Memory) structure**.
- The Main Loop decides *what* needs to be done. It sets targets (e.g., `sm.ref_velocity`), modes (`sm.control_mode`), and triggers state changes (e.g., setting `sm.act_motor_state = MOT_STATE_VEL_CONTROL_INIT`).
- The Control Loop decides *how* to do it. It reads the targets from `sm`, computes the required PWM, and writes the *actual* physical state back into `sm` (e.g., `sm.act_velocity`).
- The Main Loop then reads this actual state to update the GUI or to determine when a movement is complete (e.g., when `sm.act_motor_state` transitions back to `MOT_STATE_IDLE`).

## 3. State Machine Transitions

The system operates using two distinct, hierarchical state machines.

### 3.1. Operation State Machine (Main Loop)
Located in `main.c`, this state machine manages the overall clinical operation.
- `OP_STATE_IDLE`: System is waiting for user input (pedal press or GUI "Start"). Based on `sm.control_mode`, it branches into one of three paths:
  - **Continuous Mode:**
    - `OP_STATE_CONT_MODE_INIT`: Commands the motor controller to initialize velocity control.
    - `OP_STATE_CONT_MODE`: Waits while the motor runs. If the pedal is released or a stop command is received, it triggers the end sequence.
    - `OP_STATE_CONT_MODE_END`: Waits for the motor to physically stop before returning to `IDLE`.
  - **Oscillation Mode:**
    - `OP_STATE_OSC_MODE_VEL_CONTROL_INIT`: Initializes a timed velocity phase.
    - `OP_STATE_OSC_MODE_VEL_CONTROL`: Runs velocity control for a specific time (`sm.time_delay`).
    - `OP_STATE_OSC_MODE_VEL_CONTROL_END`: Stops the velocity phase.
    - `OP_STATE_OSC_MODE_POS_CONTROL_INIT`: Initializes the position-based oscillation control.
    - `OP_STATE_OSC_MODE_POS_CONTROL`: The motor oscillates. Waits for a stop command or pedal release to trigger the end sequence.
    - `OP_STATE_OSC_MODE_POS_CONTROL_END`: Waits for the motor to stop before returning to `IDLE`.
  - **Manual Mode:**
    - `OP_STATE_MAN_MODE`: Motor is controlled directly via a raw PWM/duty cycle command. Waits for stop command to return to `IDLE`.

### 3.2. Motor Control State Machine (Control Loop - TIM2)
Located in `m_MotorControl.c` (`MotorControl_Task()`), this state machine executes the low-level physical control corresponding to the commands set by the Operation State Machine.
- `MOT_STATE_IDLE`: Motor is stopped (`DriveMotor(0.0f)`).
- `MOT_STATE_VEL_CONTROL_INIT`: Clears PID integrators and resets trajectories (`InitControlVel()`). Transitions immediately to `MOT_STATE_VEL_CONTROL`.
- `MOT_STATE_VEL_CONTROL`: Executes `TrajectoryGeneratorVel()` to generate a smooth ramp to `sm.ref_velocity` and feeds it into the `VelocityControl()` PI loop.
- `MOT_STATE_VEL_CONTROL_END`: Uses the trajectory generator to smoothly ramp the reference velocity down to `0.0f`. Once the actual velocity is within a deadband, it transitions back to `MOT_STATE_IDLE`.
- `MOT_STATE_POS_CONTROL_INIT`: Captures the current absolute position as the origin (`act_position_offset`) and resets trajectories. Transitions to `MOT_STATE_POS_CONTROL`.
- `MOT_STATE_POS_CONTROL`: Computes relative position and runs the `RunOscillationTrajectory()` logic, tracking position setpoints.
- `MOT_STATE_POS_CONTROL_END`: Brakes the motor to `0.0f` velocity. Once stopped, returns to `MOT_STATE_IDLE`.
- `MOT_STATE_TIME_OSC_CONTROL_INIT`: Initializes time-based oscillation tracking and resets timers. Transitions to `MOT_STATE_TIME_OSC_CONTROL`.
- `MOT_STATE_TIME_OSC_CONTROL`: Executes `RunOscillationTrajectory_Time()`, directly driving velocity based on elapsed time without strict position feedback constraints.
- `MOT_STATE_MANUAL_CONTROL`: Bypasses all PID loops. Simply drives the motor using `sm.duty_cycle_command`.