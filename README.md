# Project Stride

![CAD](https://img.shields.io/badge/CAD-done-555?style=flat-square)
![Firmware](https://img.shields.io/badge/firmware-scaffold-555?style=flat-square)
![Wiring](https://img.shields.io/badge/wiring-knee_only-555?style=flat-square)
![Status](https://img.shields.io/badge/status-knee_bring--up_done-orange?style=flat-square)

A 2-DOF robotic leg built to learn closed-loop FOC over CAN with real encoder feedback. Talos was open-loop servo control on ESP32. Stride is voltage-mode field-oriented control on STM32, position feedback from magnetic encoders, joints talking over CAN instead of one shared I2C bus.

CAD is done. The knee went through bring-up on STM32 + SimpleFOC firmware in a separate repo: closed-loop PID, CAN, and the CfC controller all ran on the bench. `firmware/` is the real firmware, and right now it's a scaffold. The pure logic is written and tested (transferred from bringup repo, see `vy038/stride_bringup`), but the drivers are stubs.

<img width="466" height="571" alt="image" src="https://github.com/user-attachments/assets/cd6b38c0-7b3d-49c7-a78a-2d96d0c87323" />
<img width="466" height="571" alt="image" src="https://github.com/user-attachments/assets/76dfb38e-eb79-4beb-a0b3-a90ee20fa0e5" />

---

## Status

| Piece | Status |
|---|---|
| Joint mechanical design (housing, hollow shaft, magnet carrier) | ✅ Done |
| Full leg CAD assembly | ✅ Done (pending first print/fit check) |
| Firmware | 🟡 Scaffold only |
| Wiring / harness | 🟡 Knee wired and run on the bench. Hip not run yet |
| Single-motor open-loop spin test | ✅ Done in bring-up (knee) |
| Closed-loop PID (single joint) | ✅ Knee, in bring-up. Hunts at hold (0.04 to 0.11 rad p2p), needs gravity feedforward and more damping |
| CAN bus, two-joint coordination | 🟡 CAN verified on the knee in bring-up. Two-joint not started |
| CfC/LNN controller swap-in | ✅ Passed the golden check on target (max err 5.1e-7) and drove the knee in bring-up. Not on the real firmware yet |


---

## Why this exists

Talos proved I can build a walking robot end to end: RTOS, IK, vision, a working demo. What it didn't prove is that I can do real closed-loop motor control. No current sensing, no encoders, no CAN, no control theory beyond "set servo angle." Stride closes that gap on a single leg.

---

## Hardware

| Component | Details |
|---|---|
| Actuators | 2x GM2804H-100T gimbal motors, 7 pole pairs |
| Motor drivers | 2x SimpleFOC Mini (DRV8313). No current sensing, so this is voltage-mode FOC, not torque control |
| Supply | 22V bench supply. Drive voltage stays at or below 3V on these motors |
| Encoders | 2x AS5600, I2C, fixed address 0x36, so two separate I2C buses are required (I2C1 knee, I2C2 hip) |
| CAN | MCP2515 + TJA1050 module on SPI2, because the F401 has no CAN peripheral. 8MHz crystal, 500kbit/s. CANable on the PC (SocketCAN `can0`). PC → CANable → bus → TJA1050 → MCP2515 → SPI2 → STM32. 120Ω termination at both bus ends |
| Controller | NUCLEO-F401RE (Cortex-M4F, 84MHz) |
| Fasteners | M2.5 clearance holes, brass heat-set inserts for repeated joint assembly |

---

## Mechanical design

- Joint housing pocket: 35.2 to 35.4mm ID for the 35mm OD GM2804 body
- Hollow shaft (5mm ID), stationary, so wiring can route straight through the joint instead of around it
- Magnet carrier: slip-fit collar (35.1mm ID) on the rotating outer bell
- AS5600 PCB mounted stationary on the stator face, roughly 1mm designed air gap to the magnet
- M2.5 clearance holes at 2.7 to 2.8mm, brass heat-set inserts so the joint survives repeated disassembly

---

## Wiring plan

Every pin is in `firmware/src/config.h`.

- Both AS5600 encoders share I2C address 0x36, so two independent I2C buses are non-negotiable. No clever addressing trick gets around it.
- Encoder I2C lines run close to BLDC phase leads switching at PWM frequency. That's a potential noise problem, and it needs routing/shielding attention, not an afterthought.
- CAN bus needs 120Ω termination at both physical ends, not just somewhere on the bus.
- The MCP2515 + TJA1050 board needs 5V on VCC, so it drives MISO and INT at 5V. 1 to 4.7kΩ in series on both before they reach the Nucleo. The 3.3V VP230 board I have is faulty.
- **I2C timing:** one AS5600 read at 400kHz takes about 0.15ms (measured in bring-up). That rules out a blocking read in a 10 to 20kHz ISR. The tick starts a DMA read and uses the result on the next tick, so it never waits on I2C. 2 to 5kHz (5kHz default) until the read is measured on the real firmware.

---

## Firmware

STM32, bare metal, PlatformIO with STM32Cube HAL/LL, C11. One build is one CAN node driving one motor. Adding a joint means one pin block in `config.h` and one env, whether it's the hip on this Nucleo or one board per joint later. No Arduino core, no RTOS. FreeRTOS is off the table here: a deterministic control loop and RTOS scheduling jitter don't mix. Talos already proved I know how to use an RTOS when it's the right tool. This isn't that.

FOC is hand-rolled voltage-mode: inverse Park + SVPWM, d = 0. Decided.

**Runtime split:**

| Context | Rate | Priority | Does |
|---|---|---|---|
| Control tick (TIM2) | 2 to 5kHz | Highest | Takes last tick's encoder read, starts the next, angle + velocity, FOC, PWM |
| I2C + DMA | Per read | Below the tick | Runs the encoder read in the background |
| PendSV | 500Hz | Below the tick | Joint state machine, PID or CfC, outputs Uq |
| Superloop | Free-running | None | CAN dispatch, STATUS/FEEDBACK, watchdog kick |
| EXTI7 (MCP2515 INT, PC7) | Per frame | Lowest | Sets a flag, nothing else |

Shared data has exactly one writer:
- Tick to everyone else: telemetry through `seqlock.h`. Readers are lower priority, so they retry instead of blocking.
- PendSV to tick: Uq. The tick outranks PendSV, so a seqlock would spin forever there. PendSV copies with interrupts masked instead.
- The joint state belongs to PendSV. CAN commands from the superloop run with PendSV masked.
- CAN RX: the superloop is the only thing that touches SPI2. It reads frames into `ring.h` and dispatches them. If the INT line initializes, the EXTI handler just sets a flag and the superloop reads until INT goes high. If not, the superloop polls the MCP2515 every pass. INT stays low while anything is pending, so a level check can't miss a frame the way an edge can.

**Safety:** IWDG kicked only from the superloop, so nothing can starve it. HardFault cuts both driver EN pins. Calibration is a state, not a blocking call. Target clamped to ±0.5 rad, enable refused outside that. CfC falls back to PID on |err| > 0.3 rad or non-finite output. Leg disables after 500ms without a host frame.

The PID layer swaps for the CfC (liquid neural network) controller without touching anything else in the stack. The CfC runs at a fixed 500Hz and checks itself against a golden vector at boot, refusing CfC mode on a mismatch.

**Build:**

```
cd firmware
pio run -e nucleo_f401re -t upload
pio test -e native                    # pure-logic tests on the host
```

**Retraining the CfC** on this firmware (log under the PID, train, reflash). The serial `log` command doesn't exist here yet.

```
python pc/capture_log.py /dev/serial/by-id/<nucleo> --runs 10 --amp 0.2
python pc/train_cfc_controller.py "<dir>/log_*.csv" --project firmware
```

The trainer overwrites `lib/cfc/weights_nobackbone.h`, `src/cfc_ctrl_io.h` and `src/cfc_golden_nb.h`. Back up a good model first. Accept at R² ≥ 0.8.

**PC side (Python):** python-can host, log capture, CfC trainer. Still planned: trajectory/command generator, IK solver, logger/plotter for step-response and PID tuning.

---

## Bring-up order

Single motor, open-loop, before anything else gets layered on:

1. Spin one GM2804 open-loop off one DRV8313. Confirm commutation direction and basic control.
2. Bring up one AS5600, validate I2C read speed and reliability at target loop rate.
3. Close the loop: PID on that single joint using encoder feedback.
4. Repeat 1 to 3 for the second joint on its own I2C bus.
5. Bring CAN online, single joint, setpoints from the PC over python-can.
6. Coordinate both joints over CAN, run a basic IK-driven trajectory.
7. Only after step 6 is solid: attempt the CfC controller swap-in.

No skipping ahead to CAN or two-joint coordination before a single motor is proven closed-loop. That's how you end up debugging three unknowns at once instead of one.

The knee bring-up covered 1 to 3, single-joint CAN, and a CfC trial, all on throwaway firmware. The real firmware goes through the list again.

---

## Project structure

```
Project_Stride/
├── CAD/                          # OnShape exports, joint + leg assembly
├── firmware/                     # PlatformIO project, NUCLEO-F401RE
│   ├── platformio.ini            # envs: nucleo_f401re, native (unit tests)
│   ├── lib/cfc/                  # vendored CfC cell, don't edit
│   ├── src/
│   │   ├── main.c                # control tick, 500Hz loop, superloop
│   │   ├── config.h              # every pin and constant
│   │   ├── ring.h                # SPSC ring for CAN RX
│   │   ├── seqlock.h             # ISR to main telemetry
│   │   ├── cfc_ctrl.c/h          # CfC controller + boot golden check
│   │   ├── cfc_ctrl_io.h         # written by the trainer, stays at src/
│   │   ├── cfc_golden_nb.h       # written by the trainer, stays at src/
│   │   ├── drivers/              # hardware: i2c, spi, pwm3, as5600, mcp2515, wdg + HardFault
│   │   ├── bldc/                 # pure logic: foc, calib, pid, joint
│   │   └── can/                  # can_proto.h, can_node
│   └── test/                     # native unit tests
├── pc/
│   ├── can_host.py               # CAN host, mirrors can_proto.h
│   ├── capture_log.py            # training log capture over serial
│   ├── train_cfc_controller.py   # CfC trainer
│   ├── cfc_export.py             # used by the trainer
│   └── make_golden_header.py     # used by the trainer
└── hardware/
    └── diagrams/                 # wiring
```
