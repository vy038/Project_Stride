# Project Stride

![CAD](https://img.shields.io/badge/CAD-done-555?style=flat-square)
![Firmware](https://img.shields.io/badge/firmware-0%25-555?style=flat-square)
![Wiring](https://img.shields.io/badge/wiring-not_started-555?style=flat-square)
![Status](https://img.shields.io/badge/status-pre--bringup-orange?style=flat-square)

A 2-DOF robotic leg built to learn closed-loop FOC over CAN with real encoder feedback. Talos was open-loop servo control on ESP32. Stride is voltage-mode field-oriented control on STM32, position feedback from magnetic encoders, joints talking over CAN instead of one shared I2C bus.

Right now this repo is CAD and nothing else. No firmware, no wiring, no bring-up. This README locks in the design before any of that starts.

<img width="466" height="571" alt="image" src="https://github.com/user-attachments/assets/cd6b38c0-7b3d-49c7-a78a-2d96d0c87323" />
<img width="466" height="571" alt="image" src="https://github.com/user-attachments/assets/76dfb38e-eb79-4beb-a0b3-a90ee20fa0e5" />

---

## Status

| Piece | Status |
|---|---|
| Joint mechanical design (housing, hollow shaft, magnet carrier) | ✅ Done |
| Full leg CAD assembly | ✅ Done (pending first print/fit check) |
| Firmware | ❌ Not started |
| Wiring / harness | ❌ Not started |
| Single-motor open-loop spin test | ❌ Not started |
| Closed-loop PID (single joint) | ❌ Not started |
| CAN bus, two-joint coordination | ❌ Not started |
| CfC/LNN controller swap-in | ❌ Not started (C port exists, untested on any hardware) |


---

## Why this exists

Talos proved I can build a walking robot end to end: RTOS, IK, vision, a working demo. What it didn't prove is that I can do real closed-loop motor control. No current sensing, no encoders, no CAN, no control theory beyond "set servo angle." Stride closes that gap on a single leg.

---

## Hardware

| Component | Details |
|---|---|
| Actuators | 2x GM2804H-100T gimbal motors |
| Motor drivers | 2x SimpleFOC Mini (DRV8313). No current sensing, so this is voltage-mode FOC, not torque control |
| Encoders | 2x AS5600, I2C, fixed address 0x36, so two separate I2C buses are required |
| CAN | SN65HVD230 transceivers, CANable USB-to-CAN adapter, 120Ω termination at both bus ends |
| Controller | STM32 Nucleo |
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

- Both AS5600 encoders share I2C address 0x36, so two independent I2C buses are non-negotiable. No clever addressing trick gets around it.
- Encoder I2C lines run close to BLDC phase leads switching at PWM frequency. That's a potential noise problem, and it needs routing/shielding attention, not an afterthought.
- CAN bus needs 120Ω termination at both physical ends, not just somewhere on the bus.
- AS5600 at 400kHz I2C fast mode might not keep up with a 10 to 20kHz FOC loop. That gets validated in bring-up, not assumed away.

---

## Firmware plan

STM32, bare superloop plus timer ISR. FreeRTOS is off the table here: a 10 to 20kHz deterministic control loop and RTOS scheduling jitter don't mix. Talos already proved I know how to use an RTOS when it's the right tool. This isn't that.

**Timer ISR (10 to 20kHz):**
- FOC / commutation
- Per-joint PID
- Encoder reads

**CAN RX interrupt:**
- Updates joint setpoints from PC commands

**Modules:**
- AS5600 driver, with magnet-detect fault handling
- FOC / commutation layer. SimpleFOC abstractions vs. hand-rolled Clarke/Park/SVPWM is still an open decision
- Per-joint PID
- CAN protocol
- Safety / watchdog

Clean interface boundaries are a hard requirement, not a nice-to-have. The PID layer is meant to eventually get swapped for a CfC (liquid neural network) controller without touching anything else in the stack. That swap happens after closed-loop PID is proven on hardware, not before.

**PC side (Python):** python-can interface, trajectory/command generator, IK solver, logger/plotter for step-response and PID tuning.

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

---

## Planned project structure

```
Project_Stride/
├── cad/                     # OnShape exports, joint + leg assembly
├── firmware/
│   └── stm32/
│       ├── main.c
│       ├── foc/             # commutation, Clarke/Park/SVPWM
│       ├── encoder/         # AS5600 driver
│       ├── control/         # per-joint PID, CfC drop-in later
│       ├── can/             # protocol + RX interrupt handling
│       └── safety/          # watchdog, fault handling
├── pc/
│   ├── can_interface.py     # python-can wrapper
│   ├── trajectory.py
│   ├── ik.py
│   └── logger.py            # step-response plotting for PID tuning
├── hardware/
│   └── diagrams/            # wiring, CAN topology
└── docs/
```
