# TECHNICAL SPECIFICATION: BIOMIMETIC TENDON-DRIVEN SERVO HAND PLATFORM

---

## 1. System Architecture & Core Specifications

The platform is a fully biomimetic end-effector designed to mirror human musculoskeletal anatomy. It utilizes high-torque digital coreless micro-servo motors coupled to low-friction braided Dyneema/UHMWPE tendon cables and passive elastic antagonistic return bands, integrated directly with precision composite/3D-printed skeletal structures.

| Technical Parameter | Specification Value |
| --- | --- |
| **Degrees of Freedom (DoF)** | 27 DoF (Exact anatomical parity with the human hand) |
| **Actuation Mechanism** | High-Torque Micro-Servo Actuators with Tendon Bowden-Routing |
| **Servo Units / Channels** | Modular 16–24 Channel Forearm / Chassis Servo Bank |
| **Total Assembly Weight** | < 2 lbs (~880 g) including motors and brackets |
| **Max Grip Force** | ~6.8 kg (15 lbs) cumulative fingertip power grasp |
| **Structural Framework** | Molded carbon-fiber/PEEK or tough engineering resin bones |
| **Tendon Tensile Rating** | 50–80 lb (220–350 N) braided UHMWPE Dyneema (Ø0.8–1.0 mm) |
| **Fatigue Threshold** | 650,000+ active operational cycles |

---

## 2. Actuation & Tendon-Driven Servo Infrastructure

* **Servo Motor Architecture:** High-torque metal-gear micro servos (e.g., KST DS215MG / KingMax CLS0612W / MG90S Digital Coreless) operating at 6.0V–8.4V.
* **Tendon Spool / Winch Horns:** CNC aluminum or high-strength resin spools (Ø10–14 mm) delivering high linear force and precise tendon displacement (up to 28 mm active stroke).
* **Antagonistic Compliance & Elastic Return:**
  - Active flexor pull driven by dedicated servo motors.
  - High-cycle silicone/nitrile elastomer bands or dual-rate helical torsion springs on dorsal joint hinges provide instant extension return, passive shock compliance, and back-drivability.
* **Tendon Bowden Routing:** Low-friction PTFE liners (OD 2.0 mm, ID 1.0 mm) pass through the wrist articulation gimbal, preserving zero cable length variance during wrist pitch, yaw, and roll movements.
* **Response Dynamics:** High-speed servo latency under **40 milliseconds** with position accuracy of ±0.15°.

---

## 3. Degrees of Freedom (DoF) & Kinematic Distribution

The 27 DoF layout maps directly to human anatomical articulation:

* **Thumb (5 DoF):** Complete carpometacarpal (CMC 2-DoF: palmar abduction/adduction + true opposition/circumduction), condyloid metacarpophalangeal (MCP 2-DoF: flexion/extension + abduction), and interphalangeal (IP 1-DoF: flexion/extension).
* **Digits (4 DoF per finger × 4 = 16 DoF):** Metacarpophalangeal (MCP 2-DoF: flexion/extension + lateral abduction/adduction), proximal interphalangeal (PIP 1-DoF), and distal interphalangeal (DIP 1-DoF) curling.
* **Palm Geometry (2 DoF):** Organic bowl-shaped folding palmar arch allowing active 4th & 5th metacarpal cupping and adaptive thenar compliance.
* **Wrist Interface (4 DoF):** Flexion, extension, radial/ulnar deviation, combined with structural forearm pronation and supination.

---

## 4. Control Systems & Electronics

* **Servo Controller Integration:** 16–24 Channel 12-bit PWM PCA9685 / CAN-Bus / UART Bus servo driver boards linked to an onboard ESP32-S3 or ARM Cortex-M4 microcontroller.
* **Current & Force Monitoring:** Integrated per-channel shunt current sensing allows closed-loop stall prevention, grip force regulation, and object contact detection.
* **Magnetic Encoders:** Contactless AS5600 absolute magnetic angle encoders positioned at the MCP and CMC joint hinges provide real-time angular telemetry.
* **Neural / Teleoperation Controller:** Maps vision-based human hand tracking and intent trajectories directly into coordinated multi-channel PWM pulse widths without kinematic singularities.

---

## 5. Tendon & Servo Channel Mapping Table

| Channel ID | Muscle Group Equivalence | Servo Location | Joint / DoF Actuated | Motion Range |
|---|---|---|---|---|
| **CH01** | Flexor Digitorum Profundus (Index) | Forearm Bay 1 | Index PIP & DIP Flexion | $0^\circ - 180^\circ$ |
| **CH02** | 1st Dorsal Interosseous (Index) | Palm / Forearm Bay 2 | Index MCP Abduction / Lateral Spread | $-15^\circ - +15^\circ$ |
| **CH03** | Flexor Digitorum Superficialis (Index) | Forearm Bay 3 | Index MCP Flexion / Grip Pre-curl | $0^\circ - 90^\circ$ |
| **CH04** | Flexor Digitorum Profundus (Middle) | Forearm Bay 4 | Middle PIP & DIP Flexion | $0^\circ - 185^\circ$ |
| **CH05** | 2nd & 3rd Interossei (Middle) | Palm / Forearm Bay 5 | Middle Radial/Ulnar Deviation | $-10^\circ - +10^\circ$ |
| **CH06** | Flexor Digitorum Superficialis (Middle) | Forearm Bay 6 | Middle MCP Flexion | $0^\circ - 90^\circ$ |
| **CH07** | Flexor Digitorum Profundus (Ring) | Forearm Bay 7 | Ring PIP & DIP Flexion | $0^\circ - 185^\circ$ |
| **CH08** | Interossei (Ring) | Palm / Forearm Bay 8 | Ring MCP Abduction / Spread | $-12^\circ - +12^\circ$ |
| **CH09** | Flexor Digitorum Superficialis (Ring) | Forearm Bay 9 | Ring MCP Flexion | $0^\circ - 90^\circ$ |
| **CH10** | Flexor Digitorum Profundus (Pinky) | Forearm Bay 10 | Pinky PIP & DIP Flexion | $0^\circ - 180^\circ$ |
| **CH11** | Abductor Digiti Minimi (Pinky) | Forearm Bay 11 | Pinky MCP Abduction / Spread | $-15^\circ - +15^\circ$ |
| **CH12** | Opponens Digiti Minimi (Palmar Arch) | Palm Base / Forearm Bay 12 | 4th/5th Metacarpal Cupping | $0^\circ - 30^\circ$ |
| **CH13** | Flexor Pollicis Longus (Thumb IP) | Forearm Bay 13 | Thumb Terminal IP Flexion | $0^\circ - 85^\circ$ |
| **CH14** | Flexor Pollicis Brevis (Thumb MCP) | Forearm Bay 14 | Thumb MCP Flexion | $0^\circ - 60^\circ$ |
| **CH15** | Opponens Pollicis (Thumb CMC) | Forearm Bay 15 | Thumb Pronation / Opposition | $-30^\circ - +40^\circ$ |
| **CH16** | Abductor Pollicis Brevis (Thumb CMC) | Forearm Bay 16 | Thumb Palmar Abduction | $0^\circ - 60^\circ$ |
| **CH17** | Adductor Pollicis (Thumb Power) | Palm Bay 17 | Thumb Transverse Adduction Pinch | $0^\circ - 35^\circ$ |
| **CH18** | Flexor Carpi Radialis / Ulnaris | Forearm Mount A | Wrist Flexion / Extension Pitch | $-45^\circ - +45^\circ$ |
| **CH19** | Extensor Carpi Radialis / Ulnaris | Forearm Mount B | Wrist Radial / Ulnar Yaw | $-35^\circ - +35^\circ$ |
| **CH20** | Pronator Teres / Quadratus | Forearm Mount C | Forearm Pronation / Supination Roll | $-85^\circ - +85^\circ$ |
