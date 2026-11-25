# Montrac: questions on the real system

- **Purpose:** gather what is needed to drive the real Montrac from ROS 2 and to make a faithful simulation of it.
- **Work package:** L4 Intralogistics (Montrac).
- **Contact:** the MFJA platform manager. Reference documents: the platform wiki on Moodle (room 315).

## Context

Plan agreed with the client on 13/11/2026:

1. write a simple program, without ROS, that drives half of the Montrac;
2. turn it into a ros2_control hardware interface (`hardware_interface::SystemInterface`);
3. create a `montrac_description` package whose xacro argument `gz_sim` loads either the real hardware
   interface or the Gazebo plugin. This package replaces the current kinematic controller.

The simulation must therefore offer exactly the commands and state feedback of the real system. The current
twin (repository `mfja_3rd_floor_gz`, branch `INTERNSHIP-ALI-2026`) relies on assumptions never checked on
the hardware (section 12). This sheet is used to confirm or correct them.

**Use:** if time is short, start with section 1. Answers go in the "Answer" column; the completed sheet is
then committed.

## Leads (to confirm)

From the repository [mfja_ros2_yaskawa](https://github.com/aip-primeca-occitanie/mfja_ros2_yaskawa) (script
`supervision_IHM.py`, a student HMI marked "not tested") and from the platform manager. Even official
information may be incomplete. Network addresses are on the platform wiki and are not copied here.

| Lead | Source | Related questions |
|---|---|---|
| A "master PLC" answers in **Modbus TCP**; the script reads one coil (coil 0) as a start order and writes one coil (coil 1) to signal the end of a cycle. Is it the PLC that drives the Montrac? | `supervision_IHM.py` | 1.1, 1.2, 3.1, 3.3, 3.4 |
| A **Turck IO-Link gateway** answers in Modbus TCP (write register 2049, read register 1); it drives a Zimmer gripper. Is it also used for the Montrac (IO-Link modules)? | `supervision_IHM.py` | 1.2, 8.2 |
| The room 315 devices (robots, gateway, PLC) share one local network; addressing plan on the platform wiki. | Platform wiki, `mfja_ros2_yaskawa` | 3.5 |

## 1. Priority questions

The answers to 1.3 and 1.4 decide the ros2_control interface. The current twin commands each shuttle
individually and its sensors report which shuttle passes. If the real system cannot do that, the simulation
must be changed accordingly.

| No. | Question | Answer |
|---|---|---|
| 1.1 | What drives the Montrac today: a PLC (make, model, CPU)? One PLC per cell or one for both? Is there an operator panel or HMI? | Lead: "master PLC" (see leads) |
| 1.2 | How can a PC read the sensors and command the actuators: Ethernet (Modbus TCP, OPC UA, S7, Profinet), IO-Link master, free digital I/O, other? Which one is allowed at the MFJA? | Lead: Modbus TCP, already used by a student script towards the master PLC |
| 1.3 | Can a shuttle be stopped or restarted individually (on-board control, infrared module, power per section)? Or only with the stoppers, at fixed points? | |
| 1.4 | Do the sensors identify the shuttle (RFID, code, colour) or only detect presence? | |
| 1.5 | Is a PLC program in service? Can its variables be read and written without changing it, or must an exchange block be added? Who may change it, and with which software? | |
| 1.6 | What does "half of the Montrac" cover for the first step: a whole cell (left or right) or part of a cell? Which one is most available? | |
| 1.7 | Test conditions: authorised person present, time slots, safety rules, restoring the cell before lab sessions? | |

## 2. Documents to collect

- [ ] PLC project (program export), symbol table and I/O list
- [ ] Electrical diagrams of the cabinet and of the cell wiring
- [ ] Manufacturer documentation (montratec): shuttles, switches, stoppers, indexing units, sensors, control modules if any
- [ ] Dimensioned drawing or CAD of both circuits, including the SolidWorks source file used for the twin (with its version)
- [ ] Network addressing plan of the cell (PLC, HMI, subnet)
- [ ] Lab subjects and solutions that use the Montrac
- [ ] Risk analysis, safety instructions and start-up procedure of the cell
- [ ] Records of any earlier attempt to connect a PC (OPC UA server, Node-RED, script, supervision)
- [ ] Modbus address table of the master PLC and role of its coils and registers
- [ ] Access to the platform Moodle (room 315 wiki) for the whole team

## 3. PLC and communication

| No. | Question | Answer |
|---|---|---|
| 3.1 | Exact reference of the PLC (model, CPU, firmware version) and of its I/O or communication modules. | |
| 3.2 | Software and version needed to open the program (TIA Portal, EcoStruxure, Codesys…). Is a licence available for the team? | |
| 3.3 | Is an OPC UA or Modbus TCP server already active on the PLC? If not, can it be enabled? | |
| 3.4 | Which variables hold the state of the sensors, switches and stoppers? Which ones can be written to command them? | |
| 3.5 | Is the cell network isolated from the university network? Can a laptop be plugged in, and on which port? | Lead: one local network for the room 315 devices; configuration on the platform wiki |
| 3.6 | PLC cycle time, and a reasonable exchange rate with a PC (10, 50 or 100 Hz?). | |
| 3.7 | If the local HMI and the PC give conflicting orders, which one has priority? Is there an "external control" mode? | |
| 3.8 | Is there a spare PLC, a simulator (PLCSIM-like) or an I/O test bench to develop without blocking the cell? | |

## 4. Operating modes and start-up

| No. | Question | Answer |
|---|---|---|
| 4.1 | Complete start-up sequence: power, compressed air, reset, mode selection. | |
| 4.2 | Available modes (manual, automatic, step by step, maintenance) and how to switch between them. | |
| 4.3 | State of the actuators at power-up: default switch position, stoppers up or down. | |
| 4.4 | What does the PLC do if it loses the link with the PC? Can a watchdog be added that puts the cell in a safe state (for example all stoppers up)? | |
| 4.5 | Normal stop sequence and state in which the cell must be left for lab sessions. | |

## 5. Shuttles

| No. | Question | Answer |
|---|---|---|
| 5.1 | Number of shuttles actually available on each cell (the twin allows up to 4 per rail). | |
| 5.2 | Shuttle model, hardware generation and power supply (powered rail, other). | |
| 5.3 | Dimensions (length, width, height), mass, maximum payload, plate size and fixing. Can a 5 cm PLA cube be placed without a holder? | |
| 5.4 | Nominal speed, possible adjustment, acceleration and stopping distance (the twin assumes a constant 0.2 m/s). | |
| 5.5 | Is the direction of travel unique? Which one, seen from above? | |
| 5.6 | Do the shuttles have on-board anti-collision? What gap remains between two shuttles queued at a stop? | |
| 5.7 | Does a shuttle move as soon as it is powered, or on command? Does it restart by itself when a stopper retracts? | |
| 5.8 | Can a shuttle be added or removed while running? With which procedure? | |
| 5.9 | Shuttle behaviour on emergency stop or power cut. | |
| 5.10 | Are the shuttles numbered or visually identifiable (label, colour)? | |

## 6. Switches (A1 to A4)

| No. | Question | Answer |
|---|---|---|
| 6.1 | Do the twin's names A1 to A4 match the real marking of each cell? Where is each switch on the plan? | |
| 6.2 | Switch type (rotating plate?) and number of positions actually used. The 3D model has 3, the twin uses only 2 (outer, inner). | |
| 6.3 | Actuator (pneumatic, electric), command signal (one or two outputs) and position feedback (limit switches?). | |
| 6.4 | Switching time (the twin assumes 0.3 s). | |
| 6.5 | Are the merging switches (A2, A4) motorised? What happens if a shuttle reaches a wrongly set switch: blocked, derailed, prevented by the PLC? | |
| 6.6 | Does the PLC forbid switching while a shuttle is on or near the switch? Which sensor provides this interlock (the twin blocks within 0.35 m)? | |

## 7. Stoppers and stations

| No. | Question | Answer |
|---|---|---|
| 7.1 | Number and position of the stoppers of each cell. The twin places 4 per rail, one before each switch; those of A2 and A4 act on both merging branches. Is that right? | |
| 7.2 | Is there a stopper at each workstation? In the twin, stations stop shuttles in software, without a stopper. | |
| 7.3 | For each stopper: actuator, command signal, rest state when air or power is cut, position sensor, actuation time (the twin assumes 0.1 s). | |
| 7.4 | How does a shuttle stop against a stopper: motor cut, slipping, cam? | |
| 7.5 | How many stations per cell, and where? The twin has 4, in two indexing pairs on the outer branches. Which robot serves which stations? | |
| 7.6 | Do the stations have an indexing unit (pallet lifter, centring pin)? With which actuators and sensors? How precisely is the shuttle positioned at a station? | |

## 8. Sensors

| No. | Question | Answer |
|---|---|---|
| 8.1 | Actual list of sensors and their names. The twin uses DZI1 to DZI4 (stations), DA1 to DA4 (switch entries), DA1E to DA4E and DA1I to DA4I (branches), and one sensor 0.10 m before each stopper. What exactly do "DZI" and "DA" mean? | |
| 8.2 | Technology of each sensor (inductive, optical, magnetic), signal type (digital PNP/NPN, IO-Link) and detected object (shuttle, plate, load). | |
| 8.3 | Exact position of each sensor (dimension on a drawing or measured on site). | |
| 8.4 | Are there sensors not modelled: load presence, switch and stopper positions, air pressure, emergency-stop state, open guard? | |
| 8.5 | Response time or filtering in the PLC (to reproduce edges in simulation). | |

## 9. Safety

| No. | Question | Answer |
|---|---|---|
| 9.1 | Emergency-stop chain: where are the buttons, what does it cut (rail power, air), how is it reset? Can the PLC read its state to pass it on to ROS? | |
| 9.2 | Guards in place (covers, barriers) and rules for working next to the robots. Is there an interlock between the robots and the Montrac? | |
| 9.3 | Which protections does the PLC already provide (interlocks, anti-collision) that our commands must never bypass? | |
| 9.4 | Conditions for a first read-only test (no command sent), then for a first test commanding one stopper or one switch. | |

## 10. Geometry and layout

| No. | Question | Answer |
|---|---|---|
| 10.1 | Which file do the SolidWorks points used by the twin come from (276 points, 14 segments)? Is it up to date with the installation? | |
| 10.2 | The 04/09/2026 commit on branch `fix-staubli-placement` moves the right rails by about 5 cm. Do these values come from a measurement on site? Should they be carried over to `INTERNSHIP-ALI-2026`? | |
| 10.3 | Height of the rail plane and position of each cell relative to the robot bases (for the robot–shuttle TF frames). | |
| 10.4 | Left/right convention: entering the room, which cell is on the left and which robots border it? The documents disagree: the internship report pairs the right rail with the HC10DT and the Stäubli, while the twin configuration and the specification dossier pair the HC10DT and the KUKA with the left rail. | |
| 10.5 | May the cell be photographed and measured (cabinet, rating plates, switches, sensors)? | |

## 11. Real-life mirror of the Montrac tutorials

The client wants each video of the `tutorial_videos` folder to have an equivalent on the real hardware. Five
videos concern the Montrac:

| Video | Simulated function | Question for the real system | Answer |
|---|---|---|---|
| 08 | Send a shuttle to a station | Can a given shuttle be brought to a given station? With which sequence of stoppers and switches? | |
| 09 | Add, start, stop, reset, remove a shuttle | Which of these operations exist on the real system? Which need a manual action? | |
| 10 | Switch safely | Real command and interlocks of the switches (section 6). | |
| 11 | Raise and lower a stopper | Real command of the stoppers (section 7). | |
| 12 | Read the occupancy sensors | Real reading of the sensors (section 8). | |

## 12. Twin assumptions to confirm

Values found in `mfja_3rd_floor_gz`, branch `INTERNSHIP-ALI-2026`, commit `2321def`.

| Item | Value in the twin | Real value or comment |
|---|---|---|
| Shuttles per rail | Up to 4 (L1–L4 left, R1–R4 right) | |
| Direction of travel | Unique: A14 → A1 → A12 → A2 → A23 → A3 → A34 → A4 → A14 | |
| Switches | 4 per rail, 2 positions (E outer, I inner) | |
| Switching time | 0.3 s | |
| Stoppers | 4 per rail, one before each switch; 0 = free, 1 = blocked | |
| Stopper actuation time | 0.1 s | |
| Stations | 4 per rail, on the outer branches (A12E, A34E); stop handled in software | |
| Sensors | Binary, and report which shuttle passes | |
| Shuttle speed | Constant 0.2 m/s, no acceleration | |
| Minimum gap between shuttles | 0.33 m (centre to centre) | |
| Protected zone around a switch | 0.35 m | |
| Capacity of an inner branch | 2 shuttles | |
| Wrongly set merging switch | The shuttle stops in fault (state `FALLING`) | |
| Emergency stop | Software stop of all shuttles | |
| Load | Box resting on the shuttle, not held | |

## 13. To record on site

- [ ] Photos of the cabinet, of the PLC rating plate and of its modules
- [ ] Photos and references of the shuttles, switches, stoppers and sensors (labels)
- [ ] Time a full shuttle lap and measure the circuit length to deduce the speed
- [ ] Film one switch change and one stopper actuation (actuation times)
- [ ] Measure the gap between two shuttles queued behind a stopper
- [ ] Dimensioned sketch of each cell with the positions of sensors, stoppers and stations

## Next steps

- Complete this sheet and commit it.
- Write `docs/montrac/HARDWARE.md`: link, messages, safety, start-up sequence.
- Record the choice of communication protocol in the decision log.
- Update the twin configuration with the real values.
