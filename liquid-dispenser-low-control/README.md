# Liquid Dispenser Low Control - STM32 (Nucleo-h723zg)

This project implements a low-level control system for a Ping podlet using an STM32 Nucleo-H723ZG development board.

---

## Hardware
- Automated Liquid Dispenser control
  - 24VDC Gearmotor with Quadrature Encoder
  - XY160D Motor Driver
- XY Gantry
  - NEMA 23 Stepper Motors
  - DM556 Motor Drivers
  - Limit Switches
- Cup Dispense
  - FET w/ OTS Cup Dispenser
  - IR Sensor
- Lid Dispense
  - 24VDC Gearmotor with Quadrature Encoder
  - XY160D Motor Driver
- Lid Sealer
  - 24VDC Gearmotor with Quadrature Encoder
  - XY160D Motor Driver
  - Venturi suction
- Water/Milk Dispense
  - FET w/ OTS valve 
  
---

## Networking
- 4x STM32s communicate with a high level controller over ethernet
- JSON messages over UDP

---

# Import into STM32CubeIDE 
1. Open STM32CubeIDE
2. File > New > STM32 Project from Existing Files
3. In the dialog:
  Project Name: liquid-dispenser-low-control
  Existing Code Location: Browse and select the root folder you downloaded or cloned
  Choose: "Copy projects into workspace"
4. Click Finish
5. STM32CubeIDE will detect the .ioc file and load the configuration automatically
   
_if prompted, let the IDE install any misisng firmware packages_
