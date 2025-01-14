# Algae Handler Function
Vertical Position -> in robot perimeter
Horizontal Position -> out of robot perimeter
- Starting Position
  - No Motor Movement: No Spin
  - Solenoid: Vertical Position
- Intake Position
  - Motor Movement: Spin
  - Solenoid: Horizontal Position
- Has Algae
  - Hit Beam Brake
    - Stop Motor
    - No Motor Movement: No Spin
    - Solenoid: Vertical Position
  - Hall Effect Sensor: When magnet gets detected
- Holding Position
  - No Motor Movement: No Spin
  - Solenoid: Vertical Position
  - Hold
    - Limit and Hall Effect both switched: Stop Running
    - Hall Effect w/o Limit: Run until Limit pressed
- Outtake Position
  - Motor Movement: Neg. Spin -> Timed
  - Solenoid: Vertical Position
  - End Hold
# Coral Handler Function
Intake Position -> Down Position
Outtake Lvl 1 Position -> 45 Angle Outtake (+ error)
- Starting Position
  - LR Motor: Right
  - Outtake Motor: No Spin
  - Vertical Motor: Intake Position
  - Elevator Height/Level?
- Intake Position
  - LR Motor: Right
  - Outtake Motor: Neg. Spin
  - Vertical Motor: Intake Position
  - Elevator Height/Level?
- Left Through Position
  - LR Motor: Base off of Odom
  - Motor: Spin
  - Vertical Motor: Outtake Lvl 1 Position
  - Elevator Height/Level?
- Right Through Position
  - LR Motor: Base off of Odom
  - Motor: Spin
  - Vertical Motor: Outtake Lvl 1 Position
  - Elevator Height/Level?
- Reef Lvl 2/3 Position
  - LR Motor: Base off of Odom
  - Motor: Spin
  - Vertical Motor: Outtake Lvl 2/3 Position
  - Elevator Height/Level?
- Reef Lvl 4 Position
  - LR Motor: Base off of Odom
  - Motor: Spin
  - Vertical Motor: Outtake Lvl 4 Position
  - Elevator Height/Level?
- Possible Ending Position?
# Elevator Function
# Climber Function
# LEDs Function
## Ideas
- When coral is in the coral mechanism
- Fun rainbow when robot is connected