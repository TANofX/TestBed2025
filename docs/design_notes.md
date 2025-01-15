> [!NOTE]
> All information in this document are prototype based ideas and must be taken with caution...

> [!WARNING]
> Elevator Function has not been planned out yet due to insufficient information on prototype/elevator mechanism
<details>
<summary>:frog: Algae Handler Function</summary>
  Vertical Position -> in robot perimeter
  \
  Horizontal Position -> out of robot perimeter

### Starting Position
  - No Motor Movement: No Spin
  - Solenoid: Vertical Position

### Intake Position
  - Motor Movement: Spin
  - Solenoid: Horizontal Position

### Has Algae
  - Hit Beam Brake
    - Stop Motor
    - No Motor Movement: No Spin
    - Solenoid: Vertical Position
  - Hall Effect Sensor: When magnet gets detected

### Holding Position
  - No Motor Movement: No Spin
  - Solenoid: Vertical Position
  - Hold
    - Limit and Hall Effect both switched: Stop Running
    - Hall Effect w/o Limit: Run until Limit pressed

### Outtake Position
  - Motor Movement: Neg. Spin -> Timed
  - Solenoid: Vertical Position
  - End Hold
</details>
<details>
<summary> :shell: Coral Handler Function </summary>
  Intake Position -> Down Position
  \
  Outtake Lvl 1 Position -> 45 Angle Outtake (+ error)

### Starting Position
  - LR Motor: Right
  - Outtake Motor: No Spin
  - Vertical Motor: Intake Position
  - Elevator Height/Level?

### Intake Position
  - LR Motor: Right
  - Outtake Motor: Neg. Spin
  - Vertical Motor: Intake Position
  - Elevator Height/Level?

## :fishing_pole_and_fish: Reef Positioning States

### Left Trough Position
  - LR Motor: Base off of Odom
  - Motor: Spin
  - Vertical Motor: Outtake Lvl 1 Position
  - Elevator Height/Level?

### Right Trough Position
  - LR Motor: Base off of Odom
  - Motor: Spin
  - Vertical Motor: Outtake Lvl 1 Position
  - Elevator Height/Level?

### Reef Lvl 2/3 Position
  - LR Motor: Base off of Odom
  - Motor: Spin
  - Vertical Motor: Outtake Lvl 2/3 Position
  - Elevator Height/Level?

### Reef Lvl 4 Position
  - LR Motor: Base off of Odom
  - Motor: Spin
  - Vertical Motor: Outtake Lvl 4 Position
  - Elevator Height/Level?
  > ⚠️
  > Do not know about a possible Ending Position.

- Possible Ending Position?
</details>
<details>
<summary> :diving_mask: Elevator Function </summary>

### Intake Position
  - 

### Left/Right Trough Position
  -
  
### Level 2 Position
  -
  
### Level 3 Position
  -
  
### Level 4 Position
  - 
</details>

<details>
<summary> :crab: Climber Function </summary>

### Start Position
  - Up/Down Motor: Up Position, No Spin
  - Clamp Solenoid: Open Position
    
### Ready to Climb Position:
  -  Up/Down Motor: Down Position, Spin
  -  Clamp Solenoid: Open Position

### Clamp Position:
  - Up/Down Motor: Down Position, No Spin
  - Clamp Solenoid: Closed Position
    
### Climbing Position:
  - Up/Down Motor: Up Position, Spin
  - Clamp Solenoid: Closed Position
    
</details>
<details>
<summary> :droplet: LEDs Function </summary>
  - Turned On & Connected: Rainbow
  - Coral Intaked: Green
</details>
