# Robot breakdown
This is a FRC robot built for the 2025-2026 FTC game DECODE.

The robot is 16 by 16 inches, and able to hold onto 2 artifacts.

## Subsystems
### Intake
- Intake wheels are 2in. diameter
- 2 NEO 550 for intaking (1 for the contact wheel, 1 for the belt system)
- 3 limit switches at each possible position in the system

### Shooter
- 1 NEO mounted to with 84T to 36T reduction (7:3)
- Shooter wheel is 2.965 in. diameter
- There may be a hood for the system to adjust shooting angle
- if hood is present, Potentiometer/absolute encoder to determine hood angle
- 1 neo 550s (maybe) to andjust hood angle 

### drivetrain (Swerve)
- 4 module Stock MaxSwerve
- Bot is 16 in by 16 in
- NavX2 is centred on base

Location from the centre With NWU coordinate system is as follows:

| Module CANID (drive, turn) | X distance from centre (in) | Y distance from centre (in) | Rotational offset |
| :----------------- | :-------: | :-------: | :--------: |
| Front Left (1,2)   | 4.286576  | 5.245126  | 1/4 $\pi   |      
| Bottom Left (3,4)  | -6.250000 | 6.250000  | -1/2 $\pi  |
| Bottom Right (5,6) | -6.250000 | -6.250000 |     0      |
| Front Right (7,8)  | 4.286576  | -5.245126 | -1/4 $\pi  |

### Tracking
- Vision using limelight 3 camera (April tags)
- Field tracking using using Field2D in smartdashboard


## MVP Requirements
1. The robot drivetrain is capable of swerve movements (field and robot) [drivetrain subsystem]

2. The robot is capable of intaking and holding onto 3 artifacts [intake subsystem]

3. The robot is capable of shooting in the close and far launch zone [shooter subsystem]
    1. The robot is able to set and maintain hood angle
    2. The robot is able to set and maintain shooter velocity

4. The robot is capable of doing a 2 artifact auto [autonomous]

5. The robot is capable of getting pose from the apriltag [tracking subsystem]
    1. The robot is able to determine hood angle and shooter velocity based on the pose provided by the vision system [shooter + tracking subsystems]


6. The robot is capable of sorting artifacts by color [intake subsystem]
