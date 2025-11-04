# GuardianGears25

FRC Team 6843 Guardian Gears' command-based Project for the Reefscape season.

Remember that documentation for most things can be found in the [wpilib docs](https://docs.wpilib.org).

## Subsystems

This year, instead of having a seperate file for every command, I created each command inline in their respective subsystem files.

- CTRE Drivetrain
  - MK4i swerve with Kraken motors and Pigeon 2 gyro
- Elevator
  - Two motors spinning opposite
- End Effector
  - One motor (named Sassy) to change the angle of the arm
  - Another motor (named F Motor) at the end of the arm to intake and output coral
- Climber
  - One motor to wind or unwind the climber

## Drivetrain

- Uses swerve code generated with Phoenix Tuner X with added vision code
- Use applyRequest to send custom commands to the drivetrain. For example:

    ``` java
    // Command to drive backwards at -1.5 speed for a set amount of seconds

    public Command driveBackwards(double seconds) {
        return applyRequest(
            () -> driveRobotCentric.withVelocityX(-1.5)).withTimeout(seconds);
    }
    ```

## Automation

### Pathplanner

Pathplanner documentation can be found [here](https://pathplanner.dev/home.html)

This was our first year using pathplanner, so there is still much to improve on.

- The robot config values need to be set; Sysid needs to be used to calculate the Robot MOI.
- **The autonomous paths need to be tuned on a practice field through trial and error**.
- Pathplanner paths take up a lot of memory on the RoboRIO. Be careful when making several of them.

### Robot Pose, Pathfinding, and Vision

- The **robot pose** is the position of the robot on the field. It can either be 2D or 3D. Without a vision system, the pose is normally calculated using the encoders on the drive wheels. This measurement is far from perfect and gets offset from the actual position very easily. A vision system can be used to fix this by using Apriltags to get a more accurate pose measurement and updating the pose periodically.
  - We did not test the photonvision enough this year so our pathplanning and pathfinding was still off.
  - Photonvision documentation can be found [here](https://docs.photonvision.org/en/latest/)

- **Pathfinding** is pathplanner's function to create a path on the fly to drive to a set pose or a path you have made. The left and right triggers attempt to pathfind to the left and right reef bars respectively. The right side pathfinding works more consistently for some reason.
  - Pathfinding documentation can be found [here](https://pathplanner.dev/pplib-pathfinding.html)
- A better system for alignment to a pose is a [holonomic drive controller](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/holonomic.html), which combines multiple PID controllers and a ProfiledPIDController for the robot's different forms of movement.

### Presets

- Our presets are command groups that chain together commands to automatically score or intake pieces. These commands use PID to move parts of the robot to set positions. Command groups allow you set time between the commands and if you want to run them at the same time or sequentially. Eventually, the goal is to completely phase out manual control and just have buttons for each automated task. This requires good tuning of PID, vision, and the use of sensors to work well.

Documentation:

- [Command Groups](https://docs.wpilib.org/en/2020/docs/software/commandbased/command-groups.html)
- [Command Compositions](https://docs.wpilib.org/en/stable/docs/software/commandbased/command-compositions.html)

### PID

PID is used to set motors to move to set encoder values (setpoints) and hold them there. This is used to make feeding and scoring presets for every level.

I first tried to use the Ziegler Nichols method that was explained in the video below, but after I did the calculations to find the three values it didn't work. I ended up finding that just setting kP to .1 worked fine for L2 so I left it at that. With this tuning, if the elevator is far enough away from its setpoint, it will move to its setpoint, slow down, but not stop. The PID needs to be tuned properly through trial and error to prevent this.

Documentation/info:

- [Introduction](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html)
- [PID Control in WPILib](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html)
- [PID Control in Command-based](https://docs.wpilib.org/en/stable/docs/software/commandbased/pid-subsystems-commands.html#pid-control-in-command-based)
- [PID Control (video)](https://www.youtube.com/watch?v=UOuRx9Ujsog&t=252s)
- [What is a PID Controller? (video)](https://www.youtube.com/watch?v=OqvrYNJvtaU&t=456s)
- [How to Tune a PID Controller - Made Simple! (video)](https://www.youtube.com/watch?v=6EcxGh1fyMw&t=458s)

## Controls

Controls are set in RobotContainer.java

### Driver Controller

- Left Joystick controls forward, backward, side to side, and right stick controls rotation
  - Hold down left stick to enter brake mode, placing the wheels in an X formation
  - Press down right stick to reset field-centric heading
  - Hold left bumper to switch from field centric to robot centric
  - Hold right bumper to drive slowly
  - Both bumpers can be used at the same time
  - Be careful when using the bumpers, you should release everything before changing which bumpers you are pressing
  - Left and right triggers attempt to align to the left or right reef bar where the robot is facing

### Operator Controller

- **dpad up and down**: elevator up and down
- **x**: output coral from end effector
- **b**: intake coral to end effector
- **a**: end effector down
- **y**: end effector up

**While holding left bumper:**

- **a**: Run automation to score coral on L1
- **x**: Run automation to score coral on L2
- **b**: Run automation to score coral on L3
- **y**: Run automation to score coral on L4
- **right bumper**: Run automation to intake from feeding station
- **press right stick**: Reset elevator and end effector encoders to 0

## What to do better next time

- Tune autos on a practice field
- Tune drivetrain alignment system (photonvision + holonomic drive controller)
- Tune PID values
- Install sensors on the robot to allow for smooth transitions between commands in sequential groups (ex. setting a command to end when a sensor does not detect a gamepiece)
