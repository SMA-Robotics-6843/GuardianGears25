# GuardianGears25

FRC Team 6843 Guardian Gears' command-based Project for the Reefscape season.

Remember that documentation for most things can be found at https://docs.wpilib.org or by searching "(topic) documentation."

## Subsystems

- CTRE Drivetrain
    - MK4i motors with Pigeon 2 gyro
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

    ```
    // Command to drive backwards at -1.5 speed for a set amount of seconds

    public Command driveBackwards(double seconds) {
        return applyRequest(
            () -> driveRobotCentric.withVelocityX(-1.5)).withTimeout(seconds);
    }
    ```

## Automation

### Pathplanner
Pathplanner documentation can be found at https://pathplanner.dev/home.html

This was our first year using pathplanner, so there is still much to improved on. 
- The robot config values need to be set; Sysid needs to be used to calculate the Robot MOI. 
- The autonomous paths need to be tuned on a practice field through trial and error. 

### Robot Pose, Pathfinding, and Vision


- The **robot pose** is the position of the robot on the field. It can either be 2D or 3D. Without a vision system, the pose is normally calculated using the encoders on the drive wheels. This measurement is far from perfect and gets offset from the actual position very easily. A vision system can be used to fix this by using Apriltags to get a more accurate pose measurement and updating the pose periodically.
    - We did not test the photonvision enough this year so our pathplanning and pathfinding was still off.
    - Photonvision documentation can be at https://docs.photonvision.org/en/latest/

- **Pathfinding** is pathplanner's function to create a path on the fly to drive to a set pose or a path you have made. I set up 12 different pathfinding commands to line up with each reef branch, and 2 more to line up with feeding stations, but these were not tested. The pose of the robot drifts too often and Photonvision was not tuned enough to compensate for it.
    - Pathfinding documentation can be found at https://pathplanner.dev/pplib-pathfinding.html

### PID

PID is used to set motors to move to set encoder values and hold them there. This is used to make feeding and scoring presets for every level.

I first tried to use the Ziegler Nichols method that was explained in the video below, but after I did the calculations to find the three values it didn't work. I ended up finding that just setting kP to .1 worked fine so I left it at that. This later led to a problem with the L3 preset where the elevator would go past the correct setpoint and hit the top.

Documentation:
- [Introduction](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html)
- [PID Control in WPILib](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html)
- [PID Control in Command-based](https://docs.wpilib.org/en/stable/docs/software/commandbased/pid-subsystems-commands.html#pid-control-in-command-based)
- [Explanation video](https://www.youtube.com/watch?v=UOuRx9Ujsog&t=252s)


## Controls

Controls are set in RobotContainer.java

### Driver Controller

- Left Joystick controls forward, backward, side to side, and right stick controls rotation
    - Hold left bumper to switch from field centric to robot centric
    - Hold right bumper to drive slowly
    - Both bumpers can be used at the same time
    - Be careful when using the bumpers, you should release everything before changing which bumpers you are pressing

Pathfinding controls:
![Drawing 04-02-2025 20 21 excalidraw](https://github.com/user-attachments/assets/e61c9e23-a830-4842-b46f-cbc1d6986c3c)
