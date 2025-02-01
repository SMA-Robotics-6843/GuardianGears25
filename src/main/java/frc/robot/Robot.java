// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private PhotonCamera photonCamera;

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private final CommandXboxController joystick = new CommandXboxController(0);
  private final RobotContainer m_robotContainer;
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  
  private final double VISION_TURN_kP = 0.01;
  private final double VISION_DES_ANGLE_deg = 0.0;
  private final double VISION_STRAFE_kP = 0.5;
  private final double VISION_DES_RANGE_m = 1.25;

  // Calculate drivetrain commands from Joystick values
  double forward = -joystick.getLeftY() * MaxSpeed;
  double strafe = -joystick.getLeftX() * MaxSpeed;
  double turn = -joystick.getRightX() * MaxAngularRate;

  public Robot() {
    m_robotContainer = new RobotContainer();
    photonCamera = new PhotonCamera("Arducam");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
        SwerveDriveSubsystem drivetrain = m_robotContainer.getDrivetrain();
       // HttpCamera m_Arducam = new HttpCamera("PhotonVisionCamera","http://photonvision.local:5800");
       // CameraServer.startAutomaticCapture(m_Arducam);
        // Read in relevant data from the Camera
        boolean targetVisible = false;
        double targetYaw = 0.0;
        double targetRange = 0.0;
    var results = photonCamera.getLatestResult();
   /*  if (results.hasTargets()) {
        var target = results.getBestTarget(); // Get the best target
        targetYaw = target.getYaw();
        targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                Units.inchesToMeters(20.5), // Measured with a tape measure, or in CAD.
                Units.inchesToMeters(8.5), 
                Units.degreesToRadians(90), // Measured with a protractor, or in CAD.
                Units.degreesToRadians(target.getPitch()));
        targetVisible = true;
    }

        // Auto-align when requested
        if (joystick.a().getAsBoolean() && targetVisible) {
            // Driver wants auto-alignment to tag 7
            // And, tag 7 is in sight, so we can turn toward it.
            // Override the driver's turn and fwd/rev command with an automatic one
            // That turns toward the tag, and gets the range right.
            turn =
                    (VISION_DES_ANGLE_deg - targetYaw) * VISION_TURN_kP * Constants.Swerve.kMaxAngularSpeed;
            forward =
                    (VISION_DES_RANGE_m - targetRange) * VISION_STRAFE_kP * Constants.Swerve.kMaxLinearSpeed;
        } */

        // Command drivetrain motors based on target speeds
        drivetrain.applyRequest(() ->
        drive.withVelocityX(forward) // Drive forward with negative Y (forward)
            .withVelocityY(strafe) // Drive left with negative X (left)
            .withRotationalRate(turn) // Drive counterclockwise with negative X (left)
        );

        // Put debug information to the dashboard
        SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
        SmartDashboard.putNumber("Vision Target Range (m)", targetRange);
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
