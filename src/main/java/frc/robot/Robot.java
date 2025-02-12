// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  
  private PhotonCamera photonCamera;

  private final double VISION_TURN_kP = 0.01;
  private final double VISION_DES_ANGLE_deg = 0.0;
  private final double VISION_STRAFE_kP = 0.5;
  private final double VISION_DES_RANGE_m = 1.25;

  // Calculate drivetrain commands from Joystick values
  public double forward = 10;
  public double turn = 10;


  public Robot() {
    m_robotContainer = new RobotContainer(this);
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
             // Read in relevant data from the Camera
        boolean targetVisible = false;
        double targetYaw = 0.0;
        double targetRange = 0.0;
    var results = photonCamera.getLatestResult();
     if (results.hasTargets()) {
        var target = results.getBestTarget(); // Get the best target
        targetYaw = target.getYaw();
        targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                Units.inchesToMeters(20.5), // Measured with a tape measure, or in CAD.
                Units.inchesToMeters(8.5), 
                Units.degreesToRadians(90), // Measured with a protractor, or in CAD.
                Units.degreesToRadians(target.getPitch()));
        targetVisible = true;
    }

      turn =
              (VISION_DES_ANGLE_deg - targetYaw) * VISION_TURN_kP * Constants.Swerve.MaxAngularRate;
      forward =
              (VISION_DES_RANGE_m - targetRange) * VISION_STRAFE_kP * Constants.Swerve.MaxSpeed;
        
      // Put debug information to the dashboard
      SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
      SmartDashboard.putNumber("Vision Target Range (m)", targetRange); 
  }

  public double getForward() {
    return forward;
  }

  public double getTurn() {
    return turn;
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
