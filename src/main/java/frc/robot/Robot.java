// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveDriveSubsystem;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.wpilibj.DataLogManager;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private SwerveDriveSubsystem drivetrain;
  private Vision vision;
  private Timer m_gcTimer = new Timer();

  public Robot() {
    m_robotContainer = new RobotContainer();
    drivetrain = m_robotContainer.getDrivetrain();
    vision = new Vision();
    m_gcTimer.start();
    CameraServer.startAutomaticCapture("Climber Cam", 0);
    CameraServer.startAutomaticCapture("Intake Cam", 1);
    PathfindingCommand.warmupCommand().schedule();
    // DataLogManager.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // Run garbage collector every 5 seconds
    if(m_gcTimer.advanceIfElapsed(5)) {
      System.gc();
    }
    var visionEst = vision.getEstimatedGlobalPose();
    SmartDashboard.putBoolean("visionEst.isPresent", visionEst.isPresent());
    visionEst.ifPresent(
        est -> {
          System.out.println("visionEst present");
          // Change our trust in the measurement based on the tags we can see
          var estStdDevs = vision.getEstimationStdDevs();

          drivetrain.addVisionMeasurement(
              est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        });
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
