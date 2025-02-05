// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AprilTagAlign extends Command {
  SwerveDriveSubsystem swerveDriveSubsystem;
  private final Robot robot;
  private final RobotContainer m_robotContainer = new RobotContainer();
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

  /** Creates a new AprilTagAlign. */
  public AprilTagAlign(SwerveDriveSubsystem m_swerveDriveSubsystem, Robot m_robot) {
    this.swerveDriveSubsystem = m_swerveDriveSubsystem;
    this.robot = m_robot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_robotContainer.getDrivetrain().applyRequest(() ->
    drive.withVelocityX(robot.forward) // Drive forward with negative Y (forward)
        .withRotationalRate(robot.turn) // Drive counterclockwise with negative X (left)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
