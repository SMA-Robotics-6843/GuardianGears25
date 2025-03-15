// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import static frc.robot.Constants.ElevatorConstants.elevatorMotorLeftL4SetpointRotationsPerSecond;
import static frc.robot.Constants.ElevatorConstants.elevatorMotorRightL4SetpointRotationsPerSecond;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorToL4 extends Command {
  private ElevatorSubsystem elevatorSubsystem;

  /** Creates a new ElevatorToL4. */
  public ElevatorToL4(ElevatorSubsystem m_elevatorSubsystem) {
    elevatorSubsystem = m_elevatorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.moveElevatorToSetpoint(elevatorMotorLeftL4SetpointRotationsPerSecond, elevatorMotorRightL4SetpointRotationsPerSecond);
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
