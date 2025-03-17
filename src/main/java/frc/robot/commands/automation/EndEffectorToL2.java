// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;
import static frc.robot.Constants.EndEffectorConstants.sassyMotorL2SetpointRotationsPerSecond;
import static frc.robot.Constants.EndEffectorConstants.fMotorOutSpeed;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorToL2 extends Command {
  private EndEffectorSubsystem endEffectorSubsystem;
  
  /** Creates a new EndEffectorToL2. */
  public EndEffectorToL2(EndEffectorSubsystem m_endEffectorSubsystem) {
    endEffectorSubsystem = m_endEffectorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_endEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    endEffectorSubsystem.moveEndEffectorToSetpoint(sassyMotorL2SetpointRotationsPerSecond);
    endEffectorSubsystem.spinFMotorAtSetpoint(fMotorOutSpeed);
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
