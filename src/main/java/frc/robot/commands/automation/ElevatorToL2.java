// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import static frc.robot.Constants.ElevatorConstants.elevatorMotorLeftL2SetpointRotationsPerSecond;
import static frc.robot.Constants.ElevatorConstants.elevatorMotorRightL2SetpointRotationsPerSecond;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorToL2 extends Command {
  private ElevatorSubsystem elevatorSubsystem;


  /** Creates a new ElevatorToL2. */
  public ElevatorToL2(ElevatorSubsystem m_elevatorSubsystem) {
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
    elevatorSubsystem.moveElevatorToSetpoint(elevatorMotorLeftL2SetpointRotationsPerSecond, elevatorMotorRightL2SetpointRotationsPerSecond);
    SmartDashboard.putBoolean("isElevatorAtSetpoint", elevatorSubsystem.getIsElevatorAtSetPoint());
    if(elevatorSubsystem.getIsElevatorAtSetPoint()) {
      this.end(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ElevatorToL2 ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
