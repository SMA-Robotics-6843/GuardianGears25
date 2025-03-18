// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCoralL3 extends SequentialCommandGroup {

  ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  EndEffectorSubsystem m_endEffectorSubsystem = new EndEffectorSubsystem();

  /** Creates a new ScoreCoralL3. */
  public ScoreCoralL3(ElevatorSubsystem m_elevatorSubsystem, EndEffectorSubsystem m_endEffectorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(m_elevatorSubsystem.ElevatorToL3().withTimeout(1), m_endEffectorSubsystem.EndEffectorToL3().withTimeout(1));
  }
}
