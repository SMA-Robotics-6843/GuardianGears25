// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.automation.ElevatorToL2;
import frc.robot.commands.automation.EndEffectorToL2;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCoralL2 extends ParallelCommandGroup {

  /** Creates a new ScoreCoralL2. */
  public ScoreCoralL2(ElevatorSubsystem m_elevatorSubsystem, EndEffectorSubsystem m_endEffectorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ElevatorToL2(m_elevatorSubsystem), new EndEffectorToL2(m_endEffectorSubsystem));
  }
}
