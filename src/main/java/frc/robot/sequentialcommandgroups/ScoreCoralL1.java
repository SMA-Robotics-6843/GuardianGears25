// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sequentialcommandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import static frc.robot.constants.Constants.secondsToReachL1;
import static frc.robot.constants.Constants.secondsToRunIntakeScoring;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCoralL1 extends SequentialCommandGroup {

  /** Creates a new ScoreCoralL1. */
  public ScoreCoralL1(ElevatorSubsystem m_elevatorSubsystem, EndEffectorSubsystem m_endEffectorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Run the elevator to L1 and the end effector to L1, then run the end effector's motor out
        m_elevatorSubsystem.elevatorToL1()
            .alongWith(
                m_endEffectorSubsystem.endEffectorToL1())
            .withTimeout(secondsToReachL1),
        m_endEffectorSubsystem.fMotorOut().withTimeout(secondsToRunIntakeScoring));
  }
}
