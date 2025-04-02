// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

import static frc.robot.constants.Constants.secondsToReachLowAlgae;
import static frc.robot.constants.Constants.secondsToRunIntakeAlgae;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RemoveLowAlgae extends SequentialCommandGroup {

  /** Creates a new RemoveLowAlgae. */
  public RemoveLowAlgae(ElevatorSubsystem m_elevatorSubsystem, EndEffectorSubsystem m_endEffectorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // Run the elevator to L2 and the end effector to 2L, then run the end
        // effector's motor out
        m_endEffectorSubsystem.endEffectorToRemoveAlgae()
            .alongWith(m_elevatorSubsystem.elevatorToLowAlgae())
            .withTimeout(secondsToReachLowAlgae),

        m_endEffectorSubsystem.fMotorOut().withTimeout(secondsToRunIntakeAlgae));
  }
}
