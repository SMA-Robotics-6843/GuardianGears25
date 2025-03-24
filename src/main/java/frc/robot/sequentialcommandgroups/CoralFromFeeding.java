// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sequentialcommandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import static frc.robot.constants.Constants.secondsToReachFeeding;
import static frc.robot.constants.Constants.secondsToRunIntakeFeeding;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralFromFeeding extends SequentialCommandGroup {

  /** Creates a new CoralFromFeeding. */
  public CoralFromFeeding(ElevatorSubsystem m_elevatorSubsystem, EndEffectorSubsystem m_endEffectorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // Run the elevator to feeding and the end effector to feeding, then run the end effector's motor in
        m_elevatorSubsystem.ElevatorToFeeding()
            .alongWith(
                m_endEffectorSubsystem.EndEffectorToFeeding())
            .withTimeout(secondsToReachFeeding),

        m_endEffectorSubsystem.FMotorIn().withTimeout(secondsToRunIntakeFeeding));
  }
}
