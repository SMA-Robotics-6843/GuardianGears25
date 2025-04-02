// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import static frc.robot.constants.Constants.secondsToReleaseClimber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReleaseClimber extends SequentialCommandGroup {
  /** Creates a new ReleaseClimber. */
  public ReleaseClimber(ClimberSubsystem m_ClimberSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(m_ClimberSubsystem.unwindClimber().withTimeout(secondsToReleaseClimber));
  }
}
