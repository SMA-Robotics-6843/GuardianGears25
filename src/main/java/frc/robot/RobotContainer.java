// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.commands.GooglyEyeCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.GooglyEyeSubsystem;


public class RobotContainer {
    private final CommandXboxController joystick = new CommandXboxController(0);
   // private final CommandXboxController joystick2 = new CommandXboxController(1);

    public final GooglyEyeSubsystem GooglyEyeSubsystem = new GooglyEyeSubsystem();
    public final GooglyEyeCommand googlyEyeCommand = new GooglyEyeCommand(GooglyEyeSubsystem);
    public final SwerveDriveSubsystem drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, joystick));
       // joystick2.y().toggleOnTrue(googlyEyeCommand);
    }

    public SwerveDriveSubsystem getDrivetrain() {
        return drivetrain;
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
