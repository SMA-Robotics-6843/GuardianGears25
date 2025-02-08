// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AprilTagAlign;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.commands.SwerveDriveCommand;

public class RobotContainer {
    private final CommandXboxController joystick = new CommandXboxController(0);

    public final Robot robot;
    public final AprilTagAlign aprilTagAlign;
    public final SwerveDriveCommand swerveDriveCommand = new SwerveDriveCommand();
    public final SwerveDriveSubsystem swerveDriveSubsystem = swerveDriveCommand.getDrivetrain();

    public RobotContainer(Robot m_robot) {
        this.robot = m_robot;
        aprilTagAlign = new AprilTagAlign(swerveDriveSubsystem, robot, swerveDriveCommand);
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        swerveDriveCommand.getDrivetrain().setDefaultCommand(swerveDriveCommand);
        joystick.a().onTrue(aprilTagAlign);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
