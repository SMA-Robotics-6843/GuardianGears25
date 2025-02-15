// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AprilTagAlign;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.constants.Constants;

public class RobotContainer {
    private final CommandXboxController joystick = new CommandXboxController(0);

    private final AprilTagAlign aprilTagAlign;
    private final SwerveDriveCommand swerveDriveCommand = new SwerveDriveCommand(joystick);
    private final SwerveDriveSubsystem drivetrain = swerveDriveCommand.getDrivetrain();

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(Constants.Swerve.MaxSpeed);

    public RobotContainer() {
        aprilTagAlign = new AprilTagAlign(drivetrain, swerveDriveCommand);
        configureBindings();
    }

    private void configureBindings() {

        // Drivetrain controls
        drivetrain.setDefaultCommand(swerveDriveCommand);

        joystick.x().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
        
        //Drivetrain automation controls
        joystick.rightBumper().toggleOnTrue(aprilTagAlign);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
