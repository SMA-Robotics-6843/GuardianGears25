// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.automation.ElevatorToFeeding;
import frc.robot.commands.automation.ElevatorToL2;
import frc.robot.commands.automation.ElevatorToL3;
import frc.robot.commands.automation.EndEffectorToL2;
import frc.robot.commands.automation.EndEffectorToL3;
import frc.robot.commands.basic.elevator.ElevatorDown;
import frc.robot.commands.basic.elevator.ElevatorUp;
import frc.robot.commands.basic.endeffector.EndEffectorDown;
import frc.robot.commands.basic.endeffector.EndEffectorUp;
import frc.robot.commands.basic.endeffector.FMotorIn;
import frc.robot.commands.basic.endeffector.FMotorOut;
import frc.robot.commands.groups.ScoreCoralL2;
import frc.robot.commands.groups.ScoreCoralL3;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController controller = new CommandXboxController(0);
   // private final CommandXboxController controller2 = new CommandXboxController(1);
    
    public final SwerveDriveSubsystem drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final EndEffectorSubsystem endEffector = new EndEffectorSubsystem();

    public final ElevatorUp elevatorUp = new ElevatorUp(elevator);
    public final ElevatorDown elevatorDown = new ElevatorDown(elevator);
    public final FMotorIn fMotorIn = new FMotorIn(endEffector);
    public final FMotorOut fMotorOut = new FMotorOut(endEffector);
    public final EndEffectorUp endEffectorUp = new EndEffectorUp(endEffector);
    public final EndEffectorDown endEffectorDown = new EndEffectorDown(endEffector);

    public final ElevatorToL2 elevatorToL2 = new ElevatorToL2(elevator);
    public final ElevatorToL3 elevatorToL3 = new ElevatorToL3(elevator);
    public final ElevatorToFeeding elevatorToFeeding = new ElevatorToFeeding(elevator);
    
    public final EndEffectorToL2 endEffectorToL2 = new EndEffectorToL2(endEffector);
    public final EndEffectorToL3 endEffectorToL3 = new EndEffectorToL3(endEffector);
    
    public final ScoreCoralL2 scoreCoralL2 = new ScoreCoralL2(elevator, endEffector);
    public final ScoreCoralL3 scoreCoralL3 = new ScoreCoralL3(elevator, endEffector);
    
    private final SendableChooser<Command> autoChooser;
    
    public RobotContainer() {
        configureBindings();

        NamedCommands.registerCommand("ScoreCoralL3", scoreCoralL3);
        
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-controller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        controller.leftStick().whileTrue(drivetrain.applyRequest(() -> brake));
        controller.leftBumper().and(controller.leftTrigger()).whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))
        ));

        // TODO: Run sysid
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        controller.rightStick().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Normal
        controller.y().and(controller.leftBumper().negate()).whileTrue(endEffectorUp);
        controller.a().and(controller.leftBumper().negate()).whileTrue(endEffectorDown);
        controller.x().and(controller.leftBumper().negate()).whileTrue(fMotorOut);
        controller.b().and(controller.leftBumper().negate()).whileTrue(fMotorIn);
        controller.rightBumper().and(controller.leftBumper().negate()).whileTrue(elevatorUp);
        controller.rightTrigger().and(controller.leftBumper().negate()).whileTrue(elevatorDown);

        // Left bumper
        controller.x().and(controller.leftBumper()).whileTrue(scoreCoralL2);
        controller.b().and(controller.leftBumper()).whileTrue(scoreCoralL3);
        
        controller.povLeft().and(controller.leftBumper()).whileTrue(elevatorToL2);
        controller.povRight().and(controller.leftBumper()).whileTrue(elevatorToL3);
        controller.rightTrigger().and(controller.leftBumper()).whileTrue(elevatorToFeeding);
    }

    public SwerveDriveSubsystem getDrivetrain() {
        return drivetrain;
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        //return Commands.print("No autonomous command configured");
    }
}
