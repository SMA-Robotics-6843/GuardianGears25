// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.TunerConstants;
import static frc.robot.constants.Constants.DrivetrainConstants.*;
import frc.robot.sequentialcommandgroups.CoralFromFeeding;
import frc.robot.sequentialcommandgroups.ReleaseClimber;
import frc.robot.sequentialcommandgroups.ScoreCoralL1;
import frc.robot.sequentialcommandgroups.ScoreCoralL2;
import frc.robot.sequentialcommandgroups.ScoreCoralL3;

import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class RobotContainer {
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController controller1 = new CommandXboxController(0); // Driver controller
    private final CommandXboxController controller2 = new CommandXboxController(1); // Operator controller

    // Subsystems
    public final SwerveDriveSubsystem drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final EndEffectorSubsystem endEffector = new EndEffectorSubsystem();
    public final ClimberSubsystem climber = new ClimberSubsystem(endEffector);

    // Commands
    public final Command elevatorUp = elevator.elevatorUp();
    public final Command elevatorDown = elevator.elevatorDown();
    public final Command fMotorIn = endEffector.fMotorIn();
    public final Command fMotorOut = endEffector.fMotorOut();
    public final Command endEffectorUp = endEffector.endEffectorUp();
    public final Command endEffectorDown = endEffector.endEffectorDown();
    public final Command windClimber = climber.windClimber();
    public final Command unwindClimber = climber.unwindClimber();
    public final Command driveBackwards = drivetrain.driveBackwards(.5);

    // Use sequential command groups
    public final CoralFromFeeding coralFromFeeding = new CoralFromFeeding(elevator, endEffector);
    public final ScoreCoralL1 scoreCoralL1 = new ScoreCoralL1(elevator, endEffector);
    public final ScoreCoralL2 scoreCoralL2 = new ScoreCoralL2(elevator, endEffector);
    public final ScoreCoralL3 scoreCoralL3 = new ScoreCoralL3(elevator, endEffector);
    public final ReleaseClimber releaseClimber = new ReleaseClimber(climber);

    // Paths for pathfinding
    public final Command lineUpWithLeftBranch1 = drivetrain.lineUpWithLeftBranch1Command;
    public final Command lineUpWithLeftBranch2 = drivetrain.lineUpWithLeftBranch2Command;
    public final Command lineUpWithLeftBranch3 = drivetrain.lineUpWithLeftBranch3Command;
    public final Command lineUpWithLeftBranch4 = drivetrain.lineUpWithLeftBranch4Command;
    public final Command lineUpWithLeftBranch5 = drivetrain.lineUpWithLeftBranch5Command;
    public final Command lineUpWithLeftBranch6 = drivetrain.lineUpWithLeftBranch6Command;
    public final Command lineUpWithRightBranch1 = drivetrain.lineUpWithRightBranch1Command;
    public final Command lineUpWithRightBranch2 = drivetrain.lineUpWithRightBranch2Command;
    public final Command lineUpWithRightBranch3 = drivetrain.lineUpWithRightBranch3Command;
    public final Command lineUpWithRightBranch4 = drivetrain.lineUpWithRightBranch4Command;
    public final Command lineUpWithRightBranch5 = drivetrain.lineUpWithRightBranch5Command;
    public final Command lineUpWithRightBranch6 = drivetrain.lineUpWithRightBranch6Command;
    public final Command feedLeft = drivetrain.feedLeftCommand;
    public final Command feedRight = drivetrain.feedRightCommand;

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("CoralFromFeeding", coralFromFeeding);
        NamedCommands.registerCommand("ScoreCoralL2", scoreCoralL2);
        NamedCommands.registerCommand("ScoreCoralL3", scoreCoralL3);
        NamedCommands.registerCommand("ReleaseClimber", releaseClimber);
        NamedCommands.registerCommand("DriveBackwards", driveBackwards);

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> driveFieldCentric.withVelocityX(-controller1.getLeftY() * MaxSpeed) // Drive forward
                                                                                                      // with negative Y
                                                                                                      // (forward)
                        .withVelocityY(-controller1.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-controller1.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                       // negative X (left)
                ));

        // Controller 1 (driver)
        controller1.leftStick().whileTrue(drivetrain.applyRequest(() -> brake));
        controller1.leftBumper().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-controller1.getLeftY(), -controller1.getLeftX()))));

        // reset the field-centric heading on right stick press
        controller1.rightStick().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        controller1.a()
                .and(controller1.leftTrigger().negate())
                .and(controller1.rightTrigger().negate())
                .whileTrue(windClimber);
        controller1.y()
                .and(controller1.leftTrigger().negate())
                .and(controller1.rightTrigger().negate())
                .whileTrue(unwindClimber);
        controller1.b()
                .and(controller1.leftTrigger().negate())
                .and(controller1.rightTrigger().negate())
                .onTrue(driveBackwards);

        // TODO: Run sysid
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        controller1.back().and(controller1.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller1.back().and(controller1.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller1.start().and(controller1.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller1.start().and(controller1.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Pathfinding
        // Left side, left trigger
        controller1.povDown()
                .and(controller1.leftTrigger()).and(controller1.rightTrigger().negate())
                .whileTrue(lineUpWithLeftBranch1);
        controller1.povLeft()
                .and(controller1.leftTrigger()).and(controller1.rightTrigger().negate())
                .whileTrue(lineUpWithLeftBranch2);
        controller1.povUp()
                .and(controller1.leftTrigger()).and(controller1.rightTrigger().negate())
                .whileTrue(lineUpWithLeftBranch3);
        controller1.a()
                .and(controller1.leftTrigger()).and(controller1.rightTrigger().negate())
                .whileTrue(lineUpWithLeftBranch4);
        controller1.x()
                .and(controller1.leftTrigger()).and(controller1.rightTrigger().negate())
                .whileTrue(lineUpWithLeftBranch5);
        controller1.y()
                .and(controller1.leftTrigger()).and(controller1.rightTrigger().negate())
                .whileTrue(lineUpWithLeftBranch6);
        controller1.povRight()
                .and(controller1.leftTrigger()).and(controller1.rightTrigger().negate())
                .whileTrue(feedLeft);

        // Right side, right trigger
        controller1.povDown()
                .and(controller1.rightTrigger()).and(controller1.leftTrigger().negate())
                .whileTrue(lineUpWithLeftBranch1);
        controller1.povRight()
                .and(controller1.rightTrigger()).and(controller1.leftTrigger().negate())
                .whileTrue(lineUpWithLeftBranch2);
        controller1.povUp()
                .and(controller1.rightTrigger()).and(controller1.leftTrigger().negate())
                .whileTrue(lineUpWithLeftBranch3);
        controller1.a()
                .and(controller1.rightTrigger()).and(controller1.leftTrigger().negate())
                .whileTrue(lineUpWithLeftBranch4);
        controller1.b()
                .and(controller1.rightTrigger()).and(controller1.leftTrigger().negate())
                .whileTrue(lineUpWithLeftBranch5);
        controller1.y()
                .and(controller1.rightTrigger()).and(controller1.leftTrigger().negate())
                .whileTrue(lineUpWithLeftBranch6);
        controller1.x()
                .and(controller1.rightTrigger()).and(controller1.leftTrigger().negate())
                .whileTrue(feedRight);

        // Controller 2 (operator)
        // Normal
        controller2.y().and(controller2.leftBumper().negate()).whileTrue(endEffectorUp);
        controller2.a().and(controller2.leftBumper().negate()).whileTrue(endEffectorDown);
        controller2.x().and(controller2.leftBumper().negate()).whileTrue(fMotorOut);
        controller2.b().and(controller2.leftBumper().negate()).whileTrue(fMotorIn);
        controller2.rightBumper().and(controller2.leftBumper().negate()).whileTrue(elevatorUp);
        controller2.rightTrigger().and(controller2.leftBumper().negate()).whileTrue(elevatorDown);

        // Left bumper
        // Sequential command groups
        controller2.y().and(controller2.leftBumper()).whileTrue(coralFromFeeding);
        controller2.a().and(controller2.leftBumper()).whileTrue(scoreCoralL1);
        controller2.x().and(controller2.leftBumper()).whileTrue(scoreCoralL2);
        controller2.b().and(controller2.leftBumper()).whileTrue(scoreCoralL3);

        // Telemetry
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public SwerveDriveSubsystem getDrivetrain() {
        return drivetrain;
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
