// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commandgroups.CoralFromFeeding;
import frc.robot.commandgroups.ReleaseClimber;
import frc.robot.commandgroups.ScoreCoralL1;
import frc.robot.commandgroups.ScoreCoralL2;
import frc.robot.commandgroups.ScoreCoralL3;
import frc.robot.commandgroups.ScoreCoralL4;
import frc.robot.commandgroups.RemoveHighAlgae;
import frc.robot.commandgroups.RemoveLowAlgae;
import frc.robot.constants.TunerConstants;
import static frc.robot.constants.Constants.DrivetrainConstants.*;

import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class RobotContainer {
        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController driverController = new CommandXboxController(0); // Driver controller
        private final CommandXboxController operatorController = new CommandXboxController(1); // Operator controller

        // Subsystems
        public final LEDSubsystem ledSubsystem = new LEDSubsystem();
        public final SwerveDriveSubsystem drivetrain = TunerConstants.createDrivetrain();
        public final ElevatorSubsystem elevator = new ElevatorSubsystem(ledSubsystem);
        public final EndEffectorSubsystem endEffector = new EndEffectorSubsystem();
        public final ClimberSubsystem climber = new ClimberSubsystem(endEffector, ledSubsystem);

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
        public final Command driveForwards = drivetrain.driveForwards(.7);

        // Use sequential command groups
        public final CoralFromFeeding coralFromFeeding = new CoralFromFeeding(elevator, endEffector);
        public final ScoreCoralL1 scoreCoralL1 = new ScoreCoralL1(elevator, endEffector);
        public final ScoreCoralL2 scoreCoralL2 = new ScoreCoralL2(elevator, endEffector);
        public final ScoreCoralL3 scoreCoralL3 = new ScoreCoralL3(elevator, endEffector);
        public final ScoreCoralL4 scoreCoralL4 = new ScoreCoralL4(elevator, endEffector);
        public final RemoveLowAlgae removeLowAlgae = new RemoveLowAlgae(elevator, endEffector);
        public final RemoveHighAlgae removeHighAlgae = new RemoveHighAlgae(elevator, endEffector);
        public final ReleaseClimber releaseClimber = new ReleaseClimber(climber);

        // Paths for pathfinding
        public final Command lineUpWithLeftBranch1 = drivetrain.lineUpWithLeftBranch1Command;
        public final Command lineUpWithLeftBranch2 = drivetrain.lineUpWithLeftBranch2Command;
        public final Command lineUpWithLeftBranch3 = drivetrain.lineUpWithLeftBranch3Command;
        public final Command lineUpWithLeftBranch4 = drivetrain.lineUpWithLeftBranch4Command;
        public final Command lineUpWithLeftBranch5 = drivetrain.lineUpWithLeftBranch5Command;
        public final Command lineUpWithLeftBranch6 = drivetrain.lineUpWithLeftBranch6Command;
        public final Command lineUpWithLeftBranch6Auto = drivetrain.lineUpWithLeftBranch6CommandAuto;
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
                NamedCommands.registerCommand("ScoreCoralL4", scoreCoralL4);
                NamedCommands.registerCommand("ReleaseClimber", releaseClimber);
                NamedCommands.registerCommand("DriveBackwards", driveBackwards);
                NamedCommands.registerCommand("LineupWithLeftBranch6Auto", lineUpWithLeftBranch6Auto);

                configureBindings();

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        public Command resetEncoders() {
                return parallel(
                                runOnce(
                                                () -> {
                                                        elevator.resetElevatorEncoders();
                                                        endEffector.resetSassyMotorEncoder();
                                                }));
        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> driveFieldCentric
                                                // Drive forward with negative Y (forward)
                                                .withVelocityX(-driverController.getLeftY() * MaxSpeed)
                                                // Drive left with negative X (left)
                                                .withVelocityY(-driverController.getLeftX() * MaxSpeed)
                                                // Drive counterclockwise with negative X (left)
                                                .withRotationalRate(
                                                                -driverController.getRightX() * MaxAngularRate)));

                // Driver controls
                driverController.leftStick().whileTrue(drivetrain.applyRequest(() -> brake));
                // driverController.leftBumper().whileTrue(drivetrain.applyRequest(
                // () -> point.withModuleDirection(
                // new Rotation2d(-driverController.getLeftY(),
                // -driverController.getLeftX()))));
                driverController.leftBumper().whileTrue(drivetrain.applyRequest(
                                () -> driveRobotCentric
                                                .withVelocityX(-driverController.getLeftY() * MaxSpeed)
                                                .withVelocityY(-driverController.getLeftX() * MaxSpeed)
                                                .withRotationalRate(-driverController.getRightX() * MaxAngularRate)));

                driverController.rightBumper().whileTrue(drivetrain.applyRequest(
                                () -> driveFieldCentric
                                                .withVelocityX((-driverController.getLeftY() * MaxSpeed) / 5)
                                                .withVelocityY((-driverController.getLeftX() * MaxSpeed) / 5)
                                                .withRotationalRate(-driverController.getRightX() * MaxAngularRate)));

                driverController.rightBumper().and(driverController.leftBumper()).whileTrue(drivetrain.applyRequest(
                                () -> driveRobotCentric
                                                .withVelocityX((-driverController.getLeftY() * MaxSpeed) / 5)
                                                .withVelocityY((-driverController.getLeftX() * MaxSpeed) / 5)
                                                .withRotationalRate(-driverController.getRightX() * MaxAngularRate)));

                // reset the field-centric heading on right stick press
                driverController.rightStick().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                driverController.a()
                                .and(driverController.leftTrigger().negate())
                                .and(driverController.rightTrigger().negate())
                                .whileTrue(windClimber);
                driverController.y()
                                .and(driverController.leftTrigger().negate())
                                .and(driverController.rightTrigger().negate())
                                .whileTrue(unwindClimber);
                // driverController.b()
                // .and(driverController.leftTrigger().negate())
                // .and(driverController.rightTrigger().negate())
                // .onTrue(driveBackwards);

                // TODO: Run sysid
                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                driverController.back().and(driverController.y())
                                .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                driverController.back().and(driverController.x())
                                .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                driverController.start().and(driverController.y())
                                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                driverController.start().and(driverController.x())
                                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // Pathfinding
                // Left side, left trigger
                driverController.povDown()
                                .and(driverController.leftTrigger()).and(driverController.rightTrigger().negate())
                                .whileTrue(lineUpWithLeftBranch1);
                driverController.povLeft()
                                .and(driverController.leftTrigger()).and(driverController.rightTrigger().negate())
                                .whileTrue(lineUpWithLeftBranch2);
                driverController.povUp()
                                .and(driverController.leftTrigger()).and(driverController.rightTrigger().negate())
                                .whileTrue(lineUpWithLeftBranch3);
                driverController.a()
                                .and(driverController.leftTrigger()).and(driverController.rightTrigger().negate())
                                .whileTrue(lineUpWithLeftBranch4);
                driverController.x()
                                .and(driverController.leftTrigger()).and(driverController.rightTrigger().negate())
                                .whileTrue(lineUpWithLeftBranch5);
                driverController.y()
                                .and(driverController.leftTrigger()).and(driverController.rightTrigger().negate())
                                .whileTrue(lineUpWithLeftBranch6);
                driverController.povRight()
                                .and(driverController.leftTrigger()).and(driverController.rightTrigger().negate())
                                .whileTrue(feedLeft);

                // Right side, right trigger
                driverController.povDown()
                                .and(driverController.rightTrigger()).and(driverController.leftTrigger().negate())
                                .whileTrue(lineUpWithLeftBranch1);
                driverController.povRight()
                                .and(driverController.rightTrigger()).and(driverController.leftTrigger().negate())
                                .whileTrue(lineUpWithLeftBranch2);
                driverController.povUp()
                                .and(driverController.rightTrigger()).and(driverController.leftTrigger().negate())
                                .whileTrue(lineUpWithLeftBranch3);
                driverController.a()
                                .and(driverController.rightTrigger()).and(driverController.leftTrigger().negate())
                                .whileTrue(lineUpWithLeftBranch4);
                driverController.b()
                                .and(driverController.rightTrigger()).and(driverController.leftTrigger().negate())
                                .whileTrue(lineUpWithLeftBranch5);
                driverController.y()
                                .and(driverController.rightTrigger()).and(driverController.leftTrigger().negate())
                                .whileTrue(lineUpWithLeftBranch6);
                driverController.x()
                                .and(driverController.rightTrigger()).and(driverController.leftTrigger().negate())
                                .whileTrue(feedRight);

                // Operator controls
                // Normal
                operatorController.y().and(operatorController.leftBumper().negate()).whileTrue(endEffectorUp);
                operatorController.a().and(operatorController.leftBumper().negate()).whileTrue(endEffectorDown);
                operatorController.x().and(operatorController.leftBumper().negate()).whileTrue(fMotorOut);
                operatorController.b().and(operatorController.leftBumper().negate()).whileTrue(fMotorIn);
                operatorController.povUp().and(operatorController.leftBumper().negate()).whileTrue(elevatorUp);
                operatorController.povDown().and(operatorController.leftBumper().negate()).whileTrue(elevatorDown);

                // Left bumper
                // Sequential command groups
                operatorController.rightBumper().and(operatorController.leftBumper()).whileTrue(coralFromFeeding);
                operatorController.a().and(operatorController.leftBumper()).whileTrue(scoreCoralL1);
                operatorController.x().and(operatorController.leftBumper()).whileTrue(scoreCoralL2);
                operatorController.b().and(operatorController.leftBumper()).whileTrue(scoreCoralL3);
                operatorController.y().and(operatorController.leftBumper()).whileTrue(scoreCoralL4);
                operatorController.leftTrigger().and(operatorController.leftBumper()).whileTrue(removeLowAlgae);
                operatorController.rightTrigger().and(operatorController.leftBumper()).whileTrue(removeHighAlgae);

                // Reset elevator and end effector encoders on right stick press
                operatorController.rightStick().onTrue(resetEncoders());

                // Telemetry
                drivetrain.registerTelemetry(logger::telemeterize);
        }

        public SwerveDriveSubsystem getDrivetrain() {
                return drivetrain;
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
                // return driveForwards;
        }
}
