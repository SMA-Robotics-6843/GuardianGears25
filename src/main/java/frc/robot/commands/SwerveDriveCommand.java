// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveDriveCommand extends Command {
    public final SwerveDriveSubsystem drivetrain = TunerConstants.createDrivetrain();
    private CommandXboxController joystick;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Swerve.MaxSpeed * 0.1).withRotationalDeadband(Constants.Swerve.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  /** Creates a new SwerveDriveCommand. */
  public SwerveDriveCommand(CommandXboxController m_joystick) {
    this.joystick = m_joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("SwerveDriveCommand initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocityX = -joystick.getLeftY() * Constants.Swerve.MaxSpeed;
    double velocityY = -joystick.getLeftX() * Constants.Swerve.MaxSpeed;
    double rotationalRate = -joystick.getRightX() * Constants.Swerve.MaxAngularRate;

    System.out.println("Executing SwerveDriveCommand: velocityX=" + velocityX + ", velocityY=" + velocityY + ", rotationalRate=" + rotationalRate);

    drivetrain.applyRequest(() ->
        drive.withVelocityX(velocityX)
            .withVelocityY(velocityY)
            .withRotationalRate(rotationalRate)
    );

    SmartDashboard.putNumber("joystick left Y", joystick.getLeftY());
    SmartDashboard.putNumber("joystick left X", joystick.getLeftX());
    SmartDashboard.putNumber("joystick right X", joystick.getRightX());
  }

  public SwerveDriveSubsystem getDrivetrain() {
    return drivetrain;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("SwerveDriveCommand ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
