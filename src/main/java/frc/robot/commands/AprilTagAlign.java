// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AprilTagAlign extends Command {

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private final double VISION_TURN_kP = 0.01;
  private final double VISION_DES_ANGLE_deg = 0.0;
  private final double VISION_STRAFE_kP = 0.5;
  private final double VISION_DES_RANGE_m = 1.25;

  double forward = 0;
  double turn = 0;

  static boolean targetVisible = false;
  static double targetRange = 0;

  private PhotonCamera photonCamera;

  private final SwerveDriveSubsystem drivetrain;
  private final SwerveDriveCommand swerveDriveCommand;
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  /** Creates a new AprilTagAlign. */
  public AprilTagAlign(SwerveDriveSubsystem m_drivetrain, SwerveDriveCommand m_swerveDriveCommand) {
    this.drivetrain = m_drivetrain;
    this.swerveDriveCommand = m_swerveDriveCommand;
    photonCamera = new PhotonCamera("Arducam");
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetYaw = 0.0;
    // Read in relevant data from the Camera
    var results = photonCamera.getLatestResult();
     if (results.hasTargets()) {
        var target = results.getBestTarget(); // Get the best target
        targetYaw = target.getYaw();
        targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                Units.inchesToMeters(20.5), // Measured with a tape measure, or in CAD.
                Units.inchesToMeters(8.5),
                Units.degreesToRadians(90), // Measured with a protractor, or in CAD.
                Units.degreesToRadians(target.getPitch()));
        targetVisible = true;
    }

    turn =
            (VISION_DES_ANGLE_deg - targetYaw) * VISION_TURN_kP * Constants.Swerve.MaxAngularRate;
    forward =
            (VISION_DES_RANGE_m - targetRange) * VISION_STRAFE_kP * Constants.Swerve.MaxSpeed;

    System.out.println("AprilTagAlign execute called");
    SmartDashboard.putNumber("forward", forward);
    SmartDashboard.putNumber("turn", turn);

    swerveDriveCommand.getDrivetrain().applyRequest(() ->
    drive.withVelocityX(forward)
        .withRotationalRate(turn)
    );
  }

  public static boolean getTargetVisible() {
    return targetVisible;
  }

  public static double getTargetRange() {
    return targetRange;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
