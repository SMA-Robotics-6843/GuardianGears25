package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Constants {
    public static final double secondsToReachFeeding = 1;
    public static final double secondsToReachL1 = 1;
    public static final double secondsToReachL2 = 1;
    public static final double secondsToReachL3 = 1;
    public static final double secondsToRunIntakeFeeding = 3.5;
    public static final double secondsToRunIntakeScoring = 1;
    public static final double secondsToReleaseClimber = 1;

    public static class DrivetrainConstants {
        public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                            // speed
        public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                                          // max angular velocity
        public static final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
                .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

        public static final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
                .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

        public static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        public static final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    }

    public static class Vision {
        public static final String kCameraName = "Arducam";
        // Offsets: 0.05m forward, 0.25m to the left, 0.86m up
        public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.05, -0.25, 0.86),
                // Camera is mounted facing forward
                new Rotation3d(0, 0, 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.kDefaultField);

        // The standard deviations of our vision estimated poses, which affect
        // correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static class ElevatorConstants {
        // CAN IDs
        public static final int elevatorMotorLeftID = 2;
        public static final int elevatorMotorRightID = 3;

        // Movement
        public static final double elevatorMotorsUpSpeed = .25;
        public static final double elevatorMotorsDownSpeed = -.25;

        public static final double elevatorMotorsFeedingSetpointRotationsPerSecond = 6;
        public static final double elevatorMotorsL1SetpointRotationsPerSecond = 42.5;
        public static final double elevatorMotorsL2SetpointRotationsPerSecond = 37.5;
        public static final double elevatorMotorsL3SetpointRotationsPerSecond = 62;

        // PID tunings
        public static final double elevatorMotorLeftkP = 0.1;
        public static final double elevatorMotorLeftkI = 0;
        public static final double elevatorMotorLeftkD = 0;

        public static final double elevatorMotorRightkP = 0.1;
        public static final double elevatorMotorRightkI = 0;
        public static final double elevatorMotorRightkD = 0;
    }

    public static class EndEffectorConstants {
        // CAN IDs
        public static final int fMotorID = 4;
        public static final int sassyMotorID = 5;

        // Movement
        public static final double sassyMotorUpSpeed = -.25;
        public static final double sassyMotorDownSpeed = .25;
        public static final double fMotorInSpeed = .25;
        public static final double fMotorOutSpeed = -1;
        public static final double fMotorOutSpeedL1 = -.75;

        public static final double sassyMotorFeedingSetpointRotationsPerSecond = 4;
        public static final double sassyMotorL1SetpointRotationsPerSecond = 17;
        public static final double sassyMotorL2SetpointRotationsPerSecond = 13;
        public static final double sassyMotorL3SetpointRotationsPerSecond = 13;
        public static final double sassyMotorHoldSetpointRotationsPerSecond = 6;

        // PID tunings
        public static final double sassyMotorkP = 0.1;
        public static final double sassyMotorkI = 0;
        public static final double sassyMotorkD = 0;
    }

    public static class ClimberConstants {
        public static final int climberMotorID = 6;
        public static final double climberMotorSpeed = 1;
    }
}
