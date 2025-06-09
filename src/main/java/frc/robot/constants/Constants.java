package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import java.util.Map;

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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class Constants {
    public static final double secondsToReachFeeding = 1;
    public static final double secondsToReachL1 = .7;
    public static final double secondsToReachL2 = .7;
    public static final double secondsToReachL3 = .7;
    public static final double secondsToReachL4 = 1;
    public static final double secondsToReachLowAlgae = .7;
    public static final double secondsToReachHighAlgae = .7;
    public static final double secondsToRunIntakeFeeding = 2;
    public static final double secondsToRunIntakeScoring = 1;
    public static final double secondsToRunIntakeAlgae = 10;
    public static final double secondsToReleaseClimber = 1;

    public static class DrivetrainConstants {
        public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                            // top
                                                                                            // speed
        public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                                // second
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

    public static class VisionConstants {
        public static final String kCameraName = "Arducam";
        // Offsets: 0m forward, 0.3m to the left, 0.41m up
        public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0, 0.3, 0.41),
                // Camera is mounted facing -30 degrees or -.52 radians to the right
                new Rotation3d(0, 0, -.52));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.kDefaultField);

        // The standard deviations of our vision estimated poses, which affect
        // correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static class AutomationConstants {
        public static final double elevatorMotorsFeedingSetpoint = 1;
        public static final double elevatorMotorsL1Setpoint = 1.5;
        public static final double elevatorMotorsL2Setpoint = 3.5;
        public static final double elevatorMotorsL3Setpoint = 24; // 27.6
        public static final double elevatorMotorsL4Setpoint = 77.5;
        public static final double elevatorMotorsLowAlgaeSetpoint = 3;
        public static final double elevatorMotorsHighAlgaeSetpoint = 20;

        public static final double sassyMotorHoldSetpoint = 0;
        public static final double sassyMotorFeedingSetpoint = 4;
        public static final double sassyMotorL1Setpoint = -1;
        public static final double sassyMotorL2Setpoint = 0;
        public static final double sassyMotorL3Setpoint = 0;
        public static final double sassyMotorL4Setpoint = -12.5;
        public static final double sassyMotorRemoveAlgaeSetpoint = -6;
    }

    public static class ElevatorConstants {
        // CAN IDs
        public static final int elevatorMotorLeftID = 2;
        public static final int elevatorMotorRightID = 3;

        // Movement
        public static final double elevatorMotorsUpSpeed = .45;
        public static final double elevatorMotorsDownSpeed = -.45;

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
        public static final double sassyMotorUpSpeed = .1;
        public static final double sassyMotorDownSpeed = -.1;
        public static final double fMotorInSpeed = .25;
        public static final double fMotorOutSpeed = -1;
        public static final double fMotorOutSpeedL1 = -.75;

        // PID tunings
        public static final double sassyMotorkP = 0.1;
        public static final double sassyMotorkI = 0;
        public static final double sassyMotorkD = 0;
    }

    public static class ClimberConstants {
        public static final int climberMotorID = 6;
        public static final double climberMotorSpeed = 1;
    }

    public static class LEDConstants {
        // Our LED strip has a density of 120 LEDs per meter
        private static final Distance kLedSpacing = Meters.of(1 / 120.0);

        private static final LEDPattern gradientNavyDarkGreen = LEDPattern.gradient(
                LEDPattern.GradientType.kContinuous, Color.kNavy, Color.kDarkGreen, Color.kNavy, Color.kDarkGreen);

        // Create a new pattern that scrolls the rainbow pattern across the LED strip,
        // moving at a speed of 1 meter per second.
        public static final LEDPattern scrollingGradientNavyDarkGreen = gradientNavyDarkGreen
                .scrollAtAbsoluteSpeed(MetersPerSecond.of(.5), kLedSpacing);

        // all hues at maximum saturation and half brightness
        private static final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);

        // Create a new pattern that scrolls the rainbow pattern across the LED strip,
        // moving at a speed of 1 meter per second.
        public static final LEDPattern scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1.2), kLedSpacing);
        public static final LEDPattern scrollingRainbowSlow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(.3), kLedSpacing);
        public static final LEDPattern red = LEDPattern.solid(Color.kRed);
        public static final LEDPattern orange = LEDPattern.solid(Color.kOrange);
        public static final LEDPattern yellow = LEDPattern.solid(Color.kYellow);
        public static final LEDPattern green = LEDPattern.solid(Color.kGreen);
        public static final LEDPattern blue = LEDPattern.solid(Color.kBlue);
        public static final LEDPattern purple = LEDPattern.solid(Color.kPurple);
        public static final LEDPattern white = LEDPattern.solid(Color.kWhite);

        Map<Double, Color> maskSteps = Map.of((Double) 0.0, Color.kWhite, (Double) 0.5, Color.kBlack);
        LEDPattern base = white;
        LEDPattern mask =
        LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(0.25));

        LEDPattern scrollingWhite = base.mask(mask);
    }
}
