package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {

    public static class Swerve {
        // Physical properties
        public static final double kTrackWidth = Units.inchesToMeters(23);
        public static final double kTrackLength = Units.inchesToMeters(25);
        public static final double kRobotWidth = Units.inchesToMeters(31);
        public static final double kRobotLength = Units.inchesToMeters(26);
        public static final double kMaxLinearSpeed = Units.feetToMeters(15.5);
        public static final double kMaxAngularSpeed = Units.rotationsToRadians(2);
        public static final double kWheelDiameter = Units.inchesToMeters(4);
        public static final double kWheelCircumference = kWheelDiameter * Math.PI;

        public static final double kDriveGearRatio = 8.16; // 6.75:1 SDS MK4 L2 ratio
        public static final double kSteerGearRatio = 15.42857142857143; // 12.8:1

        public static final double kDriveDistPerPulse = kWheelCircumference / 1024 / kDriveGearRatio;
        public static final double kSteerRadPerPulse = 2 * Math.PI / 1024;
    }
    
    public static final int GOOGLY_EYE_MOTOR = 14;
    
}