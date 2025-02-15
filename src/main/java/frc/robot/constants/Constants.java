package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

public class Constants {

    public static class Swerve {
        // Physical properties
        public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    }
    
    //public static final int GOOGLY_EYE_MOTOR = 14;
    
}