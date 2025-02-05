package frc.robot;

import static edu.wpi.first.units.Units.*;
import frc.robot.generated.TunerConstants;

public class Constants {

    public static class Swerve {
        // Physical properties
        public static double kMaxLinearSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static double kMaxAngularSpeed = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    }
    
    //public static final int GOOGLY_EYE_MOTOR = 14;
    
}