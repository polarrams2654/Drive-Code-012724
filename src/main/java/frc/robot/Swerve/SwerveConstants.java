package frc.robot.Swerve;

public class SwerveConstants {
    public static final int NEO_ROUNDS_PER_MINUTE = 5676;
    
    public static final double STEER_MOTOR_P = 1.0;
    public static final double STEER_MOTOR_I = 0.0;
    public static final double STEER_MOTOR_D = 0.1;

        public static final double Wheel_Diameter_Meters =0.10033;
        public static final double Drive_Reduction =(14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        public static final boolean Drive_Inverted =true;
        public static final double Steer_Reduction =(14.0 / 50.0) * (10.0 / 60.0);
        public static final boolean Steer_inverted =false;
}
