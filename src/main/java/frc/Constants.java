package frc;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {
    // REFACTOR
    public static final double XBOX_CONTROLLER_DEADBAND = 0.07;
    public static final LinearVelocity SWERVE_MAXIMUM_SPEED = MetersPerSecond.of(8.0);
    public static final double SWERVE_MAXIMUM_ANGULAR_VELOCITY = Units.degreesToRadians(720*2);
    // END REFACTOR
    public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(8.0);
    public static final Double MAX_ANGULAR_VELOCITY = Units.degreesToRadians(720*2);
    public static final double MAX_SPEED2 = Units.feetToMeters(14.5);
    public static final double DRIVER_DEADBAND = 0.07;

    public static final int CAMERA_QUALITY_FACTOR = 4;
    public static final int INTAKE_ROLLER_MOTOR_ID = 16;
    public static final int INTAKE_ROTATION_MOTOR_ID = 15;
    public static final int Roller_In_STORAGE= 17;
    public static final int Shooter_MOTOR_ID = 18;
    
    public static final int VERTICAL_ACTUATOR_PWM_CHANNEL = 1;
}