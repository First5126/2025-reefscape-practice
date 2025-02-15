package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Revolutions;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ElevatorConstants {
    public static enum CoralLevels {
        L1(0,1),
        L2(0.5,1),
        L3(1,1),
        L4(1.5,1);

        // Height of the elevator expressed in Revolutions.
        public final Angle heightAngle;
        // Distance for starting the raise of the elevator.
        public final Distance distance;
        private CoralLevels(double height, double distance) {
            this.heightAngle = Revolutions.of(height); 
            this.distance = Meters.of(distance);
        }   
    }
    public static final int ELEVATOR_MOTOR_PORT = 1;
    public static final double ELEVATOR_SPEED = 0.75;
    public static final double ELEVATOR_MAX_HEIGHT = 2.0; // in meters
    public static final double ELEVATOR_MIN_HEIGHT = 0.0; // in meters
    public static final double ELEVATOR_TOLERANCE = 0.01; // in meters  
    
    public static final double kS = 0.25;
    public static final double kV = 0.12;
    public static final double kA = 0.01;
    public static final double kP = 4.8;
    public static final double kI = 0;
    public static final double kD = 0.1;

    public static final int FORWARD_DIGITAL_LIMIT = 0;
    public static final int REVERSE_DIGITAL_LIMIT = 0;

    public static final double GEAR_RATIO = 12.0;

    public static final double MotionMagicCruiseVelocity = 0.25; // Target cruise velocity of 80 rps
    public static final double MotionMagicAcceleration = 1; // Target acceleration of 160 rps/s (0.5 seconds)
    public static final double MotionMagicJerk = 1; // Target jerk of 1600 rps/s/s (0.1 seconds)
    public static final double MotionMagicMaxVelocity = 0.25; // Target max velocity of 80 rps
}