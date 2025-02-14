package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Revolutions;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ElevatorConstants {
    public static enum CoralLevels {
        L1(2,1),
        L2(4,1), 
        L3(6,1),
        L4(8,1);

        // Height of the elevator expressed in Revolutions.
        public final Angle heightAngle;
        // Distance for starting the raise of the elevator.
        public final Distance distance;
        private CoralLevels(double height, double distance) {
            this.heightAngle = Revolutions.of(height); 
            this.distance = Meters.of(distance);
        }   
    }
    // TODO: These Constants are currently not true. They are placeholders. We need to find the correct values.
    public static final int ELEVATOR_MOTOR_PORT = 1;
    public static final double ELEVATOR_SPEED = 0.75;
    public static final double ELEVATOR_MAX_HEIGHT = 2.0; // in Revolutions
    public static final double ELEVATOR_MIN_HEIGHT = 0.0; // in Revolutions
    public static final double ELEVATOR_TOLERANCE = 0.01; // in Revolutions   
}
