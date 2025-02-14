package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Angle;

public final class CoralPivotConstants {
    public static final double kP = 0; // Proportional constant
    public static final double kI = 0; // Integral constant
    public static final double kD = 0; // Derivative constant
    public static final double kG = 0; // Feedforward gravity constant
    public static final double kV = 0; // Feedforward velocity constant
    public static final double kA = 0; // Feedforward acceleration constant

    public static final double supplyCurrentLimit = 70.0; 
    public static final double lowerSupplyCurrentLimit = 10; 



    public static final Angle LOWER_ANGLE = Angle.ofBaseUnits(0, Degrees);
    public static final Angle UPPER_ANGLE = Angle.ofBaseUnits(0, Degrees);
}
