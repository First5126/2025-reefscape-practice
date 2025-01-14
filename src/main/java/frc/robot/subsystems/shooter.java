package frc.robot.subsystems;

import java.util.DoubleSummaryStatistics;
import java.util.Set;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooter extends SubsystemBase{
    private TalonFX shooter_left;
    private TalonFX shooter_right;

    public shooter() {
        shooter_left = new TalonFX(3);
        //shooter_right = new TalonFX(11);

        //shooter_right.setControl(new Follower(shooter_left.getDeviceID(), true));
    }

    private void set_vel(Double new_vel) {
        shooter_left.set(new_vel);
        System.out.println(shooter_left.getVelocity().toString());
    }

    public Command set_motor_speed(Double speed) {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return run(
            () -> {
                set_vel(speed);
            });
    }
    public Command zero_motor_speed() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return run(
            () -> {
                set_vel(0d);
            });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
