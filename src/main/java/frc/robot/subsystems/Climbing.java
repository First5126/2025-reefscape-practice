package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbingConstants;

public class Climbing extends SubsystemBase {

  private final TalonFX m_leftMotor = new TalonFX(0);
  private final TalonFX m_rightMotor = new TalonFX(1);
  private final PositionVoltage m_PositionVoltage = new PositionVoltage(0).withSlot(0).withFeedForward(0);

  public Climbing() {
    m_rightMotor.setControl(new Follower(m_leftMotor.getDeviceID(), true));
  }

  public Command climb() {
    return run(() -> {setControl(m_PositionVoltage.withPosition(ClimbingConstants.ROTATIONS_FOR_CLIMB));});
  }

  public Command unClimb() {
    return run(() -> {setControl(m_PositionVoltage.withPosition(0));});
  }

  public void setControl(ControlRequest control){
    m_leftMotor.setControl(control);
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
