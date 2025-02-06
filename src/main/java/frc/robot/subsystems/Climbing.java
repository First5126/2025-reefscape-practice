package frc.robot.subsystems;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbingConstants;

public class Climbing extends SubsystemBase {

  private final TalonFX m_leftMotor = new TalonFX(0);
  private final TalonFX m_rightMotor = new TalonFX(1);
  private final PositionVoltage m_PositionVoltage = new PositionVoltage(0).withSlot(0).withFeedForward(0);

  private final DigitalInput m_forwardLimit = new DigitalInput(0);
  private final DigitalInput m_reverseLimit = new DigitalInput(1);

  public Climbing() {

    m_rightMotor.setControl(new Follower(m_leftMotor.getDeviceID(), true));

  }

  public Command climb() {
    return run(() -> {setControl(m_PositionVoltage.withPosition(ClimbingConstants.ROTATIONS_FOR_CLIMB).withLimitForwardMotion(m_forwardLimit.get()).withLimitReverseMotion(m_reverseLimit.get()));});
  }

  public Command unClimb() {
    return run(() -> {setControl(m_PositionVoltage.withPosition(0).withLimitForwardMotion(m_forwardLimit.get()).withLimitReverseMotion(m_reverseLimit.get()));});

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
