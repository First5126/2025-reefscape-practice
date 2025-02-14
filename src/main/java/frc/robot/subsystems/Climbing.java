package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.ClimbingConstants;

/**
 * The Climbing subsystem is a simple mechanism that moves the robot up and down.
 */
public class Climbing extends SubsystemBase {

  private final TalonFX m_leftMotor = new TalonFX(CANConstants.LEFT_CLIMBING_MOTOR);
  private final TalonFX m_rightMotor = new TalonFX(CANConstants.RIGHT_CLIMBING_MOTOR);
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0).withFeedForward(0);
  private final DigitalInput m_forwardLimit = new DigitalInput(ClimbingConstants.FORWARD_DIGITAL_LIMIT);
  private final DigitalInput m_reverseLimit = new DigitalInput(ClimbingConstants.REVERSE_DIGITAL_LIMIT);

  public Climbing() {

    HardwareLimitSwitchConfigs leftConfig = new HardwareLimitSwitchConfigs();
    HardwareLimitSwitchConfigs rightConfig = new HardwareLimitSwitchConfigs();

    leftConfig.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANcoder;
    leftConfig.ForwardLimitRemoteSensorID = ClimbingConstants.FORWARD_DIGITAL_LIMIT;

    rightConfig.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANcoder;
    rightConfig.ForwardLimitRemoteSensorID = ClimbingConstants.FORWARD_DIGITAL_LIMIT;

    m_rightMotor.setControl(new Follower(m_leftMotor.getDeviceID(), true));
    m_leftMotor.setNeutralMode(NeutralModeValue.Brake);
    m_leftMotor.setControl(new DutyCycleOut(0));

    m_leftMotor.getConfigurator().apply(leftConfig);
    m_rightMotor.getConfigurator().apply(rightConfig);
  }
/*
 * This method sets the position of the climbing mechanism to the desired position
 */
  public Command climb() {
    return run(() -> {setPosition(ClimbingConstants.ROTATIONS_FOR_CLIMB);});
  }
/**
 * This method sets the position of the climbing mechanism to 0
 * @return
 */
  public Command unClimb() {
    return run(() -> {setPosition(0);});
  }
/**
 * This method sets the position of the climbing mechanism to the desired position
 * @param position
 */
  public void setPosition(double position){
    m_leftMotor.setControl(m_positionVoltage.withPosition(0).withLimitReverseMotion(m_reverseLimit.get()));
  }
}
