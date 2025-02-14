// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Revolutions;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ElevatorConstants.CoralLevels;


public class Elevator extends SubsystemBase {
  private final TalonFX m_leftMotor = new TalonFX(CANConstants.LEFT_ELAVOTAR_MOTOR);
  private final TalonFX m_rightMotor = new TalonFX(CANConstants.RIGHT_ELAVOTAR_MOTOR);
  private final PositionVoltage m_PositionVoltage = new PositionVoltage(0).withSlot(0).withFeedForward(0);
  private final VoltageOut m_VoltageOut = new VoltageOut(0);  

  // These fields are for when the driver taps up or down on the dpad. The elvator will go up or down a whole coral level
  private int m_goalHeightIndex = 0;

  public Elevator() {
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftConfig.Feedback.SensorToMechanismRatio = 24.0;
    leftConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    
    m_leftMotor.getConfigurator().apply(leftConfig);
    m_rightMotor.getConfigurator().apply(rightConfig);
    m_rightMotor.setControl(new Follower(m_leftMotor.getDeviceID(), true));
    m_leftMotor.setControl(m_VoltageOut.withOutput(0));
  }

  public double getElevatorHeight(){
    return m_leftMotor.getPosition().getValueAsDouble() / 24.0 * 2.0 * Math.PI * 0.05;//24:1 gear ratio, 2" diameter pulley
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Height: ", getElevatorHeight());
  }

  private void setSpeed(double speed){
    setControl(m_VoltageOut.withOutput(speed * 12.0));
  }

  private void setControl(ControlRequest control){
    m_leftMotor.setControl(control);
  }

  public Command openLoopCommand(Supplier<Double> speed) {
    return run(() -> setSpeed(speed.get()));
  }

  private void changeGoalHeightIndex(int change) {
    m_goalHeightIndex += change;

    if (m_goalHeightIndex<0) m_goalHeightIndex = 0;
    if (m_goalHeightIndex>CoralLevels.values().length-1) m_goalHeightIndex = CoralLevels.values().length-1;
    setPosition(CoralLevels.values()[m_goalHeightIndex]);
  }

  public Command lowerElevator() {
    return runOnce(() -> changeGoalHeightIndex(-1)).until(this::getIsAtPosition);
  }

  public Command raiseElevator() {
    return run(() -> changeGoalHeightIndex(1)).until(this::getIsAtPosition);
  }

  private boolean getIsAtPosition() {
    return m_leftMotor.getPosition().getValue() == CoralLevels.values()[goalHeightIndex].heightAngle;
  }

  //using exesting mPositionVoltage write set position method in meters
  private void setPosition(CoralLevels position){
    setControl(m_PositionVoltage.withPosition(position.heightAngle));
  }

  public Command goToCoralHeightPosition(CoralLevels position) {
    return run(
      ()-> setPosition(position)
      );
  }
}
