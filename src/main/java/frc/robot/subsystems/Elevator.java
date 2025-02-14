// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;

import frc.robot.constants.ElevatorConstants.CoralLevels;

/**
 * The Elevator subsystem is a simple mechanism that moves the elevator up and down.
 */
public class Elevator extends SubsystemBase {
  private final TalonFX m_leftMotor = new TalonFX(CANConstants.LEFT_ELAVOTAR_MOTOR);
  private final TalonFX m_rightMotor = new TalonFX(CANConstants.RIGHT_ELAVOTAR_MOTOR);
  private final PositionVoltage m_PositionVoltage = new PositionVoltage(0).withSlot(0).withFeedForward(0);
  private final VoltageOut m_VoltageOut = new VoltageOut(0);  
  private final CANdi m_CANDi = new CANdi(0);

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
/*
 * This method sets the position of the elevator Height, the equation is set up to convert the position of the motor to the height of the elevator
 */
  public double getElevatorHeight(){
    return m_leftMotor.getPosition().getValueAsDouble() / 24.0 * 2.0 * Math.PI * 0.05; //12:1 gear ratio, 2" diameter pulley
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Height: ", getElevatorHeight());
  }
/*
 * This method sets the speed of the elevator motor
 */
  private void setSpeed(double speed){
    setControl(m_VoltageOut.withOutput(speed * 12.0));
  }
/*
 * This method sets the control of the elevator motor.
 */
  private void setControl(ControlRequest control){
    m_leftMotor.setControl(control);
  }
/*
 * This method sets the speed of the elevator motor
 */
  public Command openLoopCommand(Supplier<Double> speed) {
    return run(() -> setSpeed(speed.get()));
  }


  //using exesting mPositionVoltage write set position method in meters
  public Command setPosition(CoralLevels position){
    return run(() -> setControl(m_PositionVoltage.withPosition(position.heightAngle)));
  }
}