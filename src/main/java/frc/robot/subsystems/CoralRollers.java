// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.CANConstants;
import frc.robot.constants.CoralConstants;

/**
 * The CoralRollers subsystem is a simple mechanism that intakes and outtakes game pieces.  
 */
public class CoralRollers extends SubsystemBase {

    private TalonFXS m_coralTalonFXS;// TalonFXS is a class that represents a motor controller. We are using TalonFXS to control the rollers
    private VelocityVoltage m_VelocityVoltage; // VelocityVoltage is a class that represents a motor controller's velocity control mode

    private CANrange m_LeftCANrange; // CANrange is a class that represents a range sensor
    private CANrange m_RightCANrange; // CANrange is a class that represents a range sensor

    private Trigger m_hasGamePiece; // Trigger is a class that represents a button that can be pressed
/**
 * The CoralRollers subsystem is a simple mechanism that intakes and outtakes game pieces.
 */
    public CoralRollers() {
        m_VelocityVoltage = new VelocityVoltage(0);

        m_coralTalonFXS = new TalonFXS(0);
        m_coralTalonFXS.setControl(new DutyCycleOut(0));

        CANrangeConfiguration CANrangeConfiguration = new CANrangeConfiguration();
        CANrangeConfiguration.ProximityParams.ProximityThreshold = CoralConstants.PROXIMITY_THRESHOLD;

        m_LeftCANrange = new CANrange(CANConstants.LEFT_CAN_RANGE_CORAL);
        m_LeftCANrange.getConfigurator().apply(CANrangeConfiguration);

        m_RightCANrange = new CANrange(CANConstants.RIGHT_CAN_RANGE_CORAL); // Left and right range sensors
        m_RightCANrange.getConfigurator().apply(CANrangeConfiguration); // Left and right range sensors

        m_hasGamePiece = new Trigger(this::isDetected).debounce(CoralConstants.DEBOUNCE); // Trigger is a class that represents a button that can be pressed
    }
/**
 * Returns the button that can be pressed
 * @return
 */
    public Trigger getCoralTrigger() {// Trigger is a class that represents a button that can be pressed
        return m_hasGamePiece;
    }
/*
 * Returns whether the game piece is detected
 */
    private boolean isDetected() {
        return m_LeftCANrange.getIsDetected().getValue() || m_RightCANrange.getIsDetected().getValue();
    }
/*
 * This method sets the speed of the rollers
 */
    private void runRollers(double speed){
        m_coralTalonFXS.set(speed);
    }

    public Command setRollerSpeed(double speed) {
        return runOnce(
            () -> {
                runRollers(speed);
            });
    }
    /*
     * This methods sets the rollers to intake the game piece. The bottom method is a command that runs the top method until the game piece is detected
     */
    private void rollIn() {
        m_coralTalonFXS.setControl(m_VelocityVoltage.withVelocity(CoralConstants.INTAKE_SPEED));
    }
    
    public Command rollInCommand() {
            return run(
                () -> {
                    rollIn();
                }).until(m_hasGamePiece).andThen(stopCommand());
    }
    /*
     * This method sets the rollers to outtake the game piece
     */
    private void rollOut() {
        m_coralTalonFXS.setControl(m_VelocityVoltage.withVelocity(CoralConstants.OUTTAKE_SPEED));
    }
    /*
     * This method sets the rollers to outtake the game piece. The bottom method is a command that runs the top method until the game piece is detected
     */
    public Command rollOutCommand() {
            return run(
                () -> {
                    rollOut();
                }).onlyWhile(m_hasGamePiece).andThen(stopCommand());
    }
    /*
     * This method stops the rollers
     */
    private void stop() {
        m_coralTalonFXS.setControl(new DutyCycleOut(0));
    }
    /*
     * This method stops the rollers
     */
    public Command stopCommand() { 
            return runOnce(
                () -> {
                    stop();
                });
    }
} 
