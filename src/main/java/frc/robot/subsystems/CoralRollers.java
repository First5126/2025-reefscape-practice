// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralRollers extends SubsystemBase {
    private SparkMax coralSparkMax;
    /** Creates a new ExampleSubsystem. */
    public CoralRollers() {
        coralSparkMax = new SparkMax(0, MotorType.kBrushless);
    }

    private void runRollers(double speed){
        coralSparkMax.set(speed);
    }
    
    /**
     * Example command factory method.
    *
    * @return a command
    */
    public Command setRollerSpeed(double speed) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
                runRollers(speed);
                /* one-time action goes here */
            });
    }
    private void rollIn() {
        coralSparkMax.set(0.8);
    }
    public Command rollInCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
            return runOnce(
                () -> {
                    rollIn();
                    /* one-time action goes here */
                });
    }
    private void rollOut() {
        coralSparkMax.set(-0.8);
    }
    public Command rollOutCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
            return runOnce(
                () -> {
                    rollOut();
                    /* one-time action goes here */
                });
    }
    private void stop() {
        coralSparkMax.set(0);
    }
    public Command stopCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
            return runOnce(
                () -> {
                    stop();
                    /* one-time action goes here */
                });
    }
} 
