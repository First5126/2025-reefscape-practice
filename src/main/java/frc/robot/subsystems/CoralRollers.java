// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralRollers extends SubsystemBase {

    private SparkMax coralSparkMax;

    public CoralRollers() {
        coralSparkMax = new SparkMax(0, MotorType.kBrushless);
    }

    private void runRollers(double speed){
        coralSparkMax.set(speed);
    }
    
    public Command setRollerSpeed(double speed) {
        return runOnce(
            () -> {
                runRollers(speed);
            });
    }
    private void rollIn() {
        coralSparkMax.set(0.8);
    }
    public Command rollInCommand() {
            return runOnce(
                () -> {
                    rollIn();
                });
    }
    private void rollOut() {
        coralSparkMax.set(-0.8);
    }
    public Command rollOutCommand() {
            return runOnce(
                () -> {
                    rollOut();
                });
    }
    private void stop() {
        coralSparkMax.set(0);
    }
    public Command stopCommand() {
            return runOnce(
                () -> {
                    stop();
                });
    }
} 
