// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Deleted Elevator branch


package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Record extends SubsystemBase {

  private String m_positionSelection = "";
  private boolean positionEnabled = false;
  

  public Record() {
    //initialize code below
    
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command setFar() {
    return Commands.runOnce(
        () -> {
             m_positionSelection = "far";
              });
  }
  public Command setNear() {
    return Commands.runOnce(
        () -> {
              m_positionSelection = "near";
              });
  }
  public Command setRight() {
    return Commands.runOnce(
        () -> {
          if (positionEnabled) {
            if (m_positionSelection == "far") {
              m_positionSelection = "far right";
            } else {
              m_positionSelection = "near right";
            }
          }             
        });
  }
  public Command setLeft() {
    return Commands.runOnce(
        () -> {
          if (positionEnabled) {
            if (m_positionSelection == "far") {
              m_positionSelection = "far left";
            } else {
              m_positionSelection = "near left";
            }
          }
        });
  }
  public Command SetEnabled() {
    return Commands.runOnce(
        () -> {
             positionEnabled = !positionEnabled;
             m_positionSelection = "";
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
    SmartDashboard.putBoolean("Position enabled", positionEnabled);
    SmartDashboard.putString("Position Selection", m_positionSelection);
    //add code below
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
