// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaeConstants;



public class AlgaeRollers extends SubsystemBase {
  
  public TalonFX m_motorOne = new TalonFX(0);
  public TalonFX m_motorTwo = new TalonFX(1);

  

  
  /** Creates a new ExampleSubsystem. */
  public AlgaeRollers() {
    m_motorTwo.setControl(new Follower(m_motorOne.getDeviceID(), true));
  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command feedIn() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          m_motorOne.set(AlgaeConstants.speed);
        });
  }

  public Command feedOut() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          m_motorOne.set(-AlgaeConstants.speed);
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
    SmartDashboard.putString("Algae Moter Rotation", m_motorOne.getDescription());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
