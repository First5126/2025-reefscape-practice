package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaeConstants;
import frc.robot.constants.CANConstants;

/**
 * The AlgaeRollers subsystem is a simple mechanism that intakes and outtakes game pieces.
 */
public class AlgaeRollers extends SubsystemBase {
  public TalonFX m_motorOne = new TalonFX(CANConstants.LEFT_ALGAE_MOTOR);
  public TalonFX m_motorTwo = new TalonFX(CANConstants.RIGHT_ALGAE_MOTOR);
  public VelocityVoltage m_velocityVoltage = new VelocityVoltage(0);

  public AlgaeRollers() {
    m_motorTwo.setControl(new Follower(m_motorOne.getDeviceID(), true));
  }

  public Command feedIn() {
    return runOnce(
        () -> {
          m_motorOne.setControl(m_velocityVoltage.withVelocity(AlgaeConstants.INTAKE_SPEED));
        });
  }

  public Command feedOut() {
    return runOnce(
        () -> {
          m_motorOne.setControl(m_velocityVoltage.withVelocity(AlgaeConstants.OUTTAKE_SPEED));
        });
  }

  public Command stop() {
    return runOnce(
        () -> {
          m_motorOne.setControl(m_velocityVoltage.withVelocity(0.0));
        });
  }

  public void periodic() {
    //SmartDashboard.putString("Algae Motor Rotation", m_motorOne.getDescription());
  }
}
