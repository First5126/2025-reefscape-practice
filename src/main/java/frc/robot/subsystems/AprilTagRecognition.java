package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaeConstants;
import frc.robot.constants.AprilTagLocalizationConstants;
import frc.robot.constants.CANConstants;
import frc.robot.constants.PoseConstants;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.LimelightHelpers.RawFiducial;

public class AprilTagRecognition extends SubsystemBase {
  private HashMap<Integer, Command> m_AprilTagHashMap;
  private CommandFactory m_commandFactory;
  private int currentAprilID = 0;

  public AprilTagRecognition(CommandFactory commandFactory) {
    m_commandFactory = commandFactory;
    m_AprilTagHashMap = new HashMap<>(23);
    //The numbers in the hashmap correlate to the AprilTag numbers
    m_AprilTagHashMap.put(0, Commands.none());
    m_AprilTagHashMap.put(1, m_commandFactory.goToCommand(PoseConstants.leftCoralStationPosition2.getPose()));
    m_AprilTagHashMap.put(2, m_commandFactory.goToCommand(PoseConstants.rightCoralStationPosition2.getPose()));
    m_AprilTagHashMap.put(3, m_commandFactory.goToCommand(PoseConstants.prossesor.getPose()));
    m_AprilTagHashMap.put(4, m_commandFactory.goToCommand(PoseConstants.rightBarge.getPose()));
    m_AprilTagHashMap.put(5, m_commandFactory.goToCommand(PoseConstants.LeftBarge.getPose()));
    m_AprilTagHashMap.put(6, m_commandFactory.goToCommand(PoseConstants.ReefPosition5.getPose()));
    m_AprilTagHashMap.put(7, m_commandFactory.goToCommand(PoseConstants.ReefPosition4.getPose()));
    m_AprilTagHashMap.put(8, m_commandFactory.goToCommand(PoseConstants.ReefPosition3.getPose()));
    m_AprilTagHashMap.put(9, m_commandFactory.goToCommand(PoseConstants.ReefPosition2.getPose()));
    m_AprilTagHashMap.put(10, m_commandFactory.goToCommand(PoseConstants.ReefPosition1.getPose()));
    m_AprilTagHashMap.put(11, m_commandFactory.goToCommand(PoseConstants.ReefPosition6.getPose()));
    m_AprilTagHashMap.put(12, m_commandFactory.goToCommand(PoseConstants.rightCoralStationPosition2.getPose()));
    m_AprilTagHashMap.put(13, m_commandFactory.goToCommand(PoseConstants.leftCoralStationPosition2.getPose()));
    m_AprilTagHashMap.put(14, m_commandFactory.goToCommand(PoseConstants.LeftBarge.getPose()));
    m_AprilTagHashMap.put(15, m_commandFactory.goToCommand(PoseConstants.rightBarge.getPose()));
    m_AprilTagHashMap.put(16, m_commandFactory.goToCommand(PoseConstants.prossesor.getPose()));
    m_AprilTagHashMap.put(17, m_commandFactory.goToCommand(PoseConstants.ReefPosition6.getPose()));
    m_AprilTagHashMap.put(18, m_commandFactory.goToCommand(PoseConstants.ReefPosition1.getPose()));
    m_AprilTagHashMap.put(19, m_commandFactory.goToCommand(PoseConstants.ReefPosition2.getPose()));
    m_AprilTagHashMap.put(20, m_commandFactory.goToCommand(PoseConstants.ReefPosition3.getPose()));
    m_AprilTagHashMap.put(21, m_commandFactory.goToCommand(PoseConstants.ReefPosition4.getPose()));
    m_AprilTagHashMap.put(22, m_commandFactory.goToCommand(PoseConstants.ReefPosition5.getPose()));
  }


  private int getClosestTagId(){
    RawFiducial[] allTags = LimelightHelpers.getRawFiducials(AprilTagLocalizationConstants.LIMELIGHT_NAME);
    RawFiducial closestTag;
    int result = 0;
    if (allTags.length > 0) {
      closestTag = allTags[0];
      result = closestTag.id;
    } else {
      result = 0;
    }
    
    return result;
  }


  public Command getAprilTagCommand() {
    Command tagCommand = Commands.select(m_AprilTagHashMap, this::getClosestTagId);
    currentAprilID = this.getClosestTagId();
    return tagCommand;
  }


  public void periodic() {
    SmartDashboard.putNumber("Current April Tag Command ID", currentAprilID);
  }
}
