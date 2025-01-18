// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.vision.AprilTagLocalizationConstants.LimelightDetails;
import frc.robot.vision.LimelightHelpers.PoseEstimate;


/**
 * A class that uses the limelight to localize the robot using AprilTags.
 * it runs in a background thread instead of the main robot loop.
 */
public class AprilTagLocalization {
  private Notifier m_notifier = new Notifier(this::poseEstimate);  //calls pose estimate on the the period
  private LimelightDetails[] m_LimelightDetails;  // list of limelights that can provide updates
  private Supplier<Pose2d> m_robotPoseSupplier;  // supplies the pose of the robot
  private boolean fullTrust;  //to allow for button trust the tag estimate over all else.
  private double oldyawDegrees = 0;
 
  
  private double maxFieldDistanceX = 0.0;
  private double maxFieldDistanceY = 0.0;


  /**
   * Creates a new AprilTagLocalization.
   * @param poseSupplier supplies the current robot pose
   * @param visionConsumer // a consumer that accepts the vision pose, timestamp, and std deviations
   * @param details // the details of the limelight, more than one can be passed to allow for multipe on the bot.
   */
  public AprilTagLocalization(Supplier<Pose2d> poseSupplier, VisionConsumer visionConsumer,  LimelightDetails ... details) {
    m_notifier.startPeriodic(0.02); // set up a pose estimation loop with a 0.02 second period.
    m_LimelightDetails = details;
    m_robotPoseSupplier = poseSupplier;
  }

  /**
   * Sets the full trust of the vision system.  The robot will trust the vision system over all other sensors.
   * @param fullTrust
   */
  public void setFullTrust(boolean fullTrust) {
    this.fullTrust = fullTrust;
  }
  private boolean isPoseOnfield(Pose2d observation) { 
    // Coordinates for where Pose is on the field
    if (
      observation.getX() < 0.0 // What X position robot is on the field.
      || observation.getY() < 0.0 // What Y position robot is on the field.
      || observation.getX() > maxFieldDistanceX // Whether the robot X position is on the field or not
      || observation.getY() > maxFieldDistanceY // Whether the robot X position is on the field or not
    ){
      return true;
    } else {
      return false;
    }
  }

  public Pose2d poseEstimate() {
      for (LimelightDetails limelightDetail : m_LimelightDetails) {
        double yawDegrees = m_robotPoseSupplier.get().getRotation().getDegrees();
        double yawRateDegreesPerSecond = (yawDegrees-oldyawDegrees)/0.02;
        LimelightHelpers.SetRobotOrientation(limelightDetail.name, yawDegrees, yawRateDegreesPerSecond,0,0,0,0 );  // Set Orientation using LimelightHelpers.SetRobotOrientation and the m_robotPoseSupplier
        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(null);  // Get the pose from the Limelight 
        if (isPoseOnfield(poseEstimate.pose)){
          return poseEstimate.pose;
        }
        // Validates the pose for sanity reject bad poses if fullTrust is true accept regarless of sanity
        // rejects poses that are more than max tag distance we trust
      }
      return null; // return null if no valid pose is found
  }
  
  @FunctionalInterface
  public interface VisionConsumer {
    void accept(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs);
  }
}