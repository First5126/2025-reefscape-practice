// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
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

  private double max_tag_distance = 0.0;

  private VisionConsumer m_VisionConsumer;


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
    m_VisionConsumer = visionConsumer;
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

  private Matrix<N3, N1> interpolate(Matrix<N3, N1> closeStdDevs, Matrix<N3, N1> farStdDevs, double scale){
    return VecBuilder.fill(MathUtil.interpolate(closeStdDevs.get(0,0), farStdDevs.get(0,0), scale),
                    MathUtil.interpolate(closeStdDevs.get(1,0), farStdDevs.get(1,0), scale),
                    MathUtil.interpolate(closeStdDevs.get(2,0), farStdDevs.get(2,0), scale));
  }

  public void poseEstimate() {
    for (LimelightDetails limelightDetail : m_LimelightDetails) {
      double yawDegrees = m_robotPoseSupplier.get().getRotation().getDegrees();
      double yawRateDegreesPerSecond = (yawDegrees-oldyawDegrees)/0.02;
      // Set Orientation using LimelightHelpers.SetRobotOrientation and the m_robotPoseSupplier
      LimelightHelpers.SetRobotOrientation(limelightDetail.name, yawDegrees, yawRateDegreesPerSecond,0,0,0,0 );  // Set Orientation using LimelightHelpers.SetRobotOrientation and the m_robotPoseSupplier
      // Get the pose from the Limelight
      PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightDetail.name);  // Get the pose from the Limelight 
      double scale = poseEstimate.avgTagDist / max_tag_distance;
      // Validate the pose for sanity reject bad poses  if fullTrust is true accept regarless of sanity
      if (fullTrust) {
        // set the pose in the pose consumer
        m_VisionConsumer.accept(poseEstimate.pose, poseEstimate.timestampSeconds, limelightDetail.closeStdDevs);
      } else if (isPoseOnfield(poseEstimate.pose) && poseEstimate.avgTagDist < max_tag_distance) { // reject poses that are more than max tag distance we trust
        // scale std deviation by distance if fullTrust is true set the stdDevs super low.
        Matrix<N3,N1> interpolated = interpolate(limelightDetail.closeStdDevs, limelightDetail.farStdDevs, scale);
        // set the pose in the pose consumer
        m_VisionConsumer.accept(poseEstimate.pose, poseEstimate.timestampSeconds, interpolated);
      }
      oldyawDegrees = yawDegrees;
    }
  }
  
  @FunctionalInterface
  public interface VisionConsumer {
    void accept(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs);
  }
}