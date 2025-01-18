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


/**
 * A class that uses the limelight to localize the robot using AprilTags.
 * it runs in a background thread instead of the main robot loop.
 */
public class AprilTagLocalization {
  private Notifier m_notifier = new Notifier(this::poseEstimate);  //calls pose estimate on the the period
  private LimelightDetails[] m_LimelightDetails;  // list of limelights that can provide updates
  private Supplier<Pose2d> m_robotPoseSupplier;  // supplies the pose of the robot
  private boolean fullTrust;  //to allow for button trust the tag estimate over all else.

  /**
   * Creates a new AprilTagLocalization.
   * @param poseSupplier supplies the current robot pose
   * @param visionConsumer // a consumer that accepts the vision pose, timestamp, and std deviations
   * @param details // the details of the limelight, more than one can be passed to allow for multipe on the bot.
   */
  public AprilTagLocalization(Supplier<Pose2d> poseSupplier, VisionConsumer visionConsumer,  LimelightDetails ... details) {
    m_notifier.startPeriodic(0.02); // set up a pose estimation loop with a 0.02 second period.
    m_LimelightDetails= details;
    m_robotPoseSupplier = poseSupplier;
  }

  /**
   * Sets the full trust of the vision system.  The robot will trust the vision system over all other sensors.
   * @param fullTrust
   */
  public void setFullTrust(boolean fullTrust) {
    this.fullTrust = fullTrust;
  }

  public void poseEstimate() {
    for (LimelightDetails limelightDetail : m_LimelightDetails) {
      // Set Orientation using LimelightHelpers.SetRobotOrientation and the m_robotPoseSupplier
      // Get the pose from the Limelight
      // Validate the pose for sanity reject bad poses  if fullTrust is true accept regarless of sanity
      // reject poses that are more than max tag distance we trust
      // scale std deviation by distance if fullTrust is true set the stdDevs super low.
      // set the pose in the pose consumer
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
