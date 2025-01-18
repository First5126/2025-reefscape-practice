// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class AprilTagLocalizationConstants {
  public static class LimelightDetails {
    public String name;
    public Matrix<N3, N1> closeStdDevs;
    public Matrix<N3, N1> farStdDevs;

    /*
     * 
     */
    public LimelightDetails(String name, Matrix<N3, N1> closeStdDevs, Matrix<N3, N1> farStdDevs) {
      this.name = name;
      this.closeStdDevs = closeStdDevs;
      this.farStdDevs = farStdDevs;
    }
  }

  public static final String LIMELIGHT_NAME = "limelight";
  public static final Matrix<N3, N1> LIMELIGHT_CLOSE_STDDEV =  VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(0));
  public static final Matrix<N3, N1> LIMELIGHT_FAR_STDDEV =  VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(0));

  public static final LimelightDetails LIMELIGHT_DETAILS = new LimelightDetails(LIMELIGHT_NAME, LIMELIGHT_CLOSE_STDDEV, LIMELIGHT_CLOSE_STDDEV);
}
