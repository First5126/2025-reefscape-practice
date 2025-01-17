// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constats.DrivetrainConstants;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.LimelightHelpers.RawFiducial;

public class ActionSubsystem extends SubsystemBase {
  private CommandSwerveDrivetrain drivetrain;
  private SwerveRequest.RobotCentric drive;

  private PIDController lLRotController = new PIDController(0.1, 0, 0);
  private PIDController lLDriveController = new PIDController(0.1, 0, 0);
  private PIDController lLDriveYController = new PIDController(0.1, 0, 0);

  public ActionSubsystem(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric drive) {
    this.drivetrain = drivetrain;
    this.drive = drive;
  }

  public Command doAction(Supplier<RawFiducial> fiducialsSupplier) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return Commands.run(
      () -> {
          goToTag(fiducialsSupplier);
      },drivetrain);
  }

  private void goToTag(Supplier<RawFiducial> fiducialsSupplier) {
    RawFiducial fiducial = fiducialsSupplier.get();
    System.out.println("supplied: "+fiducial);
    if (fiducial!=null) {

      double rotOffset = fiducial.txnc;
      double out = lLRotController.calculate(rotOffset, 0);
    
      double xVelocity = 0.0;
      double yVelocity = 0.0;
      System.out.println("distance: "+fiducial.distToCamera);
      if (fiducial.distToCamera > 0.65) {
        xVelocity = -lLDriveController.calculate(fiducial.distToCamera,10);
        yVelocity = -lLDriveYController.calculate(rotOffset, 0);
        System.out.println("xvel: "+xVelocity);
      }

      drivetrain.setControl(drive.withRotationalRate(out).withVelocityX(xVelocity).withVelocityY(yVelocity));
    }
    else {
      drivetrain.setControl(drive.withRotationalRate(0).withVelocityX(0).withVelocityY(0));
    }
  }

  public boolean hasTarget() {
    // Query wether the lime light sees a april tag
    return LimelightHelpers.getTV("limelight-back");
  }

  public RawFiducial getNearestData(RawFiducial[] fiducials) {
    int closest = 999;
    RawFiducial returnFiducial = null;
    

    for (RawFiducial fiducial : fiducials) {
      if (fiducial.distToRobot<closest) {
        closest = (int) (fiducial.distToCamera);
        returnFiducial = fiducial;
      }
    }

    return returnFiducial;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
