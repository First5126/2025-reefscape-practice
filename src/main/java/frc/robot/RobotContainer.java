// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
  private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.DriveTrain;

  private final CommandXboxController m_driver_controller = new CommandXboxController(0);
  private final CommandXboxController m_codriver_controller = new CommandXboxController(1);

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform*/
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public RobotContainer() {
    configureDriverControls();
    configureCoDriverControls();
  }

  private void configureDriverControls() {
    // Setup drivetrain default command.  Should this be elsewhere???
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    m_drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_drivetrain.applyRequest(() ->
            drive.withVelocityX(-m_driver_controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-m_driver_controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-m_driver_controller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        )
    );
    m_drivetrain.registerTelemetry(logger::telemeterize);

    // Setup driver's controlls
    m_driver_controller.a().whileTrue(m_drivetrain.applyRequest(() -> brake));
    m_driver_controller.b().whileTrue(m_drivetrain.applyRequest(() ->
        point.withModuleDirection(new Rotation2d(-m_driver_controller.getLeftY(), -m_driver_controller.getLeftX()))
    ));

    m_drivetrain.setDefaultCommand(
        m_drivetrain.gasPedalCommand(
            m_driver_controller::getRightTriggerAxis,
            m_driver_controller::getRightX,
            m_driver_controller::getLeftY,
            m_driver_controller::getLeftX
        )
    );
  }

  private void configureCoDriverControls() {
    // Setup codriver's controlls
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
