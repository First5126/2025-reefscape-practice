// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ActionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisonSubsystem;

public class RobotContainer {
  private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.DriveTrain;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform*/
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric driveRobCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final CommandXboxController m_driver_controller = new CommandXboxController(0);
    private final CommandXboxController m_codriver_controller = new CommandXboxController(1);
    private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();
    
    private final CommandXboxController m_driverController = new CommandXboxController(0);

    private final ActionSubsystem m_actionSubsystem = new ActionSubsystem(m_drivetrain,driveRobCentric);
    private final VisonSubsystem m_visonSubsystem = new VisonSubsystem();

    public RobotContainer() {
        configureBindings();
        configureCoDriverControls();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        
        m_drivetrain.setDefaultCommand(m_drivetrain.gasPedalCommand(
            m_driverController::getRightTriggerAxis,
            m_driverController::getRightX,
            m_driverController::getLeftY,
            m_driverController::getLeftX
        ));

        m_driverController.a().whileTrue(m_actionSubsystem.doAction(m_visonSubsystem::getCLosestFiducial));
        m_driverController.b().whileTrue(m_drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
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
    return autoChooser.getSelected();
  }
}
