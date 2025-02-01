package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class QuickMovementCommandFactory extends SubsystemBase {
  
  private CommandSwerveDrivetrain m_drivetrain;
  private Supplier<Pose2d> m_robotPoseSupplier;
  private Pose2d m_pointPose;
  private double m_minimumDistance;

  public QuickMovementCommandFactory(CommandSwerveDrivetrain drivetrain) {
    this.m_drivetrain = drivetrain;
    this.m_robotPoseSupplier = m_drivetrain::getPose2d;
  }

  public Command moveToGamePosition(Pose2d position, Double distance, Command ... secondaryCommands) {
    this.m_pointPose = position;
    this.m_minimumDistance = distance;

    Command returnCommand = run(() -> {m_drivetrain.goToPose(position);}).until(this::isClose).alongWith(m_drivetrain.goToPose(position));

    for (Command command : secondaryCommands) {
      returnCommand = returnCommand.alongWith(command);
    }

    return returnCommand;
  }

  // checks if the distance between the robot and the point is less than or equal to the minimum distance
  private boolean isClose() {
    return m_pointPose.getTranslation().getDistance(m_robotPoseSupplier.get().getTranslation())<=m_minimumDistance;
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
