package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class QuickMovementCommandFactory extends SubsystemBase {
  
  private CommandSwerveDrivetrain m_drivetrain;
  private Supplier<Pose2d> m_robotPoseSupplier;
  private Pose2d m_pointPose = new Pose2d(0,0,Rotation2d.fromDegrees(0));
  private double m_minimumDistance = 0;

  public QuickMovementCommandFactory(CommandSwerveDrivetrain drivetrain) {
    this.m_drivetrain = drivetrain;
    this.m_robotPoseSupplier = m_drivetrain::getPose2d;
  }

  public Command moveToGamePosition(Pose2d position, Double distance, Command ... secondaryCommands) {

    this.m_pointPose = position;
    this.m_minimumDistance = distance;

    Command goToPose = m_drivetrain.goToPose(position);
    Command then = Commands.waitUntil(this::isClose).andThen(secondaryCommands);

    Command returnCommand = Commands.parallel(goToPose,then);
    
    return returnCommand;
  }

  // checks if the distance between the robot and the point is less than or equal to the minimum distance
  private boolean isClose() {
    return m_pointPose.getTranslation().getDistance(m_robotPoseSupplier.get().getTranslation())<=m_minimumDistance;
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putBoolean("IsWithinDistance",isClose());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
