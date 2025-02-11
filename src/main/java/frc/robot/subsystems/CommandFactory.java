package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.ElevatorConstants.CoralLevels;
import frc.robot.constants.PoseConstants.Pose;


public class CommandFactory {
  
  private CommandSwerveDrivetrain m_drivetrain;
  private Supplier<Pose2d> m_robotPoseSupplier;
  private Elevator m_elevator;
  private CoralRollers m_coralRollers;
  private AlgaeRollers m_algaeRollers;
  private Climbing m_climbing;
  private LedLights m_ledLights;

  public CommandFactory(
    CommandSwerveDrivetrain drivetrain,
    AlgaeRollers algaeRollers,
    Climbing climbing,
    Elevator elevator,
    CoralRollers coralRollers,
    LedLights ledLights
  ) {
    this.m_drivetrain = drivetrain;
    this.m_robotPoseSupplier = m_drivetrain::getPose2d;
    this.m_elevator = elevator;
    this.m_coralRollers = coralRollers;
    this.m_algaeRollers = algaeRollers;
    this.m_climbing = climbing;
    this.m_ledLights = ledLights;
  }

  /*
   * Moves the robot to a position and then runs the secondary commands
   */
  public Command moveToPositionWithDistance(Pose2d position, Distance distance, Command ... secondaryCommands) { 
    Command goToPose = m_drivetrain.goToPose(position);
    BooleanSupplier isClose = ()-> position.getTranslation().getDistance(m_robotPoseSupplier.get().getTranslation()) <= distance.in(Meters);
    Command then = Commands.waitUntil(isClose).andThen(secondaryCommands);
    Command returnCommand = Commands.parallel(goToPose,then); 
    
    return returnCommand;
  }

  public Command driveAndPlaceCoral(Pose reefPose, CoralLevels level) {
    Command raiseElevator = m_elevator.setPosition(level);
    Command driveToReef = moveToPositionWithDistance(reefPose.getPose(), level.distance, raiseElevator);
    Command placeCoral = m_coralRollers.rollOutCommand();
    Command returnCommand = driveToReef.andThen(placeCoral);
    
    return returnCommand;
  }
}