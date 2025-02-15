package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateManager extends SubsystemBase {
  public static class Reef {
    int pose, level;
    boolean left;

    public Reef(int pose, int level, boolean left) {
      this.pose = pose;
      this.level = level;
      this.left = left;
    }

    public int getReefPose() {
      return pose;
    }

    public int getReefLevel() {
      return level;
    }

    public boolean getReefLeft() {
      return left;
    }
  }

  public static class Algae {
    int pose, level;

    public Algae(int pose, int level) {
      this.pose = pose;
      this.level = level;
    }

    public int getAlgaePose() {
      return pose;
    }

    public int getAlgaeLevel() {
      return level;
    }
  }

  public List<Algae> algaeList;
  public List<Reef> reefList;

  /** Creates a new StateManager. */
  public StateManager() {
    algaeList = new ArrayList<>();
    reefList = new ArrayList<>();
  }

  public Command addToReefList(int pose, int level, boolean left) {
    return run(() -> {
      reefList.add(new Reef(pose, level, left));
    });
  }

  public Command addToAlgaeList(int pose, int level) {
    return run(() -> {
      algaeList.add(new Algae(pose, level));
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command removeFromReefList(int pose, int level, boolean left) {
    return run(() -> {
      reefList.removeIf(reef -> reef.getReefPose() == pose && reef.getReefLevel() == level && reef.getReefLeft() == left);
    });
  }

  public Command removeFromAlgaeList(int pose, int level) {
    return run(() -> {
      algaeList.removeIf(algae -> algae.getAlgaePose() == pose && algae.getAlgaeLevel() == level);
    });
  }

  public Command run(Runnable toRun) {
    return new InstantCommand(toRun, this);
  }
}

