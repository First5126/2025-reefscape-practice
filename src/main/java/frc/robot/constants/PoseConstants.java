package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class PoseConstants {

  public static class Pose {
    private Pose2d redPose,bluePose;
    public Pose(Pose2d bluePose, Pose2d redPose) {
      this.bluePose = bluePose;
      this.redPose = redPose;
    }
    public Pose2d getPose(){
      if (DriverStation.getAlliance().get().name().equals("red")){
        return redPose;
      } else {
        return bluePose;
      } 
    }
  }
  

  public static final Pose leftCoralStationPosition1 = new Pose(
    new Pose2d(1.7,7.3,Rotation2d.fromDegrees(125)),
    new Pose2d(16.8,1.4,Rotation2d.fromDegrees(-55))
    );
  public static final Pose leftCoralStationPosition2 = new Pose(
    new Pose2d(1.3,7,Rotation2d.fromDegrees(125)),
    new Pose2d(16.2,1,Rotation2d.fromDegrees(-55))
    );
  public static final Pose leftCoralStationPosition3 = new Pose(
    new Pose2d(0.8,6.6,Rotation2d.fromDegrees(125)),
    new Pose2d(15.7,0.6,Rotation2d.fromDegrees(-55))
    );

  public static final Pose prossesor = new Pose(
    new Pose2d(6.4,0.6,Rotation2d.fromDegrees(-90)),
    new Pose2d(11.5,7.45,Rotation2d.fromDegrees(90))
    );

    public static final Pose ReefPosition1 = new Pose(
    new Pose2d(0.95,4,Rotation2d.fromDegrees(0)),
    new Pose2d(11.66,4.01,Rotation2d.fromDegrees(0))
    ); 
    public static final Pose ReefPosition2 = new Pose(
    new Pose2d(3.8,5.2,Rotation2d.fromDegrees(-55)),
    new Pose2d(12.3,5.2,Rotation2d.fromDegrees(-55))
    ); 
    public static final Pose ReefPosition3 = new Pose(
    new Pose2d(5.184,5.235,Rotation2d.fromDegrees(-120)),
    new Pose2d(13.75,5.22,Rotation2d.fromDegrees(-120))
    ); 
    public static final Pose ReefPosition4 = new Pose(
    new Pose2d(5.9,4,Rotation2d.fromDegrees(180)),
    new Pose2d(14.5,4,Rotation2d.fromDegrees(180))
    ); 
    public static final Pose ReefPosition5 = new Pose(
    new Pose2d(5.2,2.8,Rotation2d.fromDegrees(120)),
    new Pose2d(13.7,2.7,Rotation2d.fromDegrees(120))
    ); 
    public static final Pose ReefPosition6 = new Pose(
    new Pose2d(3.74,2.8,Rotation2d.fromDegrees(55)),
    new Pose2d(12.36,2.85,Rotation2d.fromDegrees(55))
    ); 

}

