package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;

public class FaceReef extends DriveToPose {
    
  private static final LoggedTunableNumber robotRotationOffset =
      new LoggedTunableNumber("DriveToReef/RobotRotationOffset", -Math.PI/2);
public static enum ReefDirection{
  CENTER,
  LEFT,
  RIGHT
}
// Drives to the closest reef to the bot.
public FaceReef(Drive drive) {
    super(
        drive, true,
        () -> {
            double distance = Double.MAX_VALUE;
            int tagListOffset;
            int targetTag = -1;
            Pose2d targetPose = new Pose2d();
        
            if (DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
              tagListOffset = 6;
            }else{
              tagListOffset = 17;
            }
        
            for(int i = 0; i < 6; i++){
              Pose3d tagPose = FieldConstants.aprilTags.getTagPose(i + tagListOffset).get();
              double distFromITag = drive.getPose().getTranslation().getDistance(tagPose.getTranslation().toTranslation2d());
              if(distance > distFromITag){
                distance = distFromITag;
                targetPose = tagPose.toPose2d();
                targetTag = i + tagListOffset;
              }
            }


            Pose2d offsetPose = new Pose2d(
              drive.getPose().getTranslation(),
               targetPose.getRotation().plus(new Rotation2d(robotRotationOffset.get())));

            Logger.recordOutput("DriveToReef/TargetedTag", targetTag);
            Logger.recordOutput("DriveToReef/TargetedPose", targetPose);
            Logger.recordOutput("DriveToReef/OffsetPose", targetPose);
            
            return offsetPose;
        });
  }
}
