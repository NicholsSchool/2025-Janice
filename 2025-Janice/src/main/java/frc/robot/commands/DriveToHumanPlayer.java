package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;

public class DriveToHumanPlayer extends DriveToPose {
    
  private static final LoggedTunableNumber robotRotationOffset =
      new LoggedTunableNumber("DriveToHumanPlayer/RobotRotationOffset", Math.PI);
// Drives to the closest HumanPlayer to the bot.
public DriveToHumanPlayer(Drive drive) {
    super(
        drive, true,
        () -> {
            double distance = Double.MAX_VALUE;
            int tagListOffset;
            int targetTag = -1;
            Pose2d targetPose = new Pose2d();
        
            if (DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
              tagListOffset = 1;
            }else{
              tagListOffset = 12;
            }
        
            for(int i = 0; i < 2; i++){
              Pose3d tagPose = FieldConstants.aprilTags.getTagPose(i + tagListOffset).get();
              double distFromITag = drive.getPose().getTranslation().getDistance(tagPose.getTranslation().toTranslation2d());
              if(distance > distFromITag){
                distance = distFromITag;
                targetPose = tagPose.toPose2d();
                targetTag = i + tagListOffset;
              }
            }

            // calculate offset from target pose for robot width and bumpers. This should put robot
            // right against the HumanPlayer base.
            double offsetDistanceBumper = Constants.RobotConstants.robotGoToPosBuffer;

            Pose2d offsetPose = new Pose2d(
              new Translation2d(targetPose.getX() + Math.cos(targetPose.getRotation().getRadians()) * offsetDistanceBumper,
              targetPose.getY() + Math.sin(targetPose.getRotation().getRadians()) * offsetDistanceBumper),
               targetPose.getRotation().plus(new Rotation2d(robotRotationOffset.get())));

            Logger.recordOutput("DriveToHumanPlayer/TargetedTag", targetTag);
            Logger.recordOutput("DriveToHumanPlayer/TargetedPose", targetPose);
            Logger.recordOutput("DriveToHumanPlayer/OffsetPose", targetPose);
            
            return offsetPose;
        });
  }
}
