package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

public class AutoCommands {
  // Subsystems
  private Drive drive;

  public AutoCommands(Drive drive) {
    this.drive = drive;
  }

  public Command driveToPose(Pose2d pose) {
    var drvToPose =
        new DriveToPose(
            this.drive,
            () -> {
              // return AllianceFlipUtil.apply(pose);
              return pose;
            });
    return drvToPose.until(drvToPose::atGoal);
  }

  public Command driveToPoseRelative(Pose2d pose) {
    var drvToPose =
        new DriveToPose(
            this.drive,
            () -> {
              return pose;
            });
    return drvToPose.until(drvToPose::atGoal);
  }

  public Command splineToPose(Pose2d pose) {
    var splToPose =
        new SplineToPose(
            this.drive,
            () -> {
              return pose;
            });
    return splToPose.until(splToPose::atGoal);
  }

  //   public int closestReefTag(Supplier<Pose2d> drivePose){
  //   double distance = Double.MAX_VALUE;
  //   int tagListOffset;
  //   int desiredTag = 1;

  //   if (DriverStation.getAlliance().isPresent()
  //   && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
  //     tagListOffset = 6;
  //   }else{
  //     tagListOffset = 17;
  //   }

  //   for(int i = 0; i < 6; i++){
  //     double distFromITag = drivePose.get().getTranslation().getDistance(FieldConstants.aprilTags.getTagPose(i + tagListOffset).get().getTranslation().toTranslation2d());
  //     if(distance > distFromITag){
  //       distance = distFromITag;
  //       desiredTag = i + tagListOffset;
  //     }
  //   }

  //   return desiredTag;
    
  // }

  // public Command DriveToReef(Drive drive){
  //   //gets the tag pose from the nearest tag id 
  //   Pose2d desiredTagPose = drive.closestReefTag();
    // double offsetDistance = Constants.RobotConstants.bumperThickness + Constants.RobotConstants.robotSideLengthInches;
    
    // Transform2d tagTransform = new Transform2d(desiredTagPose.getRotation().getCos() * offsetDistance,
    // desiredTagPose.getRotation().getSin() * (offsetDistance), new Rotation2d(Math.PI / 2));
  //   //.transformBy(tagTransform)
  //   // puts itself right in front of a tag on the reef and rotates 90 so the outtake is pointed at it 
  //   return new DriveToPose(drive, desiredTagPose);
  // }
}
