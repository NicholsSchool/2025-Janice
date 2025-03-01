package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;

public class DriveToReef extends InstantCommand {
  private Drive drive;

  public DriveToReef( Drive drive) {
    this.drive = drive;
    hasRequirement(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    var currentPose = drive.getPose();
    int desiredTag = closestReefTag(currentPose);

     //gets the tag pose from the nearest tag id 
     Pose2d desiredTagPose = FieldConstants.aprilTags.getTagPose(desiredTag).get().toPose2d();
     double offsetDistance = Constants.RobotConstants.bumperThickness + Constants.RobotConstants.robotSideLengthInches;
     
     Transform2d tagTransform = new Transform2d(desiredTagPose.getRotation().getCos() * offsetDistance,
     desiredTagPose.getRotation().getSin() * (offsetDistance), new Rotation2d(Math.PI / 2));
     //.transformBy(tagTransform)
     // puts itself right in front of a tag on the reef and rotates 90 so the outtake is pointed at it 
    
      
  }

   public int closestReefTag(Pose2d drivePose){
    double distance = Double.MAX_VALUE;
    int tagListOffset;
    int desiredTag = 1;

    if (DriverStation.getAlliance().isPresent()
    && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      tagListOffset = 6;
    }else{
      tagListOffset = 17;
    }

    for(int i = 0; i < 6; i++){
      double distFromITag = drivePose.getTranslation().getDistance(FieldConstants.aprilTags.getTagPose(i + tagListOffset).get().getTranslation().toTranslation2d());
      if(distance > distFromITag){
        distance = distFromITag;
        desiredTag = i + tagListOffset;
      }
    }

    return desiredTag;
    
  }
}