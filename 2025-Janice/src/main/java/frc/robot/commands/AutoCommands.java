package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.VisionConstants;
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
  // /**
  //  * this is a drive to pose that rather than it being an actual pose it just chases an object and turns until it's centered
  //  * this would be used for like a drive to algae, but the measurements 
  //  * @return
  //  */
  // public Command weaveToPose(double objectAngle, double objectArea){
  //   var weaveToPos = DriveCommands.joystickDrive(drive, () -> 0.0, () -> objectArea == 0 ? 0.0 : 0.5, () -> objectArea == 0 ? 0.1 : DriveCommands.angleToVelocity(objectAngle, 0.0), () -> true);
  //   return weaveToPos.until(() -> objectArea > VisionConstants.weaveToPoseBreakArea);
  // }

  public Command splineToPose(Pose2d pose) {
    var splToPose =
        new SplineToPose(
            this.drive,
            () -> {
              return pose;
            });
    return splToPose.until(splToPose::atGoal);
  }

  public Command TenFootTest(Drive drive) {
    return new DriveToPose(drive, new Pose2d(new Translation2d(3.048, 0), new Rotation2d(0)));
  }
  // /**
  //  * code for testing localization
  //  * @param objectAngle angle algae is from 
  //  * @param objectArea
  //  * @return
  //  */
  // public Command guardianOfTheReef(PhotonTrackedTarget target){
  //    return new SequentialCommandGroup(driveToPose(new Pose2d(new Translation2d(15, 4), new Rotation2d())));

  // }
}
