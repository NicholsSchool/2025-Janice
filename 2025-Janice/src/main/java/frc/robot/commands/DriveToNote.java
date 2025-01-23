package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants.StagingLocations;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

public class DriveToNote extends SequentialCommandGroup {

  public DriveToNote(Drive drive) {
    addCommands(
        new DriveToPose(
            drive,
            AllianceFlipUtil.apply(
                new Pose2d(StagingLocations.spikeTranslations[1], new Rotation2d(0.0)))),
        new DriveToPose(
            drive,
            AllianceFlipUtil.apply(
                new Pose2d(StagingLocations.centerlineTranslations[4], new Rotation2d(90.0)))));
  }
}
