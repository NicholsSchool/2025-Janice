package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
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
              return AllianceFlipUtil.apply(pose);
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

  public Command TenFootTest(Drive drive) {
    return new DriveToPose(drive, new Pose2d(new Translation2d(3.048, 0), new Rotation2d(0)));
  }
}
