package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

public class DriveToAmplifier extends DriveToPose {
  /**
   * Automatically drives to the amplifier and places robot in a scoring position.
   *
   * <p>Fudge factors to account for field alyout inaccuracies: fudgeXinch - amplifier position
   * fudge in X direction fudgeYinch - amplifier position fudge in Y direction
   */
  public DriveToAmplifier(Drive drive, double fudgeXinch, double fudgeYinch) {
    super(
        drive,
        () -> {
          return AllianceFlipUtil.apply(
              new Pose2d(
                  FieldConstants.ampCenter.getX() + Units.inchesToMeters(fudgeXinch),
                  FieldConstants.ampCenter.getY()
                      + Units.inchesToMeters(fudgeYinch)
                      - Units.inchesToMeters(
                          Constants.RobotConstants.robotSideLengthInches / 2.0
                              + 7.75), // 7.75 to take into account the arm poking outside the frame
                  Rotation2d.fromDegrees(-90.0)));
        });
  }
}
