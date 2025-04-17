package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.List;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise.
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall. Use the {@link #allianceFlip(Translation2d)} and {@link #allianceFlip(Pose2d)}
 * methods to flip these values based on the current alliance color.
 *
 * <p>Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)*
 */
public final class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(57.0 * 12);
  public static final double fieldWidth = Units.inchesToMeters(323.28);
  public static final double tapeWidth = Units.inchesToMeters(2.0);

  public static final AprilTagFieldLayout aprilTags =
      new AprilTagFieldLayout(
          List.of(
              new AprilTag(
                  1,
                  new Pose3d(
                      Units.inchesToMeters(657.37),
                      Units.inchesToMeters(25.80),
                      Units.inchesToMeters(58.50),
                      new Rotation3d(0, 0.0, Units.degreesToRadians(126)))),
              new AprilTag(
                  2,
                  new Pose3d(
                      Units.inchesToMeters(657.37),
                      Units.inchesToMeters(291.20),
                      Units.inchesToMeters(58.50),
                      new Rotation3d(0, 0.0, Units.degreesToRadians(234)))),
              new AprilTag(
                  3,
                  new Pose3d(
                      Units.inchesToMeters(455.15),
                      Units.inchesToMeters(317.15),
                      Units.inchesToMeters(51.25),
                      new Rotation3d(0, 0.0, Units.degreesToRadians(270)))),
              new AprilTag(
                  4,
                  new Pose3d(
                      Units.inchesToMeters(365.20),
                      Units.inchesToMeters(241.64),
                      Units.inchesToMeters(73.54),
                      new Rotation3d(0, Units.degreesToRadians(30), Units.degreesToRadians(0)))),
              new AprilTag(
                  5,
                  new Pose3d(
                      Units.inchesToMeters(365.20),
                      Units.inchesToMeters(75.39),
                      Units.inchesToMeters(73.54),
                      new Rotation3d(0, Units.degreesToRadians(30), Units.degreesToRadians(0)))),
              new AprilTag(
                  6,
                  new Pose3d(
                      Units.inchesToMeters(530.49),
                      Units.inchesToMeters(130.17),
                      Units.inchesToMeters(12.13),
                      new Rotation3d(0, 0, Units.degreesToRadians(300)))),
              new AprilTag(
                  7,
                  new Pose3d(
                      Units.inchesToMeters(546.87),
                      Units.inchesToMeters(158.50),
                      Units.inchesToMeters(12.13),
                      new Rotation3d(0.0, 0.0, Units.degreesToRadians(0)))),
              new AprilTag(
                  8,
                  new Pose3d(
                      Units.inchesToMeters(530.49),
                      Units.inchesToMeters(186.83),
                      Units.inchesToMeters(12.13),
                      new Rotation3d(0.0, 0.0, Units.degreesToRadians(60)))),
              new AprilTag(
                  9,
                  new Pose3d(
                      Units.inchesToMeters(497.77),
                      Units.inchesToMeters(186.83),
                      Units.inchesToMeters(12.13),
                      new Rotation3d(0, 0.0, Units.degreesToRadians(120)))),
              new AprilTag(
                  10,
                  new Pose3d(
                      Units.inchesToMeters(481.39),
                      Units.inchesToMeters(158.50),
                      Units.inchesToMeters(12.13),
                      new Rotation3d(0, 0.0, Units.degreesToRadians(180)))),
              new AprilTag(
                  11,
                  new Pose3d(
                      Units.inchesToMeters(497.77),
                      Units.inchesToMeters(130.17),
                      Units.inchesToMeters(12.13),
                      new Rotation3d(0, 0.0, Units.degreesToRadians(240)))),
              new AprilTag(
                  12,
                  new Pose3d(
                      Units.inchesToMeters(33.51),
                      Units.inchesToMeters(25.80),
                      Units.inchesToMeters(58.50),
                      new Rotation3d(0, 0.0, Units.degreesToRadians(54)))),
              new AprilTag(
                  13,
                  new Pose3d(
                      Units.inchesToMeters(33.51),
                      Units.inchesToMeters(291.20),
                      Units.inchesToMeters(58.50),
                      new Rotation3d(0, 0.0, Units.degreesToRadians(306)))),
              new AprilTag(
                  14,
                  new Pose3d(
                      Units.inchesToMeters(325.68),
                      Units.inchesToMeters(241.64),
                      Units.inchesToMeters(73.54),
                      new Rotation3d(0.0, Units.degreesToRadians(30), Units.degreesToRadians(180)))),
              new AprilTag(
                  15,
                  new Pose3d(
                      Units.inchesToMeters(325.68),
                      Units.inchesToMeters(75.39),
                      Units.inchesToMeters(73.54),
                      new Rotation3d(0, Units.degreesToRadians(30), Units.degreesToRadians(180)))),
              new AprilTag(
                  16,
                  new Pose3d(
                      Units.inchesToMeters(235.73),
                      Units.inchesToMeters(-0.15),
                      Units.inchesToMeters(51.25),
                      new Rotation3d(0, 0.0, Units.degreesToRadians(90)))),
                new AprilTag(
                        17,
                        new Pose3d(
                            Units.inchesToMeters(160.39),
                            Units.inchesToMeters(130.17),
                            Units.inchesToMeters(12.13),
                            new Rotation3d(0, 0.0, Units.degreesToRadians(240)))),
                new AprilTag(
                        18,
                        new Pose3d(
                        Units.inchesToMeters(144),
                        Units.inchesToMeters(158.50),
                        Units.inchesToMeters(12.13),
                        new Rotation3d(0, 0.0, Units.degreesToRadians(180)))),
                new AprilTag(
                        19,
                        new Pose3d(
                        Units.inchesToMeters(160.39),
                        Units.inchesToMeters(186.83),
                        Units.inchesToMeters(12.13),
                        new Rotation3d(0, 0.0, Units.degreesToRadians(120)))),
                new AprilTag(
                        20,
                        new Pose3d(
                        Units.inchesToMeters(193.10),
                        Units.inchesToMeters(186.83),
                        Units.inchesToMeters(12.13),
                        new Rotation3d(0, 0.0, Units.degreesToRadians(60)))),
                new AprilTag(
                        21,
                        new Pose3d(
                        Units.inchesToMeters(209.49),
                        Units.inchesToMeters(158.50),
                        Units.inchesToMeters(12.13),
                        new Rotation3d(0, 0.0, Units.degreesToRadians(0)))),
                new AprilTag(
                        22,
                        new Pose3d(
                        Units.inchesToMeters(193.10),
                        Units.inchesToMeters(130.17),
                        Units.inchesToMeters(12.13
                        ),
                        new Rotation3d(0, 0.0, Units.degreesToRadians(300))))),
                
          fieldLength,
          fieldWidth);
}