package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
  public static final double fieldLength = Units.inchesToMeters(653.2);
  public static final double fieldWidth = Units.inchesToMeters(323.28);
  public static final double tapeWidth = Units.inchesToMeters(2.0);
  public static final double wingX = Units.inchesToMeters(229.201);
  public static final double podiumX = Units.inchesToMeters(126.75);
  public static final double startingLineX = Units.inchesToMeters(74.111);

  public static final Translation2d ampCenter =
      new Translation2d(
          Units.inchesToMeters(76.1),
          fieldWidth); // turns out we did the math wrong, it's in the document with all the
  // drawings

  /** Staging locations for each note */
  public static final class StagingLocations {
    public static final double centerlineX = fieldLength / 2.0;

    // need to update
    public static final double centerlineFirstY = Units.inchesToMeters(29.638);
    public static final double centerlineSeparationY = Units.inchesToMeters(66);
    public static final double spikeX = Units.inchesToMeters(114);
    // need
    public static final double spikeFirstY = Units.inchesToMeters(161.638);
    public static final double spikeSeparationY = Units.inchesToMeters(57);

    public static final Translation2d[] centerlineTranslations = new Translation2d[5];
    public static final Translation2d[] spikeTranslations = new Translation2d[3];

    static {
      for (int i = 0; i < centerlineTranslations.length; i++) {
        centerlineTranslations[i] =
            new Translation2d(centerlineX, centerlineFirstY + (i * centerlineSeparationY));
      }
    }

    static {
      for (int i = 0; i < spikeTranslations.length; i++) {
        spikeTranslations[i] = new Translation2d(spikeX, spikeFirstY + (i * spikeSeparationY));
      }
    }
  }

  /** Each corner of the speaker * */
  public static final class Speaker {

    // corners (blue alliance origin)
    public static final Translation3d topRightSpeaker =
        new Translation3d(
            Units.inchesToMeters(18.055),
            Units.inchesToMeters(238.815),
            Units.inchesToMeters(83.091));

    public static final Translation3d topLeftSpeaker =
        new Translation3d(
            Units.inchesToMeters(18.055),
            Units.inchesToMeters(197.765),
            Units.inchesToMeters(83.091));

    public static final Translation3d bottomRightSpeaker =
        new Translation3d(0.0, Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
    public static final Translation3d bottomLeftSpeaker =
        new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));

    /** Center of the speaker opening (blue alliance) */
    public static final Translation3d centerSpeakerOpening =
        bottomLeftSpeaker.interpolate(topRightSpeaker, 0.5);
  }

  public static final class Subwoofer {
    public static final Pose2d ampFaceCorner =
        new Pose2d(
            Units.inchesToMeters(35.775),
            Units.inchesToMeters(239.366),
            Rotation2d.fromDegrees(-120));

    public static final Pose2d sourceFaceCorner =
        new Pose2d(
            Units.inchesToMeters(35.775),
            Units.inchesToMeters(197.466),
            Rotation2d.fromDegrees(120));

    public static final Pose2d centerFace =
        new Pose2d(
            Units.inchesToMeters(35.775),
            Units.inchesToMeters(218.416),
            Rotation2d.fromDegrees(180));
  }

  public static final class Stage {
    public static final Pose2d podiumLeg =
        new Pose2d(Units.inchesToMeters(126.75), Units.inchesToMeters(161.638), new Rotation2d());
    public static final Pose2d ampLeg =
        new Pose2d(
            Units.inchesToMeters(220.873),
            Units.inchesToMeters(212.425),
            Rotation2d.fromDegrees(-30));
    public static final Pose2d sourceLeg =
        new Pose2d(
            Units.inchesToMeters(220.873),
            Units.inchesToMeters(110.837),
            Rotation2d.fromDegrees(30));

    public static final Pose2d centerPodiumAmpChain =
        new Pose2d(
            podiumLeg.getTranslation().interpolate(ampLeg.getTranslation(), 0.5),
            Rotation2d.fromDegrees(120.0));
    public static final Pose2d centerAmpSourceChain =
        new Pose2d(
            ampLeg.getTranslation().interpolate(sourceLeg.getTranslation(), 0.5), new Rotation2d());
    public static final Pose2d centerSourcePodiumChain =
        new Pose2d(
            sourceLeg.getTranslation().interpolate(podiumLeg.getTranslation(), 0.5),
            Rotation2d.fromDegrees(240.0));
    public static final Pose2d center =
        new Pose2d(Units.inchesToMeters(192.55), Units.inchesToMeters(161.638), new Rotation2d());
    public static final double centerToChainDistance =
        center.getTranslation().getDistance(centerPodiumAmpChain.getTranslation());
  }

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