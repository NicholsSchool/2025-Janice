package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  private static final RobotType robot = RobotType.ROBOT_REAL;
  public static final boolean driveRobotRelative =
      false; // set to true to override all field relative and instead command in robot-relative.

  // true to place tunable numbers in dashboard for setting, false otherwise
  public static final boolean tuningMode = true;
  public static final double loopPeriodSecs = 0.02;
  public static final double MeterPerInch = 0.0254;
  public static final double KgPerLb = 0.453592;

  public static final double JOYSTICK_DEADBAND = 0.05;

  public static RobotType getRobot() {
    return RobotBase.isReal() ? robot : RobotType.ROBOT_SIM;
  }

  public static enum RobotType {
    ROBOT_REAL, // a real robot
    ROBOT_REPLAY, // data file replay (could be on real bot or simulation)
    ROBOT_SIM, // simulation
    ROBOT_FOOTBALL // Football for simulating
  }

  // CAN IDs (Controller Area Network)
  public static final class CAN {

    public static final int kFrontLeftDrive = 20;
    public static final int kFrontLeftPivot = 21;
    public static final int kFrontLeftEncoder = 22;

    public static final int kFrontRightDrive = 30;
    public static final int kFrontRightPivot = 31;
    public static final int kFrontRightEncoder = 32;

    public static final int kBackLeftDrive = 40;
    public static final int kBackLeftPivot = 41;
    public static final int kBackLeftEncoder = 42;

    public static final int kBackRightDrive = 50;
    public static final int kBackRightPivot = 51;
    public static final int kBackRightEncoder = 52;

    public static final int kShoulderL = 38;
    public static final int kShoulderR = 39;

    public static final int kIntake = 29;

    public static final int kIndexer = 47;

    public static final int kShooterTop = 48;
    public static final int kShooterBottom = 49;

    public static final int kPowerDistributionHub = 50;
  }

  public static final class RobotConstants {
    public static final double robotSideLengthInches =
        // 34.0; // robot was measured bumper to bumper to be 33in, +1 in for buffer.
        33.0; // why do we need a +1 buffer? J-Burnett
  }

  public static final class DriveConstants {
    public static final double kMAX_LINEAR_SPEED = 4.8;
    public static final double kTRACK_WIDTH_X = 0.5969; // 23.5in
    public static final double kTRACK_WIDTH_Y = 0.5969;

    public static final double lowGearScaler = 0.6;
  }

  // REV MAXSwerve Modules
  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0); // 4-in with tread
    public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2.0;
    public static final double kDrivingMotorFreeSpinRPM = 6000; // Kraken non-FOC max RPM
    public static final double odometryCoefficient = 1.0; // TODO: tune odometry and use this

    public static final double kDrivingP = 0.02; // TODO: tune these
    public static final double kDrivingI = 0.0;
    public static final double kDrivingD = 0.0;
    public static final double kDrivingFF = 0.0;
    public static final double kDrivingStaticFF = 0.1;
    public static final double kDrivingVelocityFF = 0.13;

    public static final double kTurningP = 6.2; // TODO: tune these
    public static final double kTurningI = 0.0;
    public static final double kTurningD = 0.12;
    public static final double kTurningFF = 0.0;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 35; // amps
    public static final int kTurningMotorCurrentLimit = 35; // amps

    // SDS MK4i L2 Modules
    public static final double kDRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double kTURN_GEAR_RATIO = 150.0 / 7.0;
  }

  public static final class ArmConstants {
    public static double ArmStartPosDeg = 90.0;

    public static final int ARM_CURRENT_LIMIT = 35; // amps
    public static final double MIN_ANGLE_RADS = 0;
    public static final double MAX_ANGLE_RADS = Math.PI;

    public static final double ARM_GEAR_REDUCTION = 45.0;
    public static final double ARM_GEAR_RATIO = 1.0 / ARM_GEAR_REDUCTION;

    public static final double kAbsoluteEncoderOffset = 0.0;
    // the absolute encoder value in degrees when the intended value is zero

    public static final double ARM_P = 5;
    public static final double ARM_I = 0.0;
    public static final double ARM_D = 0.02;

    public static final int kThroughBoreChannel = 7;
  }

  public static final class IntakeConstants {

    public static double kVomitRPM = -800;
    public static double kEatRPM = 600;
    public static double kDigestRPM = 1500;
    public static double kPoopRPM;
    public static double kVomitDelay;
  }

  public static final class IndexerConstants {
    public static final double kINDEXER_GEAR_RATIO = 1.0;

    public static double kIndexRPM = -1000;
    public static double kReverseRPM = 500;

    public static final int INDEX_CURRENT_LIMIT = 35; // amps
  }

  public static final class ShooterConstants {
    public static final double kSHOOTER_GEAR_RATIO = 1.0;

    public static final double kShooterRPM = 2500;
    public static final double kDeliverRPM = 1000;
    public static final double kReverseRPM = 400;

    public static final double kCurrentLimit = 35; // amps

    public static double kP = 1.5;
    public static double kD = 0.0;

    public static double shootRampUpTimeSecs = 2.5;
  }

  public static final class AutoConstants {
    public static final double driveFinishThreshold = 0.075; // TODO: tune these
    public static final double angleFinishThreshold = Math.PI / 12.0;
  }

  public static final class FiddleSongs {
    public static final String ALL_STAR = "all-star.chrp"; // TODO: add fnaf
    public static final String IMPERIAL_MARCH = "Imperial-March.chrp";
    public static final String WII_SONG = "Wii-Song.chrp";
  }

  public static final class VisionConstants {
    // both of these are the translation of the two cameras from the center of the bot
    // need to check which camera is - and which is +
    public static Translation2d cameraOnePosition = new Translation2d(-0.3, -0.3);
    public static Translation2d cameraTwoPosition = new Translation2d(-0.3, 0.3);
    public static double cameraOneAngle = Units.degreesToRadians(45);
  }
}
