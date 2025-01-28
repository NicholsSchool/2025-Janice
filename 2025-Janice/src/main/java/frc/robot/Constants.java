package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
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
    public static final int kReduxGyro = 11;

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

    public static final int kPowerDistributionHub = 50;
  }

  public static final class RobotConstants {
    public static final double robotSideLengthInches =
        // 34.0; // robot was measured bumper to bumper to be 33in, +1 in for buffer.
        33.0; // why do we need a +1 buffer? J-Burnett

    
    public static final double robotMass = 42; //kg
    public static final double MOI = 7.2; // kgm/s
    public static final ModuleConfig moduleConfig = new ModuleConfig(
      ModuleConstants.kWheelDiameterMeters / 2.0, 
      DriveConstants.kMAX_LINEAR_SPEED, 
      0.7, 
      new DCMotor(12.0, 7.09, 120.0, 40.0, 3700.0, 1), 
      40, 
      4);
      
    public static final RobotConfig robotConfig = new RobotConfig(
      robotMass,
      MOI, 
      moduleConfig,
      Units.inchesToMeters(robotSideLengthInches));
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

    public static final double kDrivingP = 0.02; 
    public static final double kDrivingI = 0.0;
    public static final double kDrivingD = 0.0;
    public static final double kDrivingFF = 0.0;
    public static final double kDrivingStaticFF = 0.1;
    public static final double kDrivingVelocityFF = 0.13;

    public static final double kTurningP = 6.2;
    public static final double kTurningI = 0.0;
    public static final double kTurningD = 0.12;
    public static final double kTurningFF = 0.0;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 90; // amps
    public static final int kTurningMotorCurrentLimit = 40; // amps
    public static final int kMotorSupplyCurrentLimit = 35; //amps

    // SDS MK4i L2 Modules
    public static final double kDRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double kTURN_GEAR_RATIO = 150.0 / 7.0;
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
