package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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
  private static final RobotType robot = RobotType.ROBOT_REAL_JANICE;
  public static final boolean driveRobotRelative =
      false; // set to true to override all field relative and instead command in robot-relative.

  // true to place tunable numbers in dashboard for setting, false otherwise
  public static final boolean tuningMode = true;
  public static final double loopPeriodSecs = 0.02;
  public static final double MeterPerInch = 0.0254;
  public static final double KgPerLb = 0.453592;

  public static final double JOYSTICK_DEADBAND = 0.08;

  public static RobotType getRobot() {
    return RobotBase.isReal() ? robot : RobotType.ROBOT_SIM;
  }

  public static enum RobotType {
    ROBOT_REAL_FRANKENLEW, // a real robot (LEW ZEALAND)
    ROBOT_REAL_JANICE, // a real robot (JANICE)
    ROBOT_REPLAY, // data file replay (could be on real bot or simulation)
    ROBOT_SIM, // simulation
    ROBOT_FOOTBALL, // Football for simulating
    ROBOT_CALIBRATE
  }

  // CAN IDs (Controller Area Network)
  public static final class CAN {
    public static final int kReduxGyro = 20;

    public static final int kFrontLeftDrive = 21;
    public static final int kBackLeftDrive = 22;
    public static final int kFrontRightDrive = 24;
    public static final int kBackRightDrive = 23;

    public static final int kFrontLeftPivot = 25;
    public static final int kBackLeftPivot = 26;
    public static final int kFrontRightPivot = 28;
    public static final int kBackRightPivot = 27;

    public static final int kFrontLeftEncoder = 29;
    public static final int kBackLeftEncoder = 30;
    public static final int kFrontRightEncoder = 32;
    public static final int kBackRightEncoder = 31;

    public static final int kPowerDistributionHub = 50;
    
    public static int kMaxFrontLeftDrivingCanId = 24;
    public static int kMaxFrontRightDrivingCanId = 26;
    public static int kMaxRearLeftDrivingCanId = 22;
    public static int kMaxRearRightDrivingCanId = 28;
    
    public static int kMaxFrontLeftTurningCanId = 23;
    public static int kMaxFrontRightTurningCanId = 25;
    public static int kMaxRearLeftTurningCanId = 21;
    public static int kMaxRearRightTurningCanId = 27;

    public static int kIntakeMotor;

    public static final int kLeftChain = 41;
    public static final int kRightChain = 42;
    public static final int elevatorEncoder = 43;
    public static final int kOuttakeMotor = 51;

    public static final int kDeAlgifierLaterator = 37;
    public static final int kDeAlgifierGrabber = 38;
  }

  public static final class RobotConstants {
    public static final double robotSideLengthInches =
        // 34.0; // robot was measured bumper to bumper to be 33in, +1 in for buffer.
        33.0; 

    
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

      public static final double bumperThicknessMeters = Units.inchesToMeters(3.5);

      public static final double robotGoToPosBuffer = Units.inchesToMeters(robotSideLengthInches) / 2 + bumperThicknessMeters;
  }

  public static final class DriveConstants {
    public static final double kMAX_LINEAR_SPEED = 3.2;
    public static final double kTRACK_WIDTH_X = Units.inchesToMeters(25); // 23.5in
    public static final double kTRACK_WIDTH_Y = Units.inchesToMeters(24.5);

    public static final double lowGearScaler = 0.6;
    public static final double reefLeftShift = 0.1 + 0.08 - Units.inchesToMeters(5);
    public static final double reefRightShift = 0.22 + 0.08;
  }

  // REV MAXSwerve Modules
  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0) * 0.9569;
    public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2.0;
    public static final double kDrivingMotorFreeSpinRPM = 5870; // Kraken non-FOC max RPM
    public static final double odometryCoefficient = 1.0;

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
  }

  public static final class ElevatorConstants{
    public static final double ElevatorCurrentLimit = 35.0;

    public static final double kElevatorP = 13;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 0;
    public static final double ElevatorMaxAccelerationRad = 1000;
    public static final double ElevatorMaxVelocityRad = 1000;
    public static final double maxHeight = -4.0;
    public static final double minHeight = 0.0;

    public static final int elevatorLimitSwitchChannel = 0;

    public static final double kElevatorGearRatio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    
    public static final double kArmL1 = -0.01;
    public static final double kArmL2 = -2.3;
    public static final double kArmL3 = -3.7;
    public static final double kArmL4 = -2.0;

    public static final double kAlgaeL2 = -0.4;
    public static final double kAlgaeL3 = -3.38;

    public static final double kElevatorManualScaler = 5.0;

  }

  public static final class IntakeConstants {

    public static final double kIndexerGearRatio = 1.0;
    public static final double INTAKE_CURRENT_LIMIT = 0;
    public static final double INTAKE_P = 0;
    public static final double INTAKE_I = 0;
    public static final double INTAKE_D = 0;
    public static final int kLeftPistonChannel = -1;
    public static final int kRightPistonChannel = -1;
    public static final double kCoralDistanceCloseBound = 300;
    public static final double kCoralDistanceFarBound = 400;

  }

  public static final class OuttakeConstants {
    public static final double kOuttakeCurrentLimit = 35.0;
    public static final double kCoralDistanceFarBound = 1.7;
    public static final double kCoralDistanceCloseBound = 1.3;
  }

public static final class DeAlgifierConstants {
  public static final double kDeAlgifierGrabberCurrentLimit = 60; //amps
  public static final double kDeAlgifierLateratorCurrentLimit = 30; // low to make sure does not overrdrive into hard stop

  public static final double kLateratorGearRatio = 5;
  public static final double kGrabberGearRatio = 3.86;

  public static final double kGrabberIntakeSetpointRPM = -300.0;
  public static final double kGrabberEjectSetpointRPM = +300.0;
  public static final double kGrabberP = 5.0;
  public static final double kGrabberD = 0.0;

  public static final double kLateratorInPositionRad = 5.0; //only used in SIM
  public static final double kLateratorOutPositionRad = -5.0; //TODO: redo sim dealgifier

  public static final double kLateratorVelocityGoalRadPerSec = 5;
  public static final double kLateratorPVelocity = 2.0;
  public static final double kLateratorDVelocity = 0.0;
}
    
  public static final class AutoConstants {
    public static final double driveFinishThreshold = 0.075;
    public static final double angleFinishThreshold = Math.PI / 12.0;

    public static final double splineFinishThreshold = 0.4;
    public static final double splineAngleFinishThreshold = Math.PI / 12.0;
    public static final double SplineV5LinearMultiplier = 1.5;
    public static final double SplineV5CircularMultiplier = 2.0;
    public static final double dotProductThreshold = 0.05;

    public static final double reefAutoRadius = 2.5;
    public static final Translation2d reefAutoCircle = new Translation2d(4.45, 4.1);
    public static final double splineV5Multipler = 0.7;
  }

  public static final class FiddleSongs {
    public static final String ALL_STAR = "all-star.chrp"; // TODO: add fnaf
    public static final String IMPERIAL_MARCH = "Imperial-March.chrp";
    public static final String WII_SONG = "Wii-Song.chrp";
  }

  public static final class VisionConstants {
    // both of these are the translation of the two cameras from the center of the bot
    // need to check which camera is - and which is +
    public static Transform3d cameraOnePosition = new Transform3d(-0.3, 0, 0.5, new Rotation3d(0, 0, -Math.PI / 2));
    public static Translation2d cameraTwoPosition = new Translation2d(-0.3, 0.3);
    public static double cameraOneAngle = Units.degreesToRadians(45);

    public static final int initVisionCountTreshold = 100;
    public static final double visionDistanceUpdateThreshold = 1.0; //meters


    public static final double tranlationPhotonStdDevs = 0.01;
    public static final double rotationPhotonStdDevs = 0.005;

    public static final int visionStatsNumBuffer = 100;
  }
}
