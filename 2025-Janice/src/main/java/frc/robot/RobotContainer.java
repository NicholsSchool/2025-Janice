package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIORedux;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  //private PowerDistribution pdh;

  // shuffleboard
  ShuffleboardTab boomerangTab;
  public static GenericEntry hasNote;

  // Controller
  public static CommandXboxController driveController = new CommandXboxController(0);
  public static CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Start position selections
  public static final LoggedTunableNumber startPositionIndex =
      new LoggedTunableNumber("start pos index: 0 or 1", 0.0);
  // Start Pos 0: Along line of the Amp.
  public static final LoggedTunableNumber startX0 =
      new LoggedTunableNumber("Start X0(m)", Units.inchesToMeters(0));
  public static final LoggedTunableNumber startY0 = new LoggedTunableNumber("Start Y0(m)", 4.05);
  public static final LoggedTunableNumber startTheta0 =
      new LoggedTunableNumber("Start Theta0(deg)", 0.0);
  // Start Pos 1: Next to human player side of Speaker.
  public static final LoggedTunableNumber startX1 =
      new LoggedTunableNumber(
          "Start X1(m)", Units.inchesToMeters(RobotConstants.robotSideLengthInches / 2));
  public static final LoggedTunableNumber startY1 = new LoggedTunableNumber("Start Y1(m)", 4.05);
  public static final LoggedTunableNumber startTheta1 =
      new LoggedTunableNumber("Start Theta1(deg)", 0.0);

  // Auto Commands
  //private final AutoCommands autoCommands;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getRobot()) {
      case ROBOT_REAL:
        // Real robot, instantiate hardware IO implementations
        //pdh = new PowerDistribution(Constants.CAN.kPowerDistributionHub, ModuleType.kRev);
        drive =
            new Drive(
                new GyroIORedux(),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        break;

      case ROBOT_SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        break;

      case ROBOT_FOOTBALL:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        break;

      default:
        // case ROBOT_REPLAY:
        // Replayed robot, disable IO implementations since the replay
        // will supply the data.

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Create auto commands
    //autoCommands = new AutoCommands(drive);

    // autoChooser.addOption("Wait 5 seconds", new WaitCommand(5.0));

    // add testing auto functions
    addTestingAutos();

    // initialize the shuffleboard outputs
    initShuffleboard();

    // Configure the button bindings
    configureButtonBindings();

    // set starting position of robot
    // setStartingPose();
  }

  private void initShuffleboard() {
    // Configure the Shuffleboard
    boomerangTab = Shuffleboard.getTab("Boomerang");
    // this is where display booleans will go
  }

  public void updateShuffleboard() {

    // if (Constants.getRobot() == RobotType.ROBOT_REAL) {
    //   SmartDashboard.putNumber("PDH/Voltage", pdh.getVoltage());
    //   SmartDashboard.putNumber("PDH/Current", pdh.getTotalCurrent());
    //   SmartDashboard.putNumber("PDH/Power", pdh.getTotalPower());
    //   SmartDashboard.putNumber("PDH/Energy", pdh.getTotalEnergy());

    //   int numChannels = pdh.getNumChannels();
    //   for (int i = 0; i < numChannels; i++) {
    //     SmartDashboard.putNumber("PDH/Channel " + i, pdh.getCurrent(i));
    //   }
    // }
      //System.out.println(LimelightHelpers.getTX("limelight"));

    // resetPosWithDashboard();
  }

  // changes robot pose with dashboard tunables
  private void resetPosWithDashboard() {

    // update robot position only if robot is disabled, otherwise
    // robot could move in unexpected ways.
    if (DriverStation.isDisabled()) {
      if (startX0.hasChanged(hashCode())
          || startY0.hasChanged(hashCode())
          || startTheta0.hasChanged(hashCode())
          || startX1.hasChanged(hashCode())
          || startY1.hasChanged(hashCode())
          || startTheta1.hasChanged(hashCode())
          || startPositionIndex.hasChanged(hashCode())) {

        setStartingPose();
      }
    }
  }

  /**
   * Set the starting pose of the robot based on position index. This should be called only when
   * robot is disabled.
   */
  public void setStartingPose() {
    // Set starting position only if operating robot in field-relative control.
    // Otherwise, robot starts at 0, 0, 0.
    if (!Constants.driveRobotRelative) {
      Pose2d startPosition0 =
          new Pose2d(
              startX0.get(), startY0.get(), new Rotation2d(Math.toRadians(startTheta0.get())));
      Pose2d startPosition1 =
          new Pose2d(
              startX1.get(), startY1.get(), new Rotation2d(Math.toRadians(startTheta1.get())));

      drive.setPose(
          startPositionIndex.get() == 0
              ? AllianceFlipUtil.apply(startPosition0)
              : AllianceFlipUtil.apply(startPosition1));
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY() * Constants.DriveConstants.lowGearScaler,
            () -> -driveController.getLeftX() * Constants.DriveConstants.lowGearScaler,
            () -> -driveController.getRightX() * 0.55,
            () -> Constants.driveRobotRelative));
    driveController.start().onTrue(new InstantCommand(() -> drive.resetFieldHeading()));
    driveController
        .leftTrigger(0.8)
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> -driveController.getRightX(),
                () -> Constants.driveRobotRelative));

    // drive.setDefaultCommand(
    //   DriveCommands.joystickDriveWithAngle(
    //     drive, 
    //     () -> -driveController.getLeftY() * 0.3, 
    //     () -> -driveController.getLeftX() * 0.3, 
    //     () -> -driveController.getRightX() * 0.5, 
    //     drive::getYaw,
    //     () -> false)
    // );

    driveController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveWithAngle(
                drive,
                () -> -driveController.getLeftY() * Constants.DriveConstants.lowGearScaler,
                () -> -driveController.getLeftX() * Constants.DriveConstants.lowGearScaler,
                () -> 180,
                () -> drive.getYaw(),
                () -> Constants.driveRobotRelative));
    driveController
        .y()
        .whileTrue(
            DriveCommands.joystickDriveWithAngle(
                drive,
                () -> -driveController.getLeftY() * Constants.DriveConstants.lowGearScaler,
                () -> -driveController.getLeftX() * Constants.DriveConstants.lowGearScaler,
                () -> LimelightHelpers.getTX("limelight"),
                () -> drive.getYaw(),
                () -> Constants.driveRobotRelative));
    driveController
        .x()
        .whileTrue(
            DriveCommands.joystickDriveWithAngle(
                drive,
                () -> -driveController.getLeftY() * Constants.DriveConstants.lowGearScaler,
                () -> -driveController.getLeftX() * Constants.DriveConstants.lowGearScaler,
                () -> 90,
                () -> drive.getYaw(),
                () -> Constants.driveRobotRelative));
    driveController
        .b()
        .whileTrue(
            DriveCommands.joystickDriveWithAngle(
                drive,
                () -> -driveController.getLeftY() * Constants.DriveConstants.lowGearScaler,
                () -> -driveController.getLeftX() * Constants.DriveConstants.lowGearScaler,
                () -> -90,
                () -> drive.getYaw(),
                () -> Constants.driveRobotRelative));
  }

  // /**
  //  * @return the command to run in autonomous
  //  */
  public Command getAutonomousCommand() {
    //   // var path = PathPlannerAuto.getStaringPoseFromAutoFile("TestAuto");
    //   // var path = PathPlannerPath.fromChoreoTrajectory("New Path");
    //   // NamedCommands.registerCommand("RunIntake", intake.runEatCommand().withTimeout(1.0));
    //   // NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> intake.stop(),
    // intake));
    //   // NamedCommands.registerCommand(
    //   //     "Shoot", new InstantCommand(() -> new Shoot(shooter, intake,
    // indexer)).withTimeout(3));
    //   // drive.setPose(PathPlannerAuto.getStaringPoseFromAutoFile("New Auto"));
    //   // return new PathPlannerAuto("New Auto");
    //   // return AutoBuilder.followPath(PathPlannerPath.fromPathFile("TestPath"));
    return autoChooser.get();
  }

  private void addAutos() {}

  private void addTestingAutos() {
    autoChooser.addOption("Wait Auto", new WaitCommand(Time.ofBaseUnits(5, Seconds)) );
    // Pathplanner Auto Testing
    // Set up feedforward characterization
    // autoChooser.addOption(
    //     "Drive FF Characterization",
    //     new FeedForwardCharacterization(
    //         drive,
    //         drive::runCharacterizationVolts,
    //         drive::getCharacterizationVelocity)); // todo change these for new robot

    // autoChooser.addOption(
    //     "Module Drive Ramp Test",
    //     new VoltageCommandRamp(drive, drive::runDriveCommandRampVolts, 0.5, 5.0));

    // autoChooser.addOption(
    //     "Module Turn Ramp Test",
    //     new VoltageCommandRamp(drive, drive::runTurnCommandRampVolts, 0.5, 5.0));

    // autoChooser.addOption(
    //     "Spline Test",
    //     autoCommands.splineToPose(
    //         new Pose2d(
    //             new Translation2d(4, 3),
    //             new Rotation2d(Math.PI / 2)))); // TODO: change these for new robot

    // autoChooser.addOption( // drives 10 ft for odometry testing
    //     "10 foot test", autoCommands.TenFootTest(drive)); // TODO: change these for new robot
  }
}
