package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToReef;
import frc.robot.commands.DriveToReef.ReefDirection;
import frc.robot.subsystems.DeAlgifier.DeAlgifier;
import frc.robot.subsystems.DeAlgifier.DeAlgifierIOReal;
import frc.robot.subsystems.DeAlgifier.DeAlgifierIOSim;
import frc.robot.subsystems.Outtake.Outtake;
import frc.robot.subsystems.Outtake.OuttakeIO;
import frc.robot.subsystems.Outtake.OuttakeIOSim;
import frc.robot.subsystems.Rumbler.Rumbler;
import frc.robot.subsystems.Outtake.OuttakeIOReal;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONAVX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOMaxSwerve;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import static frc.robot.subsystems.vision.VisionConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems

  public final Drive drive;
  public final Elevator elevator;
  private final Outtake outtake;
  @SuppressWarnings("unused")
  private final Vision vision;
  private final DeAlgifier deAlgifier;


  // shuffleboard
  ShuffleboardTab shuffleBoardTab;

  // Controller
  public static CommandXboxController driveController = new CommandXboxController(0);
  public static CommandXboxController operatorController = new CommandXboxController(1);
  public Rumbler driveRumbler = new Rumbler(driveController);
  public Rumbler operatorRumbler = new Rumbler(operatorController);

  // Start position selections
  public static final LoggedTunableNumber startPositionIndex =
      new LoggedTunableNumber("start pos index: 0 or 1", 1.34);
  // Start Pos 0: Along line of the Amp.
  public static final LoggedTunableNumber startX0 =
      new LoggedTunableNumber("Start X0(m)", Units.inchesToMeters(0));
  public static final LoggedTunableNumber startY0 = new LoggedTunableNumber("Start Y0(m)", 6.64);
  public static final LoggedTunableNumber startTheta0 =
      new LoggedTunableNumber("Start Theta0(deg)", 0.0);
  // Start Pos 1: Next to human player side of Speaker.
  public static final LoggedTunableNumber startX1 =
      new LoggedTunableNumber(
          "Start X1(m)", 1.34);
  public static final LoggedTunableNumber startY1 = new LoggedTunableNumber("Start Y1(m)", 6.64);
  public static final LoggedTunableNumber startTheta1 =
      new LoggedTunableNumber("Start Theta1(deg)", 0.0);

  // Auto Commands
  final AutoCommands autoCommands;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getRobot()) {

      case ROBOT_REAL_JANICE:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONAVX(),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        elevator = new Elevator(new ElevatorIOReal());
        outtake = new Outtake(new OuttakeIOReal());
        deAlgifier = new DeAlgifier(new DeAlgifierIOReal() {});
        vision =
             new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1));
        break;
        
      case ROBOT_REAL_FRANKENLEW:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONAVX(),
                new ModuleIOMaxSwerve(0),
                new ModuleIOMaxSwerve(1),
                new ModuleIOMaxSwerve(2),
                new ModuleIOMaxSwerve(3));
        elevator = new Elevator(new ElevatorIOSim());
        outtake = new Outtake(new OuttakeIOSim());
        deAlgifier = new DeAlgifier(new DeAlgifierIOSim() {});
        vision =
              new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1));
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
        elevator = new Elevator(new ElevatorIOSim());
        outtake = new Outtake(new OuttakeIOSim());
        deAlgifier = new DeAlgifier(new DeAlgifierIOSim() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        break;

        case ROBOT_CALIBRATE:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        outtake = new Outtake(new OuttakeIOSim());
        deAlgifier = new DeAlgifier(new DeAlgifierIOSim() {});
        vision =
             new Vision(
              drive::addVisionMeasurement,
              new VisionIOPhotonVision(camera0Name, robotToCamera0),
              new VisionIOPhotonVision(camera1Name, robotToCamera1));
        break;

      case ROBOT_FOOTBALL:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        outtake = new Outtake(new OuttakeIOSim());
        deAlgifier = new DeAlgifier(new DeAlgifierIOSim() {});
        vision =
             new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        break;
      default:
        // Replayed robot, disable IO implementations since the replay
        // will supply the data.
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        outtake = new Outtake(new OuttakeIO() {});
        deAlgifier = new DeAlgifier(new DeAlgifierIOSim() {});
        // (Use same number of dummy implementations as the real robot)
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;

    }

    // Create auto commands
    autoCommands = new AutoCommands(drive, elevator, outtake);

    // initialize the shuffleboard outputs
    initShuffleboard();

    // Configure the button bindings
    configureButtonBindings();
  }

  private void initShuffleboard() {
    // Configure the Shuffleboard
    shuffleBoardTab = Shuffleboard.getTab("Boomerang");
    // this is where display booleans will go
  }

  public void updateShuffleboard() {
    // if (RobotType.ROBOT_REAL == Constants.getRobot()) {
    //   colorInfo.pvCornerOne();
    // }
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

                driveController.povDown().whileTrue( DriveCommands.joystickDrive(
                  drive,
                  () -> -0.3,
                  () -> 0,
                  () -> 0.0,
                  () -> true));
                  
                  driveController.povUp().whileTrue( DriveCommands.joystickDrive(
                    drive,
                    () -> 0.3,
                    () -> 0,
                    () -> 0.0,
                    () -> true));
                  
                    driveController.povLeft().whileTrue( DriveCommands.joystickDrive(
                  drive,
                  () -> 0.0,
                  () -> 0.3,
                  () -> 0.0,
                  () -> true));
            
                  driveController.povRight().whileTrue( DriveCommands.joystickDrive(
                    drive,
                    () -> 0.0,
                    () -> -0.3,
                    () -> 0.0,
                    () -> true));
              
        // drive to closest reef
    driveController.x().whileTrue(new DriveToReef(drive, ReefDirection.LEFT));
    driveController.b().whileTrue(new DriveToReef(drive, ReefDirection.RIGHT));
    driveController.a().whileTrue(new DriveToReef(drive, ReefDirection.DEALGIFY));
    driveController.y().onTrue(new InstantCommand( () -> drive.requestCoast() ));

    operatorController.a().onTrue(elevator.runGoToPosCommand(Constants.ElevatorConstants.kArmL1));
    operatorController.x().onTrue(elevator.runGoToPosCommand(Constants.ElevatorConstants.kArmL3));
    operatorController.b().onTrue(elevator.runGoToPosCommand(Constants.ElevatorConstants.kArmL2));
    operatorController.povUp().onTrue(elevator.runGoToPosCommand(Constants.ElevatorConstants.kAlgaeL3));
    operatorController.povDown().onTrue(elevator.runGoToPosCommand(Constants.ElevatorConstants.kAlgaeL2));

    elevator.setDefaultCommand(new InstantCommand(() -> elevator.runManualPos(operatorController.getLeftY()), elevator));

    outtake.setDefaultCommand(new InstantCommand(() -> outtake.stop(), outtake ) );
    operatorController.leftTrigger(0.8).whileTrue(new RepeatCommand( new InstantCommand( () -> outtake.outtakeTele(), outtake )));

    // //axis 5 is Right Y
    operatorController.axisMagnitudeGreaterThan(5, 0.07).whileTrue( 
      new InstantCommand( () -> deAlgifier.lateratorManual(operatorController.getRightY())).repeatedly());
    operatorController.rightBumper().onTrue(new InstantCommand(() -> deAlgifier.lateratorManual(0.0)));

    operatorController.rightTrigger(0.8).whileTrue(new RepeatCommand(new InstantCommand( () -> deAlgifier.intake() )));
    operatorController.rightTrigger(0.8).whileFalse(new RepeatCommand(new InstantCommand( () -> deAlgifier.holdAlgae() )));

    // rumbler
    driveRumbler.setDefaultCommand(new InstantCommand(() -> driveRumbler.setRumble(RumbleType.kBothRumble, 0), driveRumbler).repeatedly());
    operatorRumbler.setDefaultCommand(new InstantCommand(() -> operatorRumbler.setRumble(RumbleType.kBothRumble, 0), operatorRumbler).repeatedly());

    driveController.b().and(() -> (LimelightHelpers.getTA("limelight") > 0.1))
    .whileTrue(new InstantCommand(() -> driveRumbler.setRumble(RumbleType.kBothRumble, 1), driveRumbler).repeatedly());

    driveController.x().and(() -> (LimelightHelpers.getTA("limelight") > 0.1))
    .whileTrue(new InstantCommand(() -> driveRumbler.setRumble(RumbleType.kBothRumble, 1), driveRumbler).repeatedly());

    driveController.a().and(() -> (LimelightHelpers.getTA("limelight") > 0.1))
    .whileTrue(new InstantCommand(() -> driveRumbler.setRumble(RumbleType.kBothRumble, 1), driveRumbler).repeatedly());

    operatorController.leftTrigger(0.8).onFalse(
      new InstantCommand(() -> driveRumbler.setRumble(RumbleType.kBothRumble, 0.3), driveRumbler).repeatedly().withTimeout(0.3));
    
    driveController.povRight().onFalse(
      new InstantCommand(() -> operatorRumbler.setRumble(RumbleType.kBothRumble, 0.3), driveRumbler).repeatedly().withTimeout(0.3));
    
  }

  // /**
  //  * @return the command to run in autonomous
  //  */
  public Command getAutonomousCommand() {


     try{
        return autoCommands.autoRoutine();
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }
}
