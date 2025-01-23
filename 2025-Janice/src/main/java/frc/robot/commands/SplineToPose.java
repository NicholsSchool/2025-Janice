// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// TJG
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SplineMath;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SplineToPose extends Command {
  private final Drive drive;
  private final boolean slowMode;
  private final Supplier<Pose2d> poseSupplier;
  private SplineMath spline;

  private boolean running = false;
  private final ProfiledPIDController splineController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);
  private final ProfiledPIDController splineThetaController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);
  private double splineErrorAbs;
  private double splineThetaErrorAbs;
  private Translation2d lastSetpointTranslation;

  private static final LoggedTunableNumber splineKp =
      new LoggedTunableNumber("SplineToPose/DriveKp");
  private static final LoggedTunableNumber splineKd =
      new LoggedTunableNumber("SplineToPose/DriveKd");
  private static final LoggedTunableNumber splineThetaKp =
      new LoggedTunableNumber("SplineToPose/ThetaKp");
  private static final LoggedTunableNumber splineThetaKd =
      new LoggedTunableNumber("SplineToPose/ThetaKd");
  private static final LoggedTunableNumber splineMaxVelocity =
      new LoggedTunableNumber("SplineToPose/DriveMaxVelocity");
  private static final LoggedTunableNumber splineMaxVelocitySlow =
      new LoggedTunableNumber("SplineToPose/DriveMaxVelocitySlow");
  private static final LoggedTunableNumber splineMaxAcceleration =
      new LoggedTunableNumber("SplineToPose/DriveMaxAcceleration");
  private static final LoggedTunableNumber splineThetaMaxVelocity =
      new LoggedTunableNumber("SplineToPose/ThetaMaxVelocity");
  private static final LoggedTunableNumber splineThetaMaxVelocitySlow =
      new LoggedTunableNumber("SplineToPose/ThetaMaxVelocitySlow");
  private static final LoggedTunableNumber splineThetaMaxAcceleration =
      new LoggedTunableNumber("SplineToPose/ThetaMaxAcceleration");
  private static final LoggedTunableNumber splineTolerance =
      new LoggedTunableNumber("SplineToPose/DriveTolerance");
  private static final LoggedTunableNumber splineToleranceSlow =
      new LoggedTunableNumber("splineThetaToPose/DriveToleranceSlow");
  private static final LoggedTunableNumber splineThetaTolerance =
      new LoggedTunableNumber("SplineToPose/ThetaTolerance");
  private static final LoggedTunableNumber splineThetaToleranceSlow =
      new LoggedTunableNumber("SplineToPose/ThetaToleranceSlow");
  private static final LoggedTunableNumber ffMinRadius =
      new LoggedTunableNumber("SplineToPose/FFMinRadius");
  private static final LoggedTunableNumber ffMaxRadius =
      new LoggedTunableNumber("SplineToPose/FFMinRadius");

  static {
    switch (Constants.getRobot()) {
      case ROBOT_FOOTBALL:
      case ROBOT_REAL:
      case ROBOT_REPLAY:
      case ROBOT_SIM:
        splineKp.initDefault(2.0);
        splineKd.initDefault(0.0);
        splineThetaKp.initDefault(2.0);
        splineThetaKd.initDefault(0.0);
        splineMaxVelocity.initDefault(Units.inchesToMeters(150.0));
        splineMaxVelocitySlow.initDefault(Units.inchesToMeters(50.0));
        splineMaxAcceleration.initDefault(Units.inchesToMeters(95.0));
        splineThetaMaxVelocity.initDefault(Units.degreesToRadians(360.0));
        splineThetaMaxVelocitySlow.initDefault(Units.degreesToRadians(90.0));
        splineThetaMaxAcceleration.initDefault(Units.degreesToRadians(720.0));
        splineTolerance.initDefault(0.01);
        splineToleranceSlow.initDefault(0.06);
        splineThetaTolerance.initDefault(Units.degreesToRadians(1.0));
        splineThetaToleranceSlow.initDefault(Units.degreesToRadians(3.0));
        ffMinRadius.initDefault(0.2);
        ffMaxRadius.initDefault(0.8);
      default:
        break;
    }
  }

  /** Drives to the specified pose under full software control. */
  public SplineToPose(Drive drive, Pose2d pose) {
    this(drive, false, pose);
  }

  /** Drives to the specified pose under full software control. */
  public SplineToPose(Drive drive, boolean slowMode, Pose2d pose) {
    this(drive, slowMode, () -> pose);
  }

  /** Drives to the specified pose under full software control. */
  public SplineToPose(Drive drive, Supplier<Pose2d> poseSupplier) {
    this(drive, false, poseSupplier);
  }

  /** Drives to the specified pose under full software control. */
  public SplineToPose(Drive drive, boolean slowMode, Supplier<Pose2d> poseSupplier) {
    this.drive = drive;
    this.slowMode = slowMode;
    this.poseSupplier = poseSupplier;
    addRequirements(drive);
    splineThetaController.enableContinuousInput(-Math.PI, Math.PI);
    // spline :)
    spline =
        new SplineMath(
            poseSupplier.get().getTranslation(),
            poseSupplier.get().getRotation().getRadians(),
            drive.getPose());
  }

  @Override
  public void initialize() {
    // Reset all controllers
    var currentPose = drive.getPose();
    splineController.reset(
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()),
        Math.min(
            0.0,
            -new Translation2d(drive.getFieldVelocity().dx, drive.getFieldVelocity().dy)
                .rotateBy(
                    poseSupplier
                        .get()
                        .getTranslation()
                        .minus(drive.getPose().getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    splineThetaController.reset(currentPose.getRotation().getRadians(), drive.getYawVelocity());
    lastSetpointTranslation = drive.getPose().getTranslation();
  }

  @Override
  public void execute() {
    running = true;

    // Update from tunable numbers
    if (splineMaxVelocity.hasChanged(hashCode())
        || splineMaxVelocitySlow.hasChanged(hashCode())
        || splineMaxAcceleration.hasChanged(hashCode())
        || splineTolerance.hasChanged(hashCode())
        || splineToleranceSlow.hasChanged(hashCode())
        || splineThetaMaxVelocity.hasChanged(hashCode())
        || splineThetaMaxVelocitySlow.hasChanged(hashCode())
        || splineThetaMaxAcceleration.hasChanged(hashCode())
        || splineThetaTolerance.hasChanged(hashCode())
        || splineThetaToleranceSlow.hasChanged(hashCode())
        || splineKp.hasChanged(hashCode())
        || splineKd.hasChanged(hashCode())
        || splineThetaKp.hasChanged(hashCode())
        || splineThetaKd.hasChanged(hashCode())) {
      splineController.setP(splineKp.get());
      splineController.setD(splineKd.get());
      splineController.setConstraints(
          new TrapezoidProfile.Constraints(
              slowMode ? splineMaxVelocitySlow.get() : splineMaxVelocity.get(),
              splineMaxAcceleration.get()));
      splineController.setTolerance(slowMode ? splineToleranceSlow.get() : splineTolerance.get());
      splineThetaController.setP(splineThetaKp.get());
      splineThetaController.setD(splineThetaKd.get());
      splineThetaController.setConstraints(
          new TrapezoidProfile.Constraints(
              slowMode ? splineThetaMaxVelocitySlow.get() : splineThetaMaxVelocity.get(),
              splineThetaMaxAcceleration.get()));
      splineThetaController.setTolerance(
          slowMode ? splineThetaToleranceSlow.get() : splineThetaTolerance.get());
    }

    // Get current and target pose
    var currentPose = drive.getPose();
    var targetPose = poseSupplier.get();
    spline.update(currentPose);

    // Calculate drive speed
    double currentDistance =
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation());
    double ffScaler =
        MathUtil.clamp(
            (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
            0.0,
            1.0);
    splineErrorAbs = currentDistance;
    splineController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        splineController.getSetpoint().velocity);
    double splineVelocityScalar =
        splineController.getSetpoint().velocity * ffScaler
            - splineController.calculate(splineErrorAbs, 0.0);
    if (currentDistance < splineController.getPositionTolerance()) splineVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(
                GeomUtil.translationToTransform(splineController.getSetpoint().position, 0.0))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        splineThetaController.getSetpoint().velocity * ffScaler
            + splineThetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    splineThetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (splineThetaErrorAbs < splineThetaController.getPositionTolerance()) thetaVelocity = 0.0;

    // Command speeds
    var driveVelocity =
        new Pose2d(new Translation2d(), spline.driveVector().getAngle())
            .transformBy(GeomUtil.translationToTransform(splineVelocityScalar, 0.0))
            .getTranslation();
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));

    // Log data
    Logger.recordOutput("SplineToPose/DistanceMeasured", currentDistance);
    Logger.recordOutput("SplineToPose/DistanceSetpoint", splineController.getSetpoint().position);
    Logger.recordOutput("SplineToPose/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("SplineToPose/ThetaSetpoint", splineThetaController.getSetpoint().position);
    Logger.recordOutput(
        "Odometry/SplineToPoseSetpoint",
        new Pose2d(
            lastSetpointTranslation, new Rotation2d(splineThetaController.getSetpoint().position)));
    Logger.recordOutput("Odometry/SplineToPoseGoal", targetPose);
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    drive.stop();
    Logger.recordOutput("Odometry/SplineToPoseSetpoint", new Pose2d());
    Logger.recordOutput("Odometry/SplineToPoseGoal", new Pose2d());
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return running && splineController.atGoal() && splineThetaController.atGoal();
  }

  /**
   * Checks if the robot pose is within the allowed drive and theta tolerances.
   *
   * <p>//TODO: @tom is this in meters
   *
   * @param driveTolerance the finish distance for drive in meters
   * @param thetaTolerance the angle threashold
   */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(splineErrorAbs) < driveTolerance
        && Math.abs(splineErrorAbs) < thetaTolerance.getRadians();
  }

  /** Returns whether the command is actively running. */
  public boolean isRunning() {
    return running;
  }

  public boolean isFinished() {
    running =
        this.withinTolerance(
            Constants.AutoConstants.driveFinishThreshold,
            new Rotation2d(Constants.AutoConstants.angleFinishThreshold));
    return running;
  }
}
