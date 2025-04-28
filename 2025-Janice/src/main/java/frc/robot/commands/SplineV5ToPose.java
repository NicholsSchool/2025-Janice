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
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.BradyMathLib;
import frc.robot.util.Circle;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SplineV5ToPose extends Command {
  private final Drive drive;
  private final boolean slowMode;
  private final Supplier<Pose2d> poseSupplier;
  private Supplier<Circle> avoidanceCircleSupplier;
  private Circle avoidanceCircle;

  private boolean running = false;
  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);
  private double driveErrorAbs;
  private double thetaErrorAbs;
  private Translation2d lastSetpointTranslation;

  private static final LoggedTunableNumber driveKp = new LoggedTunableNumber("SplineV5ToPose/DriveKp");
  private static final LoggedTunableNumber driveKd = new LoggedTunableNumber("SplineV5ToPose/DriveKd");
  private static final LoggedTunableNumber thetaKp = new LoggedTunableNumber("SplineV5ToPose/ThetaKp");
  private static final LoggedTunableNumber thetaKd = new LoggedTunableNumber("SplineV5ToPose/ThetaKd");
  private static final LoggedTunableNumber driveMaxVelocity =
      new LoggedTunableNumber("SplineV5ToPose/DriveMaxVelocity");
  private static final LoggedTunableNumber driveMaxVelocitySlow =
      new LoggedTunableNumber("SplineV5ToPose/DriveMaxVelocitySlow");
  private static final LoggedTunableNumber driveMaxAcceleration =
      new LoggedTunableNumber("SplineV5ToPose/DriveMaxAcceleration");
  private static final LoggedTunableNumber thetaMaxVelocity =
      new LoggedTunableNumber("SplineV5ToPose/ThetaMaxVelocity");
  private static final LoggedTunableNumber thetaMaxVelocitySlow =
      new LoggedTunableNumber("SplineV5ToPose/ThetaMaxVelocitySlow");
  private static final LoggedTunableNumber thetaMaxAcceleration =
      new LoggedTunableNumber("SplineV5ToPose/ThetaMaxAcceleration");
  private static final LoggedTunableNumber driveTolerance =
      new LoggedTunableNumber("SplineV5ToPose/DriveTolerance");
  private static final LoggedTunableNumber driveToleranceSlow =
      new LoggedTunableNumber("SplineV5ToPose/DriveToleranceSlow");
  private static final LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber("SplineV5ToPose/ThetaTolerance");
  private static final LoggedTunableNumber thetaToleranceSlow =
      new LoggedTunableNumber("SplineV5ToPose/ThetaToleranceSlow");
  private static final LoggedTunableNumber ffMinRadius =
      new LoggedTunableNumber("SplineV5ToPose/FFMinRadius");
  private static final LoggedTunableNumber ffMaxRadius =
      new LoggedTunableNumber("SplineV5ToPose/FFMinRadius");

  static {
    switch (Constants.getRobot()) {
      case ROBOT_FOOTBALL:
      case ROBOT_REAL_FRANKENLEW:
      case ROBOT_REAL_JANICE:
      case ROBOT_REPLAY:
      case ROBOT_SIM:
        driveKp.initDefault(2.0);
        driveKd.initDefault(0.0);
        thetaKp.initDefault(5.0);
        thetaKd.initDefault(0.0);
        driveMaxVelocity.initDefault(Units.inchesToMeters(150.0));
        driveMaxVelocitySlow.initDefault(Units.inchesToMeters(50.0));
        driveMaxAcceleration.initDefault(Units.inchesToMeters(95.0));
        thetaMaxVelocity.initDefault(Units.degreesToRadians(360.0));
        thetaMaxVelocitySlow.initDefault(Units.degreesToRadians(90.0));
        thetaMaxAcceleration.initDefault(Units.degreesToRadians(720.0));
        driveTolerance.initDefault(0.01);
        driveToleranceSlow.initDefault(0.06);
        thetaTolerance.initDefault(Units.degreesToRadians(1.0));
        thetaToleranceSlow.initDefault(Units.degreesToRadians(3.0));
        ffMinRadius.initDefault(0.2);
        ffMaxRadius.initDefault(0.8);
      default:
        break;
    }
  }

  /** Drives to the specified pose under full software control. */
  public SplineV5ToPose(Drive drive, Pose2d pose, Circle circle) {
    this(drive, false, pose, circle);
  }

  /** Drives to the specified pose under full software control. */
  public SplineV5ToPose(Drive drive, boolean slowMode, Pose2d pose, Circle circle) {
    this(drive, slowMode, () -> pose, () -> circle);
  }

  /** Drives to the specified pose under full software control. */
  public SplineV5ToPose(Drive drive, Supplier<Pose2d> poseSupplier, Supplier<Circle> circleSupplier) {
    this(drive, false, poseSupplier, circleSupplier);
  }

  /** Drives to the specified pose under full software control. */
  public SplineV5ToPose(Drive drive, boolean slowMode, Supplier<Pose2d> poseSupplier, Supplier<Circle> circleSupplier) {
    this.drive = drive;
    this.slowMode = slowMode;
    this.poseSupplier = poseSupplier;
    addRequirements(drive);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    this.avoidanceCircleSupplier = circleSupplier;
  }

  @Override
  public void initialize() {
    // Reset all controllers
    var currentPose = drive.getPose();
    driveController.reset(
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
    thetaController.reset(currentPose.getRotation().getRadians(), drive.getYawVelocity());
    lastSetpointTranslation = drive.getPose().getTranslation();
  }

  @Override
  public void execute() {
    running = true;

    // Update from tunable numbers
    if (driveMaxVelocity.hasChanged(hashCode())
        || driveMaxVelocitySlow.hasChanged(hashCode())
        || driveMaxAcceleration.hasChanged(hashCode())
        || driveTolerance.hasChanged(hashCode())
        || driveToleranceSlow.hasChanged(hashCode())
        || thetaMaxVelocity.hasChanged(hashCode())
        || thetaMaxVelocitySlow.hasChanged(hashCode())
        || thetaMaxAcceleration.hasChanged(hashCode())
        || thetaTolerance.hasChanged(hashCode())
        || thetaToleranceSlow.hasChanged(hashCode())
        || driveKp.hasChanged(hashCode())
        || driveKd.hasChanged(hashCode())
        || thetaKp.hasChanged(hashCode())
        || thetaKd.hasChanged(hashCode())) {
      driveController.setP(driveKp.get());
      driveController.setD(driveKd.get());
      driveController.setConstraints(
          new TrapezoidProfile.Constraints(
              slowMode ? driveMaxVelocitySlow.get() : driveMaxVelocity.get(),
              driveMaxAcceleration.get()));
      driveController.setTolerance(slowMode ? driveToleranceSlow.get() : driveTolerance.get());
      thetaController.setP(thetaKp.get());
      thetaController.setD(thetaKd.get());
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(
              slowMode ? thetaMaxVelocitySlow.get() : thetaMaxVelocity.get(),
              thetaMaxAcceleration.get()));
      thetaController.setTolerance(slowMode ? thetaToleranceSlow.get() : thetaTolerance.get());
    }

    // Get current and target pose
    var currentPose = drive.getPose();
    var targetPose = poseSupplier.get();
    avoidanceCircle = avoidanceCircleSupplier.get();

    // Calculate drive speed
    double currentDistance =
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation());
    double ffScaler =
        MathUtil.clamp(
            (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
            0.0,
            1.0);
    driveErrorAbs = currentDistance;
    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(driveErrorAbs, 0.0);
    if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(
                GeomUtil.translationToTransform(driveController.getSetpoint().position, 0.0))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;
    // Command speeds
    var driveLinearVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0))
            .getTranslation();
    

    double angleBetween = Math.atan2(poseSupplier.get().getY() - avoidanceCircle.y, poseSupplier.get().getX() - avoidanceCircle.x)
    - Math.atan2(currentPose.getY() - avoidanceCircle.y, currentPose.getX() - avoidanceCircle.x);
    
    double distFromCircle = Math.hypot(currentPose.getX() - avoidanceCircle.x, currentPose.getY() - avoidanceCircle.y);

    var driveCircularVelocity = new Pose2d(new Translation2d(-(currentPose.getY() - avoidanceCircle.y),(currentPose.getX() - avoidanceCircle.x)),
     new Rotation2d()).times(Math.cos(angleBetween) < 0 ? Math.signum(Math.sin(angleBetween)) : BradyMathLib.clip(2 * Math.sin(angleBetween), -1.0, 1.0)); 

    var driveOutwardVelocity = new Pose2d(new Translation2d((currentPose.getX() - avoidanceCircle.x),(currentPose.getY() - avoidanceCircle.y)), new Rotation2d()).times(BradyMathLib.clip(Math.pow(5.0, avoidanceCircle.radius - distFromCircle) - 1.0, 0.0, 4.0));
    
    if((currentPose.getX() - avoidanceCircle.x) * driveLinearVelocity.getX() + (currentPose.getY() - avoidanceCircle.y) * driveLinearVelocity.getY() > Constants.AutoConstants.dotProductThreshold){
      drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds( 
        driveLinearVelocity.getX() * 
        AutoConstants.SplineV5LinearMultiplier + driveOutwardVelocity.getX(),
       driveLinearVelocity.getY() * 
        AutoConstants.SplineV5LinearMultiplier + driveOutwardVelocity.getY()
         , thetaVelocity, currentPose.getRotation()));
    }else{
    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds((
        AutoConstants.SplineV5CircularMultiplier * driveCircularVelocity.getX() * BradyMathLib.clip(1 - distFromCircle + avoidanceCircle.radius, 0.0, 1.0)+  
        driveLinearVelocity.getX() * 
        AutoConstants.SplineV5LinearMultiplier * (BradyMathLib.clip(distFromCircle - avoidanceCircle.radius, -0.3, 1.0)) + driveOutwardVelocity.getX()),
        (AutoConstants.SplineV5CircularMultiplier * driveCircularVelocity.getY() * BradyMathLib.clip(1 - distFromCircle + avoidanceCircle.radius, 0.0, 1.0)
        + driveLinearVelocity.getY() * 
        AutoConstants.SplineV5LinearMultiplier * (BradyMathLib.clip(distFromCircle - avoidanceCircle.radius, -0.3, 1.0)) + driveOutwardVelocity.getY())
         , thetaVelocity, currentPose.getRotation()));
    }
    // Log data
    Logger.recordOutput("SplineV5ToPose/DistanceMeasured", currentDistance);
    Logger.recordOutput("SplineV5ToPose/DistanceSetpoint", driveController.getSetpoint().position);
    Logger.recordOutput("SplineV5ToPose/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("SplineV5ToPose/ThetaSetpoint", thetaController.getSetpoint().position);
    Logger.recordOutput(
        "Odometry/SplineV5ToPoseSetpoint",
        new Pose2d(
            lastSetpointTranslation, new Rotation2d(thetaController.getSetpoint().position)));
    Logger.recordOutput("Odometry/SplineV5ToPoseGoal", targetPose);
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    drive.stop();
    Logger.recordOutput("Odometry/SplineV5ToPoseSetpoint", new Pose2d());
    Logger.recordOutput("Odometry/SplineV5ToPoseGoal", new Pose2d());
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }

  /**
   * Checks if the robot pose is within the allowed drive and theta tolerances.
   *
   * @param driveTolerance the finish distance for drive in meters
   * @param thetaTolerance the angle threashold
   */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }

  public void updateCircle(Circle circle){
    this.avoidanceCircle = circle;
  }

  /** Returns whether the command is actively running. */
  public boolean isRunning() {
    return running;
  }

  public boolean isFinished() {
    running =
        this.withinTolerance(
            Constants.AutoConstants.splineFinishThreshold,
            new Rotation2d(Constants.AutoConstants.splineAngleFinishThreshold));
    return running;
  }

  //TODO: do this as a drive to pos interface like drive to reef
}