// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier robotRelative) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
                  Constants.JOYSTICK_DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega =
              MathUtil.applyDeadband(omegaSupplier.getAsDouble(), Constants.JOYSTICK_DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // If commanding field-relative then adjust for alliance and
          // convert to robot speeds.
          // Otherwise, use robot speeds directly.
          ChassisSpeeds chassisSpeeds;
          if (robotRelative.getAsBoolean()) {
            chassisSpeeds =
                new ChassisSpeeds(
                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                    omega * drive.getMaxAngularSpeedRadPerSec());
          } else {
            if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
              linearVelocity = linearVelocity.rotateBy(Rotation2d.fromRadians(Math.PI));
            }
            chassisSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                    omega * drive.getMaxAngularSpeedRadPerSec(),
                    drive.getRotation());
          }
          drive.runVelocity(chassisSpeeds);
        },
        drive);
  }

  public static Command joystickDriveWithAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      // TODO change back to angle supplier
      DoubleSupplier desiredAngle,
      DoubleSupplier robotYawSupplier,
      BooleanSupplier robotRelative) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
                  Constants.JOYSTICK_DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // if driving field-relative, then check which alliance to swap
          // linear direction.
          if (!robotRelative.getAsBoolean()) {
            if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
              linearVelocity = linearVelocity.rotateBy(Rotation2d.fromRadians(Math.PI));
            }
          }
          double angularRotation =
              angleToVelocity(desiredAngle.getAsDouble(), robotYawSupplier.getAsDouble());
          // Convert to field relative speeds & send command
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  angularRotation,
                  drive.getRotation()));
        },
        drive);
  }

  public static Command joystickDriveFacingPoint(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Translation2d> facingPose,
      DoubleSupplier robotYawSupplier,
      DoubleSupplier angleOffset, 
      BooleanSupplier robotRelative) {
    return Commands.run(
        () -> {
          // Apply deadband
          Translation2d flippedFacingPose = AllianceFlipUtil.apply(facingPose.get());
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
                  Constants.JOYSTICK_DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // get desired angle
          Pose2d robotPose = drive.getPose();
          double desiredAngle =
              Math.atan2(
                  flippedFacingPose.getY() - robotPose.getY(),
                  AllianceFlipUtil.apply(facingPose.get().getX()) - robotPose.getX()) + angleOffset.getAsDouble();

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();
          
                  if (!robotRelative.getAsBoolean()) {
                    if (DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                      linearVelocity = linearVelocity.rotateBy(Rotation2d.fromRadians(Math.PI));
                    }
                  }

          double angularRotation =
              angleToVelocity(desiredAngle * 180 / Math.PI, robotYawSupplier.getAsDouble());
          // Convert to field relative speeds & send command
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  angularRotation,
                  drive.getRotation()));
        },
        drive);
  }

  public static double angleToVelocity(double desiredAngle, double robotYaw) {
    double currentYaw = robotYaw;
    double difference = desiredAngle - Math.toDegrees(currentYaw);
    double error;
    double kP = 8.0;

    if (Math.abs(difference) > 180) {
      error = difference - (360 * (Math.abs(difference) / difference));
    } else {
      error = difference;
    }

    return kP * Math.sin(0.5 * Math.toRadians(error));
  }

}