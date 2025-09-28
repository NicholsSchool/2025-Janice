package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.commands.DriveToReef.ReefDirection;
import frc.robot.subsystems.Outtake.Outtake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.util.*;

public class AutoCommands {
  // Subsystems
  private Drive drive;
  private Elevator elevator;
  private Outtake outtake;

  public AutoCommands(Drive drive, Elevator elevator, Outtake outtake) {
    this.drive = drive;
    this.elevator = elevator;
    this.outtake = outtake;
  }

  public Command driveToPose(Pose2d pose) {
    var drvToPose =
        new DriveToPose(
            this.drive,
            () -> {
              // return AllianceFlipUtil.apply(pose);
              return pose;
            });
    return drvToPose.until(drvToPose::atGoal);
  }

  public Command driveToPoseRelative(Pose2d pose) {
    var drvToPose =
        new DriveToPose(
            this.drive,
            () -> {
              return pose;
            });
    return drvToPose.until(drvToPose::atGoal);
  }
  // /**
  //  * this is a drive to pose that rather than it being an actual pose it just chases an object and turns until it's centered
  //  * this would be used for like a drive to algae, but the measurements 
  //  * @return
  //  */
  // public Command weaveToPose(double objectAngle, double objectArea){
  //   var weaveToPos = DriveCommands.joystickDrive(drive, () -> 0.0, () -> objectArea == 0 ? 0.0 : 0.5, () -> objectArea == 0 ? 0.1 : DriveCommands.angleToVelocity(objectAngle, 0.0), () -> true);
  //   return weaveToPos.until(() -> objectArea > VisionConstants.weaveToPoseBreakArea);
  // }

    public Command splineV5ToPose(Supplier<Pose2d> pose, Supplier<Circle> circle, boolean slowmode) {
        var splToPose =
            new SplineV5ToPose(this.drive, () -> {return pose.get();}, () -> {return circle.get();});
        return splToPose.until(splToPose::atGoal);
    }

    public Command autoReefRoutine(IntSupplier reefPosition, IntSupplier coralLevel, BooleanSupplier shortestPath, Supplier<DriveToReef.ReefDirection> reefDirection){
        DoubleSupplier desiredArmHeight = () -> 0.0;
        switch(coralLevel.getAsInt()){
            case 1:
                desiredArmHeight = () -> Constants.ElevatorConstants.kArmL1; 
                break;
            case 2: 
                desiredArmHeight = () -> Constants.ElevatorConstants.kArmL2 - 0.12;
                break;
            case 3:
                desiredArmHeight = () -> Constants.ElevatorConstants.kArmL3 - 0.1;
                break;
            case 4: 
                desiredArmHeight = () -> Constants.ElevatorConstants.kArmL4; 
                break;
    }
    //-π/6 is a multiplier that converts clock angles to radians
    double reefNormalAngle = reefPosition.getAsInt() * -Math.PI / 6;

    Pose2d orbitPose = new Pose2d(new Translation2d(Math.cos(reefNormalAngle) * Constants.AutoConstants.reefAutoRadius + Constants.AutoConstants.reefAutoCircle.getX(), 
     Math.sin(reefNormalAngle) * Constants.AutoConstants.reefAutoRadius + Constants.AutoConstants.reefAutoCircle.getY()), new Rotation2d(reefNormalAngle));

    Command orbitToPose = splineV5ToPose(() -> AllianceFlipUtil.apply(orbitPose).transformBy(new Transform2d(new Translation2d(), new Rotation2d(-Math.PI / 2))),
      () -> new Circle(() -> AllianceFlipUtil.apply(Constants.AutoConstants.reefAutoCircle), () -> Constants.AutoConstants.reefAutoRadius), false);

     return new SequentialCommandGroup(orbitToPose, new WaitCommand(0.4),
     new ParallelCommandGroup(new DriveToReef(drive, reefDirection.get()), elevator.commandGoToPos(desiredArmHeight.getAsDouble())), outtake.commandOuttake());
  }

  public Command autoHumanRoutine(BooleanSupplier topHumanPlayer, BooleanSupplier shortestPath){

    int tagIndex = topHumanPlayer.getAsBoolean() ? 13 : 12;
    Pose2d humanTagPose = FieldConstants.aprilTags.getTagPose(tagIndex).get().toPose2d();
    Pose2d desiredPose = new Pose2d(
      new Translation2d(humanTagPose.getX() + 3 * Constants.RobotConstants.robotGoToPosBuffer * Math.cos(humanTagPose.getRotation().getRadians()),
      humanTagPose.getY() + 3 * Constants.RobotConstants.robotGoToPosBuffer * Math.sin(humanTagPose.getRotation().getRadians())), humanTagPose.getRotation());

    Command orbit = 
      splineV5ToPose(() -> new Pose2d(AllianceFlipUtil.apply((desiredPose.getTranslation())), AllianceFlipUtil.apply(desiredPose.getRotation()).rotateBy(new Rotation2d(Math.PI))),
      () -> new Circle(AllianceFlipUtil.apply(Constants.AutoConstants.reefAutoCircle), Constants.AutoConstants.reefAutoRadius), true);
    Command nudge = DriveCommands.joystickDrive(
      drive,
      () -> 0.5,
      () -> 0,
      () -> 0.0,
      () -> true).repeatedly().withTimeout(0.15);
    return new SequentialCommandGroup(
      new ParallelCommandGroup(orbit, elevator.commandGoToPos(Constants.ElevatorConstants.kArmL1)),
      new DriveToHumanPlayer(drive),nudge, new WaitCommand(1.5)
    );
  }

  public Command autoRoutine(){
    return new SequentialCommandGroup(
      autoReefRoutine(() -> 10, () -> 2, () -> true, () -> ReefDirection.LEFT),
      autoHumanRoutine(() -> true, () -> false),
      autoReefRoutine(() -> 6, () -> 2, () -> true, () -> ReefDirection.LEFT),
       autoHumanRoutine(() -> true, () -> false));
  }

  //return autoReefRoutine(() -> 12, () -> 3, () -> true, ReefDirection.LEFT);

  public Command TenFootTest(Drive drive) {
    return new DriveToPose(drive, new Pose2d(new Translation2d(3.048, 0), new Rotation2d(0)));
  }

  
  // /**
  //  * code for testing localization
  //  * @param objectAngle angle algae is from 
  //  * @param objectArea
  //  * @return
  //  */
  // public Command guardianOfTheReef(PhotonTrackedTarget target){
  //    return new SequentialCommandGroup(driveToPose(new Pose2d(new Translation2d(15, 4), new Rotation2d())));

  // }
}