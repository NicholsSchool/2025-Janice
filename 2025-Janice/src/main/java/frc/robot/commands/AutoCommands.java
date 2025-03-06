package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Constants.VisionConstants;
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

  public Command splineV5ToPose(Supplier<Pose2d> pose, Supplier<Circle> circle) {
    var splToPose =
        new SplineV5ToPose(
            this.drive,() -> {return pose.get();}, circle.get());
    return splToPose.until(splToPose::atGoal);
  }

  public Command autoReefRoutine(IntSupplier reefPosition, IntSupplier coralLevel, BooleanSupplier shortestPath, Supplier<DriveToReef.ReefDirection> reefDirection){
    DoubleSupplier desiredArmHeight = () -> 0.0;
    switch(coralLevel.getAsInt()){
      case 1:
      desiredArmHeight = () -> Constants.ElevatorConstants.kArmL1; 
      case 2: 
      desiredArmHeight = () -> Constants.ElevatorConstants.kArmL2; 
      case 3:
      desiredArmHeight = () -> Constants.ElevatorConstants.kArmL3; 
      case 4: 
      desiredArmHeight = () -> Constants.ElevatorConstants.kArmL4; 
    }
    //-π/6 is a multiplier that converts clock angles to radians
    double reefNormalAngle = reefPosition.getAsInt() * -Math.PI / 6;

    Command elevatorAndPose = new ParallelCommandGroup(elevator.runGoToPosCommand(desiredArmHeight.getAsDouble()),
     splineV5ToPose(() -> AllianceFlipUtil.apply(new Pose2d(new Translation2d(Math.cos(reefNormalAngle) * Constants.AutoConstants.reefAutoRadius + Constants.AutoConstants.reefAutoCircle.getX(), 
     Math.sin(reefNormalAngle) * Constants.AutoConstants.reefAutoRadius + Constants.AutoConstants.reefAutoCircle.getY()), new Rotation2d(reefNormalAngle + Math.PI / 2)))
     , () -> new Circle(() -> AllianceFlipUtil.apply(Constants.AutoConstants.reefAutoCircle), () -> Constants.AutoConstants.reefAutoRadius)));

    return new SequentialCommandGroup(elevatorAndPose, new DriveToReef(drive, reefDirection.get()));
  }

  public Command autoHumanRoutine(BooleanSupplier topHumanPlayer, BooleanSupplier shortestPath){

    int tagIndex = topHumanPlayer.getAsBoolean() ? 13 : 12;
    Pose2d humanTagPose = FieldConstants.aprilTags.getTagPose(tagIndex).get().toPose2d();
    humanTagPose.plus(new Transform2d(new Translation2d(Constants.RobotConstants.robotGoToPosBuffer * Math.cos(humanTagPose.getRotation().getRadians()),
    Constants.RobotConstants.robotGoToPosBuffer * Math.sin(humanTagPose.getRotation().getRadians())), new Rotation2d()));

    Command orbit = 
     splineV5ToPose(() -> AllianceFlipUtil.apply(humanTagPose),
      () -> new Circle(AllianceFlipUtil.apply(Constants.AutoConstants.reefAutoCircle), Constants.AutoConstants.reefAutoRadius));

    return orbit;
  }

  public Command autoRoutine(){
    return new SequentialCommandGroup(autoReefRoutine(() -> 12, () -> 3, () -> true, () -> ReefDirection.LEFT), autoHumanRoutine(() -> true, () -> true));
  }

  

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
