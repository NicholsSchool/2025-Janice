package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveToReef.ReefDirection;
import frc.robot.subsystems.Outtake.Outtake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;


public class DriveAndScore extends SequentialCommandGroup {
    public DriveAndScore( Drive drive, Elevator elevator, Outtake outtake, ReefDirection reefDirection, boolean isL3Score ) {
        double scoreSetpoint = isL3Score ? Constants.ElevatorConstants.kArmL3 : Constants.ElevatorConstants.kArmL2;
        this.addRequirements(drive);
        addCommands(
            new DriveToReef(drive, reefDirection),
            elevator.runGoToPosCommand(scoreSetpoint - 0.1),
            new RepeatCommand( new InstantCommand( () -> outtake.outtakeTele(), outtake )
                .onlyIf(() -> Math.abs(elevator.getHeight() - scoreSetpoint - 0.1 ) < 0.03 ) )
                .finallyDo(() -> elevator.setTargetPos(Constants.ElevatorConstants.kArmL1))
            );
    }
}
