package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorToPos extends Command{
    Elevator elevator;
    double desiredPos;
    
    public ElevatorToPos(Elevator elevator, double targetPos) {
        this.elevator = elevator;
        this.desiredPos = targetPos;
        addRequirements(elevator);
      }
    
      // Called when the command is initially scheduled.
      @Override
      public void initialize() {
        elevator.runGoToPosCommand(desiredPos);
      }
    
      @Override
      public boolean isFinished() {
        return elevator.isAtGoal();
      }
    }
