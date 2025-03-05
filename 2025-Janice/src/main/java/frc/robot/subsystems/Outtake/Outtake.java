package frc.robot.subsystems.Outtake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakeIO;
import frc.robot.subsystems.Outtake.OuttakeIO.OuttakeIOInputs;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
    
    private OuttakeIO io;
    private final OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();
    
    public Outtake (OuttakeIO io){
        this.io = io;
    }
    
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Outtake", inputs);
        
        if (DriverStation.isDisabled()) {}
        
    }
    public void outtake() {
        io.setVoltage(-1.0);
    }
    
    public void stop() {
        io.setVoltage(0.0);
    }

    public Command outtakeAuto(){
        if(inputs.hasCoral){
            return new InstantCommand(() -> outtake());
        }else{
            return new InstantCommand(() -> stop());
        }

    }
}