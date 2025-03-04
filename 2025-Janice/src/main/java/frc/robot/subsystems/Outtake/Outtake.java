package frc.robot.subsystems.Outtake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakeIO;
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
}