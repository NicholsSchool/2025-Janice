package frc.robot.subsystems.Outtake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

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
        io.setVoltage(-2.0);
    }
    
    public void stop() {
        io.setVoltage(0.0);
    }

    public void processCoral() {
        if( inputs.hasCoral )
            this.stop();
        else io.setVoltage(-0.4);
    }

    public BooleanSupplier hasCoral(){
        return () -> inputs.hasCoral;
    }


    public Command commandOuttake() {   
        return new FunctionalCommand(
            () -> System.out.println("Outtaking"),
            () -> outtake(),
            interrupted -> stop(),
            () -> !inputs.hasCoral,
            this);
    }    
}