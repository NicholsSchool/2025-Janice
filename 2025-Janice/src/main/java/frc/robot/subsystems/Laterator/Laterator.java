package frc.robot.subsystems.Laterator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;

public class Laterator extends SubsystemBase{
    
    private LateratorIO io;
    private final LateratorIOInputsAutoLogged inputs = new LateratorIOInputsAutoLogged();

    private double voltage = 0.0;
    private boolean isManual = true;
    private double lateralManual = 0.0;
    private LateratorMode lateratorMode;

    enum LateratorMode{
        OUTTAKE,
        INTAKE,
        STOP,
        IDLE
    }

    public Laterator(LateratorIO io){
        this.io = io;

        lateratorMode = LateratorMode.IDLE;
    }

    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Laterator", inputs);
        if (DriverStation.isDisabled()) {}

        if(isManual){
            io.setVoltage(lateralManual * 3);
        }else{
            voltage = 0.0;
            io.setVoltage(voltage); 
        }

        switch(lateratorMode){
            case OUTTAKE -> {
                if (inputs.limitSwitch && voltage > 0) {
                    voltage = 0.0;
                }
                else {
                    voltage = 3.0;
                }
            }
            case INTAKE -> {
                voltage = -3.0;
            }
            case STOP, IDLE -> {
                voltage = 0.0;
            }
            default -> {
                voltage = 0.0;
            }
        }
            io.setVoltage(voltage);
        }
    

    public void intake(){
        lateratorMode = LateratorMode.INTAKE;
    }

    public void outtake(){
        lateratorMode = LateratorMode.OUTTAKE;
    }

    public void stop() {
        lateratorMode = LateratorMode.STOP;
    }

    public void idle() {
        lateratorMode = LateratorMode.IDLE;
    }
}

