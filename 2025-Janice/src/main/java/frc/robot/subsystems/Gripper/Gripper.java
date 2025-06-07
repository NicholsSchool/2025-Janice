package frc.robot.subsystems.Gripper;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase{
    
    private GripperIO io;
    private final GripperIOInputsAutoLogged inputs = new GripperIOInputsAutoLogged();
    private double voltage = 0.0;
    private GripperMode gripperMode;

    enum GripperMode{
        OUTTAKE,
        INTAKE,
        STOP,
        IDLE
    }

    public Gripper(GripperIO io){
        this.io = io;

        gripperMode = GripperMode.IDLE;
    }

    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Gripper", inputs);
        if (DriverStation.isDisabled()) {}

        switch(gripperMode){
            case OUTTAKE -> {
                voltage = 1.5;
                io.setVoltage(voltage);
            }
            case INTAKE -> {
                voltage = -3.0;
                io.setVoltage(voltage);
            }
            case STOP -> {
                voltage = 0.0;
                io.setVoltage(voltage);
            }
            case IDLE -> {
                voltage = 0.0;
                io.setVoltage(voltage);
            }
            default -> {
                voltage = 0.0;
                io.setVoltage(voltage);
            }
        }
           
        }
    

    public void intake(){
        gripperMode = GripperMode.INTAKE;
    }

    public void outtake(){
        gripperMode = GripperMode.OUTTAKE;
    }

    public void stop() {
        gripperMode = GripperMode.STOP;
    }

    public void idle() {
        gripperMode = GripperMode.IDLE;
    }

}
