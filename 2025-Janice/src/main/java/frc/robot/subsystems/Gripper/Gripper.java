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

    public Gripper(GripperIO io){
        this.io = io;
    }

    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Gripper", inputs);
        if (DriverStation.isDisabled()) {}
    }

    public void intake(){
        io.setVoltage(-3.0)
    }

    public void outtake(){
        io.setVoltage(1.5)
    }

    public void stop() {
        io.setVoltage(0.0);
    }


}
