package frc.robot.subsystems.DeAlgifier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeAlgifierConstants;

import org.littletonrobotics.junction.Logger;

public class DeAlgifier extends SubsystemBase{
    private DeAlgifierIO io;
    private final DeAlgifierIOInputsAutoLogged inputs = new DeAlgifierIOInputsAutoLogged();

    private final PIDController armPidController;
    private final PIDController kickerPidController;
    
    public DeAlgifier(DeAlgifierIO io){
        this.io = io;
        this.armPidController = new PIDController(DeAlgifierConstants.ARM_P, 0, DeAlgifierConstants.ARM_D);
        this.kickerPidController = new PIDController(DeAlgifierConstants.KICKER_P, 0, DeAlgifierConstants.KICKER_D);
        armPidController.reset();
        kickerPidController.reset();
    }
    
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("DeAlgifier", inputs);
        
        if (DriverStation.isDisabled()) {
             io.setArmVoltage(0.0);
             io.setKickerVoltage(0.0);
        } 
        else {
            io.setArmVoltage(armPidController.calculate(inputs.armPositionRad));
            io.setKickerVoltage( kickerPidController.getSetpoint() == 0.0 ? 0.0 : -5.0 );
            System.out.println(kickerPidController.getSetpoint());
        }
    }

    public void deAlgify() {
        armPidController.setSetpoint(DeAlgifierConstants.kArmAlgaeSetpointRad);
        kickerPidController.setSetpoint(DeAlgifierConstants.kKickerSetpointRPM);
        System.out.println("Dealgifying");
    }

    public void resetToZero() {
        armPidController.setSetpoint(0.0);
        kickerPidController.setSetpoint(0.0);
    }
    
    public void stop() {
        armPidController.setSetpoint(inputs.armPositionRad);
        kickerPidController.setSetpoint(0.0);
    }
}