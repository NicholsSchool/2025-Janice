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

    private static double kickerSetpointRPM = 0.0;
    private static double armSetpointRad = 0.0;
    
    public DeAlgifier(DeAlgifierIO io){
        this.io = io;
        this.armPidController = new PIDController(DeAlgifierConstants.ARM_P, 0, DeAlgifierConstants.ARM_D);
        this.kickerPidController = new PIDController(DeAlgifierConstants.KICKER_P, 0, DeAlgifierConstants.KICKER_D);
    }
    
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("DeAlgifier", inputs);
        
        if (DriverStation.isDisabled()) { this.stop(); } 
        else {
            io.setArmVoltage(armPidController.calculate(armSetpointRad));
            io.setKickerVoltage(kickerPidController.calculate(kickerSetpointRPM));
        }
    }

    public void deAlgify() {
        armSetpointRad = DeAlgifierConstants.kArmAlgaeSetpointRad;
        kickerSetpointRPM = DeAlgifierConstants.kKickerSetpointRPM;
    }
    
    public void stop() {
        io.setArmVoltage(0.0);
        io.setKickerVoltage(0.0);
    }
}