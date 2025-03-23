package frc.robot.subsystems.DeAlgifier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeAlgifierConstants;

import org.littletonrobotics.junction.Logger;

public class DeAlgifier extends SubsystemBase{
    private DeAlgifierIO io;
    private final DeAlgifierIOInputsAutoLogged inputs = new DeAlgifierIOInputsAutoLogged();

    private final PIDController lateratorPidController;
    private final PIDController grabberPidController;
    
    public DeAlgifier(DeAlgifierIO io){
        this.io = io;
        lateratorPidController = new PIDController(DeAlgifierConstants.kLateratorPVelocity, 0, DeAlgifierConstants.kLateratorDVelocity);
        grabberPidController = new PIDController(DeAlgifierConstants.kGrabberP, 0, DeAlgifierConstants.kGrabberD);
    }
    
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("DeAlgifier", inputs);
        
        if (DriverStation.isDisabled()) {
             io.setLateratorVoltage(0.0);
             io.setGrabberBrake(true);
        } 
    }

    /**
     * Called once to request laterator out
     */
    public void lateratorOut() {
        if( !inputs.frontLimitSwitch )
            io.setLateratorVoltage(lateratorPidController.calculate(inputs.lateratorVelocityRadPerSec, DeAlgifierConstants.kLateratorVelocityGoalRadPerSec));
        else io.setLateratorVoltage(0.0);
    }

    /**
     * Called once to request laterator in
     */
    public void lateratorIn() {
        if( !inputs.backLimitSwitch)
            io.setLateratorVoltage(lateratorPidController.calculate(inputs.lateratorVelocityRadPerSec, -DeAlgifierConstants.kLateratorVelocityGoalRadPerSec));
        else io.setLateratorVoltage(0.0);
    }

    public void intake() {
        io.setGrabberBrake(false);
        io.setGrabberVoltage(grabberPidController.calculate(inputs.grabberVelocityRPM, DeAlgifierConstants.kGrabberIntakeSetpointRPM));
    }

    public void outtake() {
        io.setGrabberBrake(false);
        io.setGrabberVoltage(grabberPidController.calculate(inputs.grabberVelocityRPM, DeAlgifierConstants.kGrabberEjectSetpointRPM));
    }

    public void holdAlgae() {
        io.setGrabberBrake(true);
    }
}