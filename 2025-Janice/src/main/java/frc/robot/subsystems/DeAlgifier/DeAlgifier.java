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

    private LateratorMode lateratorMode;
    private GrabberMode grabberMode;

    private double lateralManual = 0.0;

    boolean manual = true;

    enum LateratorMode {
        IN,
        OUT,
        IDLE,
    }

    enum GrabberMode {
        INTAKE,
        OUTTAKE,
        HOLD,
        IDLE
    }

    
    public DeAlgifier(DeAlgifierIO io){
        this.io = io;
        lateratorPidController = new PIDController(DeAlgifierConstants.kLateratorPVelocity, 0, DeAlgifierConstants.kLateratorDVelocity);
        grabberPidController = new PIDController(DeAlgifierConstants.kGrabberP, 0, DeAlgifierConstants.kGrabberD);

        lateratorPidController.reset();
        grabberPidController.reset();

        lateratorMode = LateratorMode.IDLE;
        grabberMode = GrabberMode.IDLE;
    }
    
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("DeAlgifier", inputs);
        
        if (DriverStation.isDisabled()) {
             io.setLateratorVoltage(0.0);
             io.setGrabberBrake(true);

             lateratorMode = LateratorMode.IDLE;
             grabberMode = GrabberMode.IDLE; //for safety upon reenable

             return;
        } 
        if(manual){
            io.setLateratorVoltage(lateralManual);
        }else{
        
            switch( lateratorMode ) {
            case IN -> {
                if( !inputs.backLimitSwitch)
                    io.setLateratorVoltage(lateratorPidController.calculate(
                        inputs.lateratorVelocityRadPerSec, -DeAlgifierConstants.kLateratorVelocityGoalRadPerSec));
                else io.setLateratorVoltage(0.0);
            }
            case OUT -> {
                if( !inputs.frontLimitSwitch )
                    io.setLateratorVoltage(lateratorPidController.calculate(
                        inputs.lateratorVelocityRadPerSec, DeAlgifierConstants.kLateratorVelocityGoalRadPerSec));
                else io.setLateratorVoltage(0.0);
            }
            case IDLE -> io.setLateratorVoltage(0.0);
            default -> io.setLateratorVoltage(0.0);
        }

        switch( grabberMode ) {
            case INTAKE -> {
                io.setGrabberBrake(false);
                io.setGrabberVoltage(grabberPidController.calculate(inputs.grabberVelocityRPM, DeAlgifierConstants.kGrabberIntakeSetpointRPM));
            }
            case OUTTAKE -> {
                io.setGrabberBrake(false);
                io.setGrabberVoltage(grabberPidController.calculate(inputs.grabberVelocityRPM, DeAlgifierConstants.kGrabberEjectSetpointRPM));
            }
            case HOLD, IDLE -> io.setGrabberBrake(true);
            default -> io.setGrabberBrake(true);
        }
        }
    }
    

    /**
     * Called once to request laterator out
     */
    public void lateratorOut() {
        lateratorMode = LateratorMode.OUT;
    }

    public void lateratorManual(double input){
        lateralManual = input*4.0;
    }

    /**
     * Called once to request laterator in
     */
    public void lateratorIn() {
       lateratorMode = LateratorMode.IN;
    }

    public void intake() {
        grabberMode = GrabberMode.INTAKE;
    }

    public void outtake() {
        grabberMode = GrabberMode.OUTTAKE;
    }

    public void holdAlgae() {
        grabberMode = GrabberMode.HOLD;
    }
}