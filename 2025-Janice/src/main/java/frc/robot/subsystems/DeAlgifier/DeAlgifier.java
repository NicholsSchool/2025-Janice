package frc.robot.subsystems.DeAlgifier;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class DeAlgifier extends SubsystemBase{
    private DeAlgifierIO io;
    private final DeAlgifierIOInputsAutoLogged inputs = new DeAlgifierIOInputsAutoLogged();
    
    public DeAlgifier(DeAlgifierIO io){
        this.io = io;
    }
    
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("DeAlgifier", inputs);
        
        if (DriverStation.isDisabled()) {}
        
    }
    
    public void deAlgify() {
        io.setVoltage(-1.0);
    }
    
    public void stop() {
        io.setVoltage(0.0);
    }
}