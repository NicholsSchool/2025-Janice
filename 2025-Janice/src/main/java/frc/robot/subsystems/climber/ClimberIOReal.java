package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;

public class ClimberIOReal implements ClimberIO {

    private Solenoid climber;

    public ClimberIOReal() {
        climber = new Solenoid(null, Constants.ClimberConstants.kChannel);
    }

    public void updateInputs(CLimberIOInputs inputs) {
    

    }

    public void setClimbState(boolean extended){
        climber.set(extended);
    }
}
