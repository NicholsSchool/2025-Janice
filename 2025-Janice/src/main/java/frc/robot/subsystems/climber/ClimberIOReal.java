package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;

public class ClimberIOReal implements ClimberIO {

    private Solenoid lClimber;
    private Solenoid rClimber;

    public ClimberIOReal() {
        lClimber = new Solenoid(null, Constants.ClimberConstants.kLChannel);
        rClimber = new Solenoid(null, Constants.ClimberConstants.kRChannel);
    }

    public void updateInputs(CLimberIOInputs inputs) {
    

    }
}
