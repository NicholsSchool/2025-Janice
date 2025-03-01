package frc.robot.subsystems.climber;

public class ClimberIOSim implements ClimberIO {
    
    boolean climberState = false;

    public void updateInputs (CLimberIOInputs inputs) {
        inputs.extended = climberState;
    }

    public void setClimbState(boolean extended) {
        climberState = extended;
    };
}
