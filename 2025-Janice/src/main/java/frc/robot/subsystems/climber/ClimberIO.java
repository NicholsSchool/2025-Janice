package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    
    @AutoLog
    public static class CLimberIOInputs {
        public boolean extended = false;
    }

    public default void updateInputs (CLimberIOInputs inputs) {}

    public default void setClimbState(boolean extended) {};
    }

