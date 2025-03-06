package frc.robot.subsystems.DeAlgifier;

import org.littletonrobotics.junction.AutoLog;
public interface DeAlgifierIO{

    @AutoLog
    public static class DeAlgifierIOInputs {
        public double motorVoltage = 0.0;
        public double supplyVoltage = 0.0;
        public double currentAmps = 0.0;
    }
        /** Updates the set of loggable inputs. */
        public default void updateInputs(DeAlgifierIOInputs inputs) {}
        
        /** Set voltage command */
        public default void setVoltage(double voltage) {}
}
