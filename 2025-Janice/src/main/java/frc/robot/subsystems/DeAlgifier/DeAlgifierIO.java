package frc.robot.subsystems.DeAlgifier;

import org.littletonrobotics.junction.AutoLog;
public interface DeAlgifierIO {

    @AutoLog
    public static class DeAlgifierIOInputs {
        public double armMotorVoltage = 0.0;
        public double armSupplyVoltage = 0.0;
        public double armCurrentAmps = 0.0;
        public double armPositionRad = 0.0;

        public double kickerMotorVoltage = 0.0;
        public double kickerSupplyVoltage = 0.0;
        public double kickerCurrentAmps = 0.0;
        public double kickerVelocityRPM = 0.0;
    }
        /** Updates the set of loggable inputs. */
        public default void updateInputs(DeAlgifierIOInputs inputs) {}
        
        /** Set voltage command */
        public default void setArmVoltage(double voltage) {}

        public default void setKickerVoltage( double voltage ) {}
}
