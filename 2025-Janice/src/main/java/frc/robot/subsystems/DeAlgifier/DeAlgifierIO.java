package frc.robot.subsystems.DeAlgifier;

import org.littletonrobotics.junction.AutoLog;
public interface DeAlgifierIO {

    @AutoLog
    public static class DeAlgifierIOInputs {
        public double lateratorMotorVoltage = 0.0;
        public double lateratorSupplyVoltage = 0.0;
        public double lateratorCurrentAmps = 0.0;
        public double lateratorPositionRad = 0.0;
        public double lateratorVelocityRadPerSec = 0.0;

        public double grabberMotorVoltage = 0.0;
        public double grabberSupplyVoltage = 0.0;
        public double grabberCurrentAmps = 0.0;
        public double grabberVelocityRPM = 0.0;

        public boolean frontLimitSwitch = false;
        public boolean backLimitSwitch = false;
    }
        /** Updates the set of loggable inputs. */
        public void updateInputs(DeAlgifierIOInputs inputs);
        
        /** Set voltage command */
        public default void setLateratorVoltage(double voltage) {}

        public default void setGrabberVoltage( double voltage ) {}

        public default void setGrabberBrake(boolean enable) {} 
}
