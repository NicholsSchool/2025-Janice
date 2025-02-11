package frc.robot.subsystems.EndEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
    
    @AutoLog
    public static class EndEffectorIOInputs{
        public double[] currentAmps = {0.0, 0.0};
        public double[] appliedVolts = {0.0, 0.0};
        public double[] velocityRadPerSec = {0.0, 0.0};
        public double currentHeight = 0.0;
        public boolean closed = false;
    }
      /** Updates the set of loggable inputs. */
  public default void updateInputs(EndEffectorIOInputs inputs) {};
  public default void setVoltage(double voltage) {};

}