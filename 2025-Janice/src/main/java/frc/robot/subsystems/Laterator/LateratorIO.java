package frc.robot.subsystems.Laterator;

import org.littletonrobotics.junction.AutoLog;

public interface LateratorIO {
    @AutoLog
    public static class LateratorIOInputs {
        public double motorVoltage = 0.0;
        public double supplyVoltage = 0.0;
        public double currentAmps = 0.0;
        
        public boolean limitSwitch = false;
    }

     /** Updates the set of loggable inputs. */
  public default void updateInputs(LateratorIOInputs inputs) {}

  /** Set voltage command */
  public default void setVoltage(double voltage) {}


}