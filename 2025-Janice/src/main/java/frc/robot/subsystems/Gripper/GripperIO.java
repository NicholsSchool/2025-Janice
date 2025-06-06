package frc.robot.subsystems.Gripper;

import org.littletonrobotics.junction.AutoLog;

public interface GripperIO {
    @AutoLog
    public static class GripperIOInputs {
        public double motorVoltage = 0.0;
        public double supplyVoltage = 0.0;
        public double currentAmps = 0.0;
    }

     /** Updates the set of loggable inputs. */
  public default void updateInputs(GripperIOInputs inputs) {}

  /** Set voltage command */
  public default void setVoltage(double voltage) {}

  public default void setGripperBrake(boolean enable) {} 

}

