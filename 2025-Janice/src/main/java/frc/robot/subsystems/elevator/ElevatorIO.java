package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs{
        public double[] currentAmps = {0.0, 0.0};
        public double[] appliedVolts = {0.0, 0.0};
        public double[] velocityRadPerSec = {0.0, 0.0};
        public double currentHeight = 0.0;
        public boolean limitSwitch = false;
    }
      /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {};
  public default void setVoltage(double voltage) {};


}
