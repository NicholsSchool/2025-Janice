package frc.robot.subsystems.Outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {
    @AutoLog
    public static class OuttakeIOInputs {
        public double motorVoltage = 0.0;
        public double supplyVoltage = 0.0;
        public double currentAmps = 0.0;
        public boolean hasCoral = false;
        public double distance = -1.0;
    }

    /** Updates the set of loggable inputs. */
  public default void updateInputs(OuttakeIOInputs inputs) {}

  /** Set voltage command */
  public default void setVoltage(double voltage) {}
}
