package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  @AutoLog
  public static class ArmIOInputs {
    public double angleRads = 0.0;
    public double angleDegs = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double motorVoltage = 0.0;
    public double supplyVoltage = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  /** Set voltage command */
  public default void setVoltage(double voltage) {}
}
