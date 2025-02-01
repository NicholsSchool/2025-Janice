package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  @AutoLog
  public static class ArmIOInputs {
    public double angleRads = 0.0;
    public double angleDegs = 0.0;
    public double[] velocityRadsPerSec = new double[] {0, 0};
    public double[] appliedVolts = new double[] {0, 0};
    public double[] appliedOutput = new double[] {0, 0};
    public double[] busVoltage = new double[] {0, 0};
    public double[] currentAmps = new double[] {0, 0};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  /** Set voltage command */
  public default void setVoltage(double voltage) {}
}
