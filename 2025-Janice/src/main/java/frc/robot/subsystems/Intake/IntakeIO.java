package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {

     public double[] velocityRadsPerSec = new double[] {0, 0};
     public double[] appliedVolts = new double[] {0, 0};
     public double[] appliedOutput = new double[] {0, 0};
     public double[] busVoltage = new double[] {0, 0};
     public double[] currentAmps = new double[] {0, 0};
    }

/** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Set voltage command */
  public default void setVoltage(double voltage) {}
}