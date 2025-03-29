package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {

     public double velocityRadsPerSec = 0.0;
     public double appliedVolts = 0.0;
     public double motorVoltage = 0.0;
     public double supplyVoltage = 0.0;
     public double currentAmps = 0.0;
     public boolean hasCoralAligned = false;
    }

/** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Set voltage command */
  public default void setVoltage(double voltage) {}

  public default void setSolenoidState (boolean out) {}
}