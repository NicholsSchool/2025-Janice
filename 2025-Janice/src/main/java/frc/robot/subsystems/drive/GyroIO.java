package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

/** The interface for any GYRO connected to the robot */
public interface GyroIO {
  /**
   * Updates the set of loggable inputs.
   *
   * @param inputs the inputs to update
   */
  default void updateInputs(GyroIOInputs inputs) {}

  @AutoLog
  class GyroIOInputs {
    public boolean connected = false;
    public double rollPositionRad = 0.0;
    public double pitchPositionRad = 0.0;
    public double yawPositionRad = 0.0; // cumulative yaw angle of robot. positive turn to left
    public double rollVelocityRadPerSec = 0.0;
    public double pitchVelocityRadPerSec = 0.0;
    public double yawVelocityRadPerSec = 0.0; // current yaw velocity, positive turn to left.
    public boolean isTipped = false;
  }

  default void resetIMU() {}
}
