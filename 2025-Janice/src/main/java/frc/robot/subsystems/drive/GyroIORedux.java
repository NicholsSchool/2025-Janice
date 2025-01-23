package frc.robot.subsystems.drive;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroSettings;

/** Hardware interface for the Reduxrobotics Can-and-gyro IMU */
public class GyroIORedux implements GyroIO {
  private final Canandgyro gyro;

  /** Creates a new gyro, with period 0.02 on yaw updates, and 0.1 on status updates. */
  public GyroIORedux( int CANID ) {
    gyro = new Canandgyro( CANID );
    gyro.clearStickyFaults();
    gyro.setPartyMode(0);

    CanandgyroSettings settings = new CanandgyroSettings();
    settings.setAngularPositionFramePeriod(0.02); // 20ms update
    settings.setYawFramePeriod(0.02);
    settings.setStatusFramePeriod(0.1);

    gyro.setSettings(settings, 0.050);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = gyro.isConnected();
    inputs.yawPositionRad = -Math.toRadians(gyro.getYaw());
    inputs.yawVelocityRadPerSec = -Math.toRadians(gyro.getAngularVelocityYaw());
  }

  @Override
  public void resetIMU() {
    System.out.println("resetting IMU");
    gyro.setYaw(0);
  }
}
