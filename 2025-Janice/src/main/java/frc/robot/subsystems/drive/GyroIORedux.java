package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLogOutput;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroSettings;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CAN;

/** Hardware interface for the Reduxrobotics Can-and-gyro IMU */
public class GyroIORedux implements GyroIO {
  private final Canandgyro gyro;

  /** Creates a new gyro, with period 0.02 on yaw updates, and 0.1 on status updates. */
  public GyroIORedux() {
    gyro = new Canandgyro( CAN.kReduxGyro );
    gyro.clearStickyFaults();
    gyro.setPartyMode(0);

    CanandgyroSettings settings = new CanandgyroSettings();

    gyro.setSettings(settings);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = gyro.isConnected();
    inputs.yawPositionRad =  Units.rotationsToRadians(gyro.getYaw());
    inputs.yawVelocityRadPerSec = Units.rotationsToRadians(gyro.getAngularVelocityYaw());
  }

  @Override
  public void resetIMU() {
    System.out.println("resetting IMU");
    gyro.setYaw(0);
  }

  @AutoLogOutput
  public double[] getAccelData() {
    return new double[] { gyro.getAccelerationX(), gyro.getAccelerationY(), gyro.getAccelerationZ() };
  }
}
