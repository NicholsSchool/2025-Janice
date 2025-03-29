package frc.robot.subsystems.drive;

import frc.robot.Constants;
import java.util.Objects;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

/** Hardware interface for the NAVX 3 axis gyro */
public class GyroIONAVX implements GyroIO {
  private final AHRS navx;

  /** Constructor to initialize the NAVX */
  public GyroIONAVX() {
    Constants.RobotType robotType = Objects.requireNonNull(Constants.getRobot());
    switch (robotType) {
      case ROBOT_REAL_FRANKENLEW:
      case ROBOT_REAL_JANICE:
        navx = new AHRS(NavXComType.kMXP_SPI);
        break;
      default:  
        throw new RuntimeException("Invalid robot for NAVX");
    }
  }

  /**
   * Update the AK hardware inputs
   *
   * @param inputs the inputs to update
   */
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navx.isConnected();
    // navx uses positive yaw turn to right, so flip sign
    inputs.yawPositionRad = -Math.toRadians(navx.getAngle());
    inputs.yawVelocityRadPerSec = -Math.toRadians(navx.getRate());
    inputs.isTipped = Math.abs(navx.getPitch()) > 10.0 || Math.abs(navx.getRoll()) > 10.0;
  }

  @Override
  public void resetIMU() {
    System.out.println("resetting imu");
    navx.zeroYaw();
  }
}
