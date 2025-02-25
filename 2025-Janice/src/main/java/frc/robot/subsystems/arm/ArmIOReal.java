package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class ArmIOReal implements ArmIO {

  private TalonFX armMotor;
  private DutyCycleEncoder absoluteEncoder;

  public ArmIOReal() {
    armMotor = new TalonFX(CAN.kArmMotor);

    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = Constants.ArmConstants.ARM_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armMotor.getConfigurator().apply(config);

    absoluteEncoder = new DutyCycleEncoder(Constants.ArmConstants.kThroughBoreChannel);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.angleDegs = this.getArmAngle(Degrees);
    inputs.angleRads = this.getArmAngle(Radians);
    inputs.velocityRadsPerSec = armMotor.getVelocity().getValueAsDouble();
    inputs.motorVoltage = armMotor.getMotorVoltage().getValueAsDouble();
    inputs.supplyVoltage = armMotor.getSupplyVoltage().getValueAsDouble();
    inputs.currentAmps = armMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    armMotor.setVoltage(-voltage);
  }

  private double getArmAngle(AngleUnit unit) {
    if (unit == Units.Degrees || unit == Degree) {
      return absoluteEncoder.get() * 360;
    } else {
      return absoluteEncoder.get() * 2 * Math.PI;
    }
  }
}