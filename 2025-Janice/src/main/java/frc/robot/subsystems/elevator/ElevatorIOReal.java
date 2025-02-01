package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class ElevatorIOReal implements ElevatorIO{
    private TalonFX lShoulder;
    private TalonFX rShoulder;
    private DigitalInput elevatorLimitSwitch;

    public ElevatorIOReal() {
    lShoulder = new TalonFX(CAN.kLeftChain);
    rShoulder = new TalonFX(CAN.kRightChain);

    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = Constants.ElevatorConstants.ElevatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    lShoulder.getConfigurator().apply(config);
    rShoulder.setControl(new Follower(CAN.kLeftChain, true));

    elevatorLimitSwitch = new DigitalInput(Constants.ElevatorConstants.elevatorLimitSwitchChannel);
  }

  private double getHeight(){
    //TODO: we need to figure out what the conversion is between this and the actual height is 
    return lShoulder.getPosition().getValueAsDouble();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs){
    inputs.currentHeight = this.getHeight();
    inputs.appliedVolts = new double[] {lShoulder.getMotorVoltage().getValueAsDouble(), rShoulder.getMotorVoltage().getValueAsDouble()};
    inputs.velocityRadPerSec = new double[] {lShoulder.getVelocity().getValueAsDouble(), rShoulder.getVelocity().getValueAsDouble()};
    inputs.currentAmps = new double[] {lShoulder.getStatorCurrent().getValueAsDouble(), lShoulder.getStatorCurrent().getValueAsDouble()};
  }

  @Override
  public void setVoltage(double voltage) {
    lShoulder.setVoltage(voltage);
  }
  
}
