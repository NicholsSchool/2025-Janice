package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class ElevatorIOReal implements ElevatorIO{
    private TalonFX lShoulder;
    private TalonFX rShoulder;

    public ElevatorIOReal() {
    lShoulder = new TalonFX(CAN.kLeftChain);
    rShoulder = new TalonFX(CAN.kRightChain);

    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = Constants.ElevatorConstants.ElevatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    lShoulder.getConfigurator().apply(config);
    rShoulder.setControl(new Follower(CAN.kLeftChain, true));
  }

  
}
