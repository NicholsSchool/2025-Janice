package frc.robot.subsystems.DeAlgifier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class DeAlgifierIOReal implements DeAlgifierIO {

    private TalonFX DeAlgifierMotor;

    public DeAlgifierIOReal(){
        DeAlgifierMotor = new TalonFX(CAN.kDeAlgifierMotor);
         var config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = Constants.DeAlgifierConstants.kDeAlgifierCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        DeAlgifierMotor.getConfigurator().apply(config);

    }

    public void updateInputs(DeAlgifierIOInputs inputs){
        inputs.motorVoltage = DeAlgifierMotor.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = DeAlgifierMotor.getStatorCurrent().getValueAsDouble();
        inputs.supplyVoltage = DeAlgifierMotor.getSupplyVoltage().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage){
        DeAlgifierMotor.setVoltage(voltage);
    }

}
