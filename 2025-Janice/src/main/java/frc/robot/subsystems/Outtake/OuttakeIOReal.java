package frc.robot.subsystems.Outtake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class OuttakeIOReal implements OuttakeIO {

    private TalonFX outtakeMotor;

    public OuttakeIOReal(){
        outtakeMotor = new TalonFX(CAN.kOuttakeMotor);

         var config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = Constants.OuttakeConstants.kOuttakeCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        outtakeMotor.getConfigurator().apply(config);

    }

    public void updateInputs(OuttakeIOInputs inputs){
        inputs.motorVoltage = outtakeMotor.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = outtakeMotor.getStatorCurrent().getValueAsDouble();
        inputs.supplyVoltage = outtakeMotor.getSupplyVoltage().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage){
        outtakeMotor.setVoltage(voltage);
    }

}
