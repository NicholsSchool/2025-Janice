package frc.robot.subsystems.Laterator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class LateratorIOReal implements LateratorIO{
      
    private TalonFX lateratorMotor;

    private DigitalInput limitSwitch;

    public LateratorIOReal(){
        lateratorMotor = new TalonFX(CAN.kLaterator, "Elevator");
     
         var config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = Constants.LateratorConstants.kLateratorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        lateratorMotor.getConfigurator().apply(config);
       
        limitSwitch = new DigitalInput(CAN.kLateratorLimitSwitch);
    }

    public void updateInputs(LateratorIOInputs inputs){
        inputs.motorVoltage = lateratorMotor.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = lateratorMotor.getStatorCurrent().getValueAsDouble();
        inputs.supplyVoltage = lateratorMotor.getSupplyVoltage().getValueAsDouble();
        
        inputs.limitSwitch = limitSwitch.get();
    }

    @Override
    public void setVoltage(double voltage){
        lateratorMotor.setVoltage(voltage);
    }
  
}
