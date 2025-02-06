package frc.robot.subsystems.EndEffector;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class EndEffectorIOReal implements EndEffectorIO{
    private TalonFX indexer;
    private AnalogInput endEffectorDistanceSensor;
    private Solenoid grabSolenoid;
    //TODO: Need to implement the distance sensor and solenoid

    public EndEffectorIOReal() {
        indexer = new TalonFX(CAN.kRightChain);
    
        var talonConfig = new TalonFXConfiguration();
        talonConfig.CurrentLimits.StatorCurrentLimit = Constants.EndEffectorConstants.CurrentLimit;
        talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        indexer.getConfigurator().apply(talonConfig);
    
        endEffectorDistanceSensor = new AnalogInput(Constants.EndEffectorConstants.elevatorLimitSwitchChannel);
      }

      @Override
      public void updateInputs(EndEffectorIOInputs inputs){
        inputs.appliedVolts = new double[] {indexer.getMotorVoltage().getValueAsDouble()};
        inputs.velocityRadPerSec = new double[] {indexer.getVelocity().getValueAsDouble()};
        inputs.currentAmps = new double[] {indexer.getStatorCurrent().getValueAsDouble()};
      }
    
      @Override
      public void setVoltage(double voltage) {
        indexer.setVoltage(voltage);
      }
}
