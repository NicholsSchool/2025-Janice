package frc.robot.subsystems.EndEffector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorIOReal implements EndEffectorIO{
    private TalonFX intake;
    private Rev2mDistanceSensor distanceSensor;
    private Solenoid grabSolenoid;

    public EndEffectorIOReal() {
        intake = new TalonFX(CAN.kEndEffectorIntake);
    
        var talonConfig = new TalonFXConfiguration();
        talonConfig.CurrentLimits.StatorCurrentLimit = Constants.EndEffectorConstants.EndEffectorCurrentLimit;
        talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        grabSolenoid = new Solenoid(PneumaticsModuleType.REVPH, EndEffectorConstants.kSolenoidChannel);
        intake.getConfigurator().apply(talonConfig);
        distanceSensor = new Rev2mDistanceSensor(Port.kOnboard, Unit.kMillimeters, RangeProfile.kHighAccuracy);
      }

      @Override
      public void updateInputs(EndEffectorIOInputs inputs){
        inputs.appliedVolts = new double[] {intake.getMotorVoltage().getValueAsDouble()};
        inputs.velocityRadPerSec = new double[] {intake.getVelocity().getValueAsDouble()};
        inputs.currentAmps = new double[] {intake.getStatorCurrent().getValueAsDouble()};
        inputs.closed = grabSolenoid.get();
        inputs.hasCoral = distanceSensor.getRange() < Constants.EndEffectorConstants.kDistanceThreshold;
      }
    
      @Override
      public void setVoltage(double voltage) {
        intake.setVoltage(voltage);
      }

      @Override
      public void setClamp(boolean on){
        grabSolenoid.set(on);
      }

    

      
}
