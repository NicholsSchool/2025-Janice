package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorIOReal implements EndEffectorIO{
    private TalonFX intake;
    private Rev2mDistanceSensor distanceSensor;
    private Solenoid grabSolenoid;
    //TODO: Need to implement the distance sensor and solenoid

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
