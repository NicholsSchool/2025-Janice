package frc.robot.subsystems.Intake;

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

public class IntakeIOReal implements IntakeIO {
    
    //TODO: Need to fimplement the Solenoids because I dont know how to do that
    private TalonFX indexer;
    private Solenoid extender;
    private Rev2mDistanceSensor lSensor, rSensor;

    public IntakeIOReal() {
        indexer = new TalonFX(CAN.kIntakeMotor);
        extender = new Solenoid(PneumaticsModuleType.REVPH, Constants.IntakeConstants.kLeftPistonChannel);
        lSensor = new Rev2mDistanceSensor(Port.kOnboard, Unit.kMillimeters, RangeProfile.kHighAccuracy);
        rSensor = new Rev2mDistanceSensor(Port.kOnboard, Unit.kMillimeters, RangeProfile.kHighAccuracy);
    
        var config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = Constants.IntakeConstants.INTAKE_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        indexer.getConfigurator().apply(config);
    
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.velocityRadsPerSec = indexer.getVelocity().getValueAsDouble();
        inputs.motorVoltage = indexer.getMotorVoltage().getValueAsDouble();
        inputs.supplyVoltage = indexer.getSupplyVoltage().getValueAsDouble();
        inputs.currentAmps = indexer.getStatorCurrent().getValueAsDouble();
        inputs.hasCoralAligned = isCoralAligned();
    }
          
      @Override
      public void setVoltage(double voltage) {
        indexer.setVoltage(-voltage);
      }

      @Override
      public void setSolenoidState(boolean out){
        extender.set(out);
      }

      private boolean isCoralAligned(){

        return
        lSensor.getRange() < Constants.IntakeConstants.kCoralDistanceFarBound &&
        lSensor.getRange() > Constants.IntakeConstants.kCoralDistanceCloseBound &&
        rSensor.getRange() < Constants.IntakeConstants.kCoralDistanceFarBound &&
        rSensor.getRange() > Constants.IntakeConstants.kCoralDistanceCloseBound;
      }

}
