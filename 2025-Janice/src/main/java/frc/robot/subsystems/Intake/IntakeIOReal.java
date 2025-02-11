package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class IntakeIOReal implements IntakeIO {
    
    //TODO: Need to fimplement the Solenoids because I dont know how to do that
    private TalonFX indexer;
    private Solenoid Lextender;
    private Solenoid Rextender;

    public IntakeIOReal() {
        indexer = new TalonFX(CAN.kIntakeMotor);
    
        var config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = Constants.IntakeConstants.INTAKE_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        indexer.getConfigurator().apply(config);
    
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.velocityRadsPerSec =
            new double[] {
              indexer.getVelocity().getValueAsDouble()
            };
        inputs.appliedOutput =
            new double[] {
              indexer.getMotorVoltage().getValueAsDouble()
            };
        inputs.busVoltage =
            new double[] {
              indexer.getSupplyVoltage().getValueAsDouble()
            };
        inputs.appliedVolts =
            new double[] {
              inputs.busVoltage[0] * inputs.appliedOutput[0]
            };
        inputs.currentAmps =
            new double[] {
              indexer.getStatorCurrent().getValueAsDouble()
            };
      }
    
      @Override
      public void setVoltage(double voltage) {
        indexer.setVoltage(-voltage);
      }
}
