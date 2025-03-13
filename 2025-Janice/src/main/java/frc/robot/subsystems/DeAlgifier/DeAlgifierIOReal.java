package frc.robot.subsystems.DeAlgifier;

import static edu.wpi.first.units.Units.Radian;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.CAN;
import frc.robot.Constants.DeAlgifierConstants;

public class DeAlgifierIOReal implements DeAlgifierIO {

    private TalonFX arm;
    private SparkMax kicker;

    public DeAlgifierIOReal() {
        arm = new TalonFX(CAN.kDeAlgifierArm, "Elevator");
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = DeAlgifierConstants.kDeAlgifierCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        arm.getConfigurator().apply(config);

        kicker = new SparkMax(CAN.kDeAlgifierKicker, MotorType.kBrushless );
        SparkMaxConfig config2 = new SparkMaxConfig();
        config2.absoluteEncoder.positionConversionFactor(1.0);
        kicker.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void updateInputs(DeAlgifierIOInputs inputs){
        inputs.armMotorVoltage = arm.getMotorVoltage().getValueAsDouble();
        inputs.armCurrentAmps = arm.getStatorCurrent().getValueAsDouble();
        inputs.armSupplyVoltage = arm.getSupplyVoltage().getValueAsDouble();
        inputs.armPositionRad = kicker.getAbsoluteEncoder().getPosition() * Math.PI * 2;

        inputs.kickerMotorVoltage = kicker.getAppliedOutput();
        inputs.kickerSupplyVoltage = kicker.getBusVoltage();
        inputs.kickerCurrentAmps = kicker.getOutputCurrent();
        inputs.kickerVelocityRPM = kicker.getEncoder().getVelocity();
    }

    @Override
    public void setArmVoltage(double voltage){
        arm.setVoltage(voltage);
    }

    @Override
    public void setKickerVoltage( double voltage ) {
        kicker.setVoltage(voltage);
    }
}
