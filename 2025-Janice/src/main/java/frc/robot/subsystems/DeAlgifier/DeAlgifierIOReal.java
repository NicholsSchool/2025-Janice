package frc.robot.subsystems.DeAlgifier;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DeAlgifierConstants;

public class DeAlgifierIOReal implements DeAlgifierIO {

    private TalonFX laterator;
    private TalonFX grabber;
    private StaticBrake staticBrake;
    private VoltageOut voltageControl;

    private DigitalInput frontLimitSwitch;
    private DigitalInput backLimitSwitch;

    public DeAlgifierIOReal() {
        laterator = new TalonFX(CAN.kDeAlgifierLaterator, "Elevator");
        TalonFXConfiguration lateratorConfig = new TalonFXConfiguration();
        lateratorConfig.CurrentLimits.StatorCurrentLimit = 20;
        lateratorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        lateratorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        lateratorConfig.Feedback.SensorToMechanismRatio = DeAlgifierConstants.kLateratorGearRatio;
        laterator.getConfigurator().apply(lateratorConfig);

        grabber = new TalonFX(CAN.kDeAlgifierGrabber, "Elevator");
        TalonFXConfiguration grabberConfig = new TalonFXConfiguration();
        grabberConfig.CurrentLimits.StatorCurrentLimit = DeAlgifierConstants.kDeAlgifierGrabberCurrentLimit;
        grabberConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        grabberConfig.CurrentLimits.SupplyCurrentLimit = 30;
        grabberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        grabberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        grabberConfig.Feedback.SensorToMechanismRatio = DeAlgifierConstants.kGrabberGearRatio;
        grabber.getConfigurator().apply(grabberConfig);

        frontLimitSwitch = new DigitalInput(Constants.DeAlgifierConstants.frontLimitSwitchID);
        backLimitSwitch = new DigitalInput(Constants.DeAlgifierConstants.backLimitSwitchID);

        staticBrake = new StaticBrake();

        voltageControl = new VoltageOut(0.0);
        grabber.setControl(voltageControl);
    }

    public void updateInputs(DeAlgifierIOInputs inputs){
        inputs.lateratorMotorVoltage = laterator.getMotorVoltage().getValueAsDouble();
        inputs.lateratorCurrentAmps = laterator.getStatorCurrent().getValueAsDouble();
        inputs.lateratorSupplyVoltage = laterator.getSupplyVoltage().getValueAsDouble();
        inputs.lateratorPositionRad = laterator.getPosition().getValue().in(Radian);
        inputs.lateratorVelocityRadPerSec = laterator.getVelocity().getValue().in(RadiansPerSecond);

        inputs.grabberMotorVoltage = grabber.getMotorVoltage().getValueAsDouble();
        inputs.grabberSupplyVoltage = grabber.getSupplyVoltage().getValueAsDouble();
        inputs.grabberCurrentAmps = grabber.getSupplyCurrent().getValueAsDouble();
        inputs.grabberVelocityRPM = grabber.getVelocity().getValue().in(RPM);

        inputs.frontLimitSwitch = !frontLimitSwitch.get();
        inputs.backLimitSwitch = !backLimitSwitch.get();
    }

    @Override
    public void setLateratorVoltage(double voltage){
        laterator.setVoltage(voltage);
    }

    @Override
    public void setGrabberVoltage( double voltage ) {
        voltageControl.Output = voltage;
    }

    public void setGrabberBrake( boolean enable ) {
        grabber.setControl(enable ? staticBrake : voltageControl);
    }
}
