package frc.robot.subsystems.Outtake;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.OuttakeConstants;

public class OuttakeIOReal implements OuttakeIO {

    private TalonFX outtakeMotor;
    private CANrange outtakeSensor;

    public OuttakeIOReal(){
        outtakeMotor = new TalonFX(CAN.kOuttakeMotor, "Elevator");
        outtakeSensor = new CANrange(CAN.kOuttakeSensor, "Elevator");

        var config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = Constants.OuttakeConstants.kOuttakeCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        outtakeMotor.getConfigurator().apply(config);

        CANrangeConfiguration sensorConfig = new CANrangeConfiguration();
        sensorConfig.FovParams.withFOVRangeX(OuttakeConstants.kOuttakeSensorFOV);
        sensorConfig.FovParams.withFOVRangeY(OuttakeConstants.kOuttakeSensorFOV);
        sensorConfig.ProximityParams.withProximityThreshold(OuttakeConstants.kCoralDistanceThreshold);
        outtakeSensor.getConfigurator().apply(sensorConfig);
    }

    public void updateInputs(OuttakeIOInputs inputs) {
        inputs.motorVoltage = outtakeMotor.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = outtakeMotor.getStatorCurrent().getValueAsDouble();
        inputs.supplyVoltage = outtakeMotor.getSupplyVoltage().getValueAsDouble();
        inputs.hasCoral = outtakeSensor.getIsDetected().getValue().booleanValue();
        inputs.distance = outtakeSensor.getDistance().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage){
        outtakeMotor.setVoltage(voltage);
    }

    @AutoLogOutput
    public double distanceStdDevs() {
        return outtakeSensor.getDistanceStdDev().getValueAsDouble();
    }

    @AutoLogOutput
    public double getSignialStrength() {
        return outtakeSensor.getSignalStrength().getValueAsDouble();
    }

    @AutoLogOutput
    public double getAmbientSignal() {
        return outtakeSensor.getAmbientSignal().getValueAsDouble();
    }
}
