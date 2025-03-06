package frc.robot.subsystems.Outtake;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;


import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class OuttakeIOReal implements OuttakeIO {

    private TalonFX outtakeMotor;
    private Rev2mDistanceSensor outtakeSensor;

    public OuttakeIOReal(){
        outtakeMotor = new TalonFX(CAN.kOuttakeMotor, "Elevator");
        outtakeSensor = new Rev2mDistanceSensor(Port.kOnboard, Unit.kMillimeters, RangeProfile.kHighAccuracy);

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
        inputs.hasCoral = seesCoral();
    }

    @Override
    public void setVoltage(double voltage){
        outtakeMotor.setVoltage(voltage);
    }

    public boolean seesCoral(){
        return outtakeSensor.getRange() < Constants.OuttakeConstants.kCoralDistanceFarBound &&
        outtakeSensor.getRange() > Constants.OuttakeConstants.kCoralDistanceCloseBound;
    }

    @AutoLogOutput
    public double getOuttakeSensor() {
        return outtakeSensor.getRange();
    }

}
