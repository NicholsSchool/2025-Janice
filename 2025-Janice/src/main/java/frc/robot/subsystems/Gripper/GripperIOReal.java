package frc.robot.subsystems.Gripper;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class GripperIOReal implements GripperIO {

    public TalonFX gripperMotor;

    public GripperIOReal(){
        gripperMotor = new TalonFX(CAN.kGripperMotor, "Elevator");
     
         var config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = Constants.GripperConstants.kGripperCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        gripperMotor.getConfigurator().apply(config);
       

    }

    public void updateInputs(GripperIOInputs inputs){
        inputs.motorVoltage = gripperMotor.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = gripperMotor.getStatorCurrent().getValueAsDouble();
        inputs.supplyVoltage = gripperMotor.getSupplyVoltage().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage){
        gripperMotor.setVoltage(voltage);
    }

}
