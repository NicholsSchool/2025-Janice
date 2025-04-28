package frc.robot.subsystems.DeAlgifier;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.DeAlgifierConstants;

public class DeAlgifierIOSim implements DeAlgifierIO {
    
    private static final DCMotor laterator = DCMotor.getKrakenX60(1);
    private static final DCMotor grabber = DCMotor.getFalcon500(1);

    private final DCMotorSim lateratorMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(laterator, 0.025, DeAlgifierConstants.kLateratorGearRatio),
          laterator);
    
    private final DCMotorSim grabberMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(grabber, 0.025, DeAlgifierConstants.kGrabberGearRatio),
          grabber);

    public void updateInputs(DeAlgifierIOInputs inputs){
        inputs.lateratorMotorVoltage = lateratorMotor.getInputVoltage();
        inputs.lateratorCurrentAmps = lateratorMotor.getCurrentDrawAmps();
        inputs.lateratorSupplyVoltage = lateratorMotor.getInputVoltage();
        inputs.lateratorPositionRad = lateratorMotor.getAngularPositionRad();
        inputs.lateratorVelocityRadPerSec = lateratorMotor.getAngularVelocityRadPerSec();

        inputs.grabberMotorVoltage = grabberMotor.getInputVoltage();
        inputs.grabberSupplyVoltage = grabberMotor.getInputVoltage();
        inputs.grabberCurrentAmps = grabberMotor.getCurrentDrawAmps();
        inputs.grabberVelocityRPM = grabberMotor.getAngularVelocityRPM();

        inputs.frontLimitSwitch = inputs.lateratorPositionRad > DeAlgifierConstants.kLateratorOutPositionRad;
        inputs.backLimitSwitch = inputs.lateratorPositionRad < DeAlgifierConstants.kLateratorInPositionRad;
    }

    @Override
    public void setLateratorVoltage(double voltage){
        lateratorMotor.setInputVoltage(voltage);
    }

    @Override
    public void setGrabberVoltage( double voltage ) {
        grabberMotor.setInputVoltage(voltage);
    }

    @Override
    public void setGrabberBrake( boolean enable ) {
        if( enable )
            grabberMotor.setInputVoltage(0);
    }
}
