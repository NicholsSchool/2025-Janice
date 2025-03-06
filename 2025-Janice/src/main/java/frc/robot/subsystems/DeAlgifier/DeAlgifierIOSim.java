package frc.robot.subsystems.DeAlgifier;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.DeAlgifierConstants;

public class DeAlgifierIOSim implements DeAlgifierIO {
    
    private static final DCMotor DeAlgifierMotorModel = DCMotor.getKrakenX60(2); //TODO make another motor.

     private final DCMotorSim DeAlgifierMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DeAlgifierMotorModel, 0.025, DeAlgifierConstants.kIndexerGearRatio),
          DeAlgifierMotorModel);

    public void updateInputs(DeAlgifierIOInputs inputs){
        inputs.supplyVoltage = DeAlgifierMotor.getInputVoltage();
        inputs.currentAmps = DeAlgifierMotor.getCurrentDrawAmps();
    }

    public void setVoltage(double voltage) {
        DeAlgifierMotor.setInputVoltage(voltage);
    }
}
