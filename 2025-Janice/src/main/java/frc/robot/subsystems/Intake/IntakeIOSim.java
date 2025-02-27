package frc.robot.subsystems.Intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {
    private static final DCMotor indexerModel = DCMotor.getKrakenX60(1);
    private boolean extender = false;

    private final DCMotorSim indexer =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(indexerModel, 0.025, IntakeConstants.kIndexerGearRatio),
          indexerModel);

    public void updateInputs(IntakeIOInputs inputs){
        inputs.velocityRadsPerSec = indexer.getAngularVelocityRadPerSec();
        inputs.supplyVoltage = indexer.getInputVoltage();
        inputs.currentAmps = indexer.getCurrentDrawAmps();
        inputs.hasCoralAligned = false;
    }

    public void setVoltage(double voltage) {
        indexer.setInputVoltage(voltage);
    }

}
