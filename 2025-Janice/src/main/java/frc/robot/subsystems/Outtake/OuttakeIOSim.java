package frc.robot.subsystems.Outtake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IntakeConstants;

public class OuttakeIOSim implements OuttakeIO {

    private static final DCMotor outtakeMotorModel = DCMotor.getKrakenX60(1);

     private final DCMotorSim outtakeMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(outtakeMotorModel, 0.025, IntakeConstants.kIndexerGearRatio),
          outtakeMotorModel);

    public void updateInputs(OuttakeIOInputs inputs){
        inputs.supplyVoltage = outtakeMotor.getInputVoltage();
        inputs.currentAmps = outtakeMotor.getCurrentDrawAmps();
    }

    public void setVoltage(double voltage) {
        outtakeMotor.setInputVoltage(voltage);
    }
}

