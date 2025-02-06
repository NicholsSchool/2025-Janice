package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO{
      private static final DCMotor lSimModel = DCMotor.getKrakenX60(1);
      private static final DCMotor rSimModel = DCMotor.getKrakenX60(1);

      private final DCMotorSim lSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(lSimModel, 0.025, Constants.ModuleConstants.kDRIVE_GEAR_RATIO),
          lSimModel);
  private final DCMotorSim rSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(rSimModel, 0.025, Constants.ModuleConstants.kTURN_GEAR_RATIO),
          rSimModel);
    
    double elevatorAppliedVolts = 0.0;

    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        inputs.appliedVolts = new double[] {lSim.getInputVoltage(), rSim.getInputVoltage()};
        inputs.velocityRadPerSec = new double[] {lSim.getAngularVelocityRadPerSec(),rSim.getAngularVelocityRadPerSec()};
        inputs.currentHeight = this.getCurrentHeight();
        inputs.currentAmps = new double[] {lSim.getCurrentDrawAmps(), rSim.getCurrentDrawAmps()};
    }

    private double getCurrentHeight(){
        // find a regression for it 
        return lSim.getAngularPositionRad();
    }

    @Override
    public void setVoltage(double voltage) {
      lSim.setInputVoltage(voltage);
      rSim.setInputVoltage(voltage);
    }

}
