package frc.robot.subsystems.EndEffector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class EndEffectorIOSim implements EndEffectorIO{
    private static final DCMotor lSimModel = DCMotor.getKrakenX60(1);

    private final DCMotorSim lSim =
    new DCMotorSim(
        LinearSystemId.createDCMotorSystem(lSimModel, 0.025, Constants.ModuleConstants.kDRIVE_GEAR_RATIO),
        lSimModel);
  
  double elevatorAppliedVolts = 0.0;

  @Override
  public void updateInputs(EndEffectorIOInputs inputs){
      inputs.appliedVolts = new double[] {lSim.getInputVoltage()};
      inputs.velocityRadPerSec = new double[] {lSim.getAngularVelocityRadPerSec()};
      inputs.currentAmps = new double[] {lSim.getCurrentDrawAmps()};
  }

  private double getCurrentHeight(){
      // find a regression for it 
      return lSim.getAngularPositionRad();
  }

  @Override
  public void setVoltage(double voltage) {
    lSim.setInputVoltage(voltage);
  }

}
