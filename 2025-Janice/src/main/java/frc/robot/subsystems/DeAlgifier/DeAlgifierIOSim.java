package frc.robot.subsystems.DeAlgifier;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.DeAlgifierConstants;
import frc.robot.Constants.IntakeConstants;

public class DeAlgifierIOSim implements DeAlgifierIO {
    
    private static final DCMotor arm = DCMotor.getFalcon500(1);
    private static final DCMotor kicker = DCMotor.getNeo550(1);

    private final DCMotorSim armMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(arm, 0.025, 1.0),
          arm);
    
    private final DCMotorSim kickerMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(kicker, 0.025, DeAlgifierConstants.kKickerGearRatio),
          kicker);

    public void updateInputs(DeAlgifierIOInputs inputs){ //TODO: fix these update inputs
        inputs.armMotorVoltage = armMotor.getInputVoltage();
        inputs.armSupplyVoltage = armMotor.getInputVoltage();
        inputs.armCurrentAmps = armMotor.getCurrentDrawAmps();
        inputs.armPositionRad = armMotor.getAngularPositionRad();

        inputs.kickerMotorVoltage = kickerMotor.getInputVoltage();
        inputs.kickerSupplyVoltage = kickerMotor.getInputVoltage();
        inputs.kickerCurrentAmps = kickerMotor.getCurrentDrawAmps();
        inputs.kickerVelocityRPM = kickerMotor.getAngularVelocityRPM();
    }

    @Override
    public void setArmVoltage( double voltage ) {
        armMotor.setInputVoltage(voltage);
    }

    @Override
    public void setKickerVoltage( double voltage ) {
        kickerMotor.setInputVoltage(voltage);
    }
}
