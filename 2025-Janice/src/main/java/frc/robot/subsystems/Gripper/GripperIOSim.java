package frc.robot.subsystems.Gripper;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Gripper.GripperIO.GripperIOInputs;


public class GripperIOSim implements GripperIO {

    private static final DCMotor gripperMotorModel = DCMotor.getFalcon500(1);

    private final DCMotorSim gripperMotor =
      new DCMotorSim(
         LinearSystemId.createDCMotorSystem(gripperMotorModel, 0.025, GripperConstants.kGripperGearRatio),
         gripperMotorModel);

         
    
    public void updateInputs(GripperIOInputs inputs){
        inputs.supplyVoltage = gripperMotor.getInputVoltage();
        inputs.currentAmps = gripperMotor.getCurrentDrawAmps();
    }

    public void setVoltage(double voltage) {
        gripperMotor.setInputVoltage(voltage);
    }

}

    

