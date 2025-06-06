import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LateratorConstants;
import frc.robot.subsystems.Gripper.GripperIO.GripperIOInputs;
import frc.robot.subsystems.Laterator.LateratorIO;

public class LateratorIOSim implements LateratorIO{
    
    private static final DCMotor lateratorMotorModel = DCMotor.getKrakenX60(1);

    private final DCMotorSim lateratorMotor =
      new DCMotorSim(
         LinearSystemId.createDCMotorSystem(lateratorMotorModel, 0.025, LateratorConstants.kLateratorGearRatio),
         lateratorMotorModel);

         
    
    public void updateInputs(LateratorIOInputs inputs){
        inputs.supplyVoltage = lateratorMotor.getInputVoltage();
        inputs.currentAmps = lateratorMotor.getCurrentDrawAmps();
    }

    public void setVoltage(double voltage) {
        lateratorMotor.setInputVoltage(voltage);
    }
}
