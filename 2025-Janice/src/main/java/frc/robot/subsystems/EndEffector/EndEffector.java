package frc.robot.subsystems.EndEffector;

import frc.robot.Constants.EndEffectorConstants;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class EndEffector extends SubsystemBase {
    
    private EndEffectorIO io;
    private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

    private double spin = 0.0;
    
    private boolean clampOn = false;

    private double intakeVoltage = 0.0;



    private static final LoggedTunableNumber EndEffectorMaxVelocityRad =
      new LoggedTunableNumber("EndEffector/MaxVelocityRad");
  private static final LoggedTunableNumber EndEffectorMaxAccelerationRad =
      new LoggedTunableNumber("EndEffector/MaxAccelerationRad");

  public EndEffector(EndEffectorIO io){
    this.io = io;

    EndEffectorMaxAccelerationRad.initDefault(1.1);
    EndEffectorMaxVelocityRad.initDefault(0.9);

   
}
public void periodic(){
    io.updateInputs(inputs);

    updateTunables();

    Logger.processInputs("End Effector", inputs);

     if (DriverStation.isDisabled()) {}

  io.setVoltage(intakeVoltage);

  io.setClamp(clampOn);
  
}

private void updateTunables() {
    // Update from tunable numbers
    if (EndEffectorMaxVelocityRad.hasChanged(hashCode())
        || EndEffectorMaxAccelerationRad.hasChanged(hashCode())) { 
    }
  }


  @AutoLogOutput
  public double[] getOutputCurrent() {
    return inputs.currentAmps;
  }

  @AutoLogOutput
  public boolean hasCoral(){
    return inputs.hasCoral;
  }

  public void setVoltage(double voltage, boolean stopForCoral){
    //TODO: check if voltage is positive or negative for intaking
    if (stopForCoral && voltage < 0 && hasCoral()) {intakeVoltage = 0; return;}
    intakeVoltage = voltage;
  }

  public void setClamp(boolean on){
    clampOn = on;
  }
}
