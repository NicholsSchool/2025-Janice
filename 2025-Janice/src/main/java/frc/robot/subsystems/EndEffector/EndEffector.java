package frc.robot.subsystems.EndEffector;

import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class EndEffector extends SubsystemBase {
    
    private EndEffectorIO io;
    private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

    private double spin = 0.0;
    private double voltageCmdPid = 0.0;

    private boolean open = false;

    private final ProfiledPIDController endEffectorPidController =
      new ProfiledPIDController(
          0,0 ,0, new TrapezoidProfile.Constraints(0,0));

    private static final LoggedTunableNumber EndEffectorMaxVelocityRad =
      new LoggedTunableNumber("EndEffector/MaxVelocityRad");
  private static final LoggedTunableNumber EndEffectorMaxAccelerationRad =
      new LoggedTunableNumber("EndEffector/MaxAccelerationRad");

      private static final LoggedTunableNumber EndEffectorKp = new LoggedTunableNumber("EndEffector/Kp");
  private static final LoggedTunableNumber EndEffectorKi = new LoggedTunableNumber("EndEffector/Ki");
  private static final LoggedTunableNumber EndEffectorKd = new LoggedTunableNumber("EndEffector/Kd");

  public EndEffector(EndEffectorIO io){
    this.io = io;

    EndEffectorMaxAccelerationRad.initDefault(1.1);
    EndEffectorMaxVelocityRad.initDefault(0.9);


    EndEffectorKp.initDefault(EndEffectorConstants.kEndEffectorP);
    EndEffectorKi.initDefault(EndEffectorConstants.kEndEffectorI);
    EndEffectorKd.initDefault(EndEffectorConstants.kEndEffectorD);

    endEffectorPidController.setP(EndEffectorKp.get());
    endEffectorPidController.setI(EndEffectorKi.get());
    endEffectorPidController.setD(EndEffectorKd.get());
}
//TODO: Add to this
public void periodic(){
    io.updateInputs(inputs);

    updateTunables();

    Logger.processInputs("End Effector", inputs);

     if (DriverStation.isDisabled()) {}


    io.setVoltage(voltageCmdPid);
}

private void updateTunables() {
    // Update from tunable numbers
    if (EndEffectorMaxVelocityRad.hasChanged(hashCode())
        || EndEffectorMaxAccelerationRad.hasChanged(hashCode())
        || EndEffectorKp.hasChanged(hashCode())
        || EndEffectorKi.hasChanged(hashCode())
        || EndEffectorKd.hasChanged(hashCode())) {
            endEffectorPidController.setP(EndEffectorKp.get());
            endEffectorPidController.setI(EndEffectorKi.get());
            endEffectorPidController.setD(EndEffectorKd.get());
            endEffectorPidController.setConstraints(
          new TrapezoidProfile.Constraints(EndEffectorMaxVelocityRad.get(), EndEffectorMaxAccelerationRad.get()));
    }
  }

  @AutoLogOutput
  public double getVoltageCommandPid() {
    return voltageCmdPid;
  }

  @AutoLogOutput
  public double[] getOutputCurrent() {
    return inputs.currentAmps;
  }
}
