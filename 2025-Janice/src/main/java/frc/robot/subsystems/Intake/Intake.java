package frc.robot.subsystems.Intake;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.BradyMathLib;

public class Intake extends SubsystemBase {
    
    private IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private double prevVelocity = 0.0;
    private double accelRad = 0.0;

    private boolean hasCoralAligned = false;

    public Intake (IntakeIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    accelRad = (BradyMathLib.avg(inputs.velocityRadsPerSec, prevVelocity) / 0.02);
    prevVelocity = inputs.velocityRadsPerSec;

    if (DriverStation.isDisabled()) {}

  }

  public void startIntake(){

    hasCoralAligned = false;

    io.setVoltage(-1.0);
    io.setSolenoidState(true);
  }

  public void intakePeriodic(){

    if (inputs.hasCoralAligned) { hasCoralAligned = true; }

    io.setVoltage(this.hasCoralAligned ? 1.0 : -1.0);
    io.setSolenoidState(hasCoralAligned);

  }

  public void stopIntake(){
    io.setSolenoidState(false);
    io.setVoltage(0.0);
  }


  @AutoLogOutput
  public double getAcceleration() {
    return accelRad;
  }

  @AutoLogOutput
  public double getOutputCurrent() {
    return inputs.currentAmps;
  }
}
