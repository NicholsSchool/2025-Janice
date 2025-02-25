package frc.robot.subsystems.Intake;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import edu.wpi.first.math.util.Units;
import frc.robot.util.BradyMathLib;

public class Intake extends SubsystemBase {
    
    private IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private final SimpleMotorFeedforward ffModel = new SimpleMotorFeedforward(0.0, 0.2);
    private final ProfiledPIDController armPidController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);


    private double prevVelocity = 0.0;
    private double accelRad = 0.0;
    private double voltageCmdPid = 0.0;


    private static final LoggedTunableNumber intakeMaxVelocityRad =
        new LoggedTunableNumber("Arm/MaxVelocityRad");
    private static final LoggedTunableNumber intakeMaxAccelerationRad =
        new LoggedTunableNumber("Arm/MaxAccelerationRad");
    private static final LoggedTunableNumber intakeKp = new LoggedTunableNumber("Intake/Kp");
    private static final LoggedTunableNumber intakeKi = new LoggedTunableNumber("Intake/Ki");
    private static final LoggedTunableNumber intakeKd = new LoggedTunableNumber("Intake/Kd");

    public Intake (IntakeIO io) {
    this.io = io;

    intakeMaxAccelerationRad.initDefault(1.1);
    intakeMaxVelocityRad.initDefault(0.9);

    intakeKp.initDefault(IntakeConstants.INTAKE_P);
    intakeKi.initDefault(IntakeConstants.INTAKE_I);
    intakeKd.initDefault(IntakeConstants.INTAKE_D);

    armPidController.setP(intakeKp.get());
    armPidController.setI(intakeKi.get());
    armPidController.setD(intakeKd.get());
    //armPidController.setTolerance(Units.degreesToRadians(positionToleranceDeg.get()));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    updateTunables();

    accelRad = (BradyMathLib.avg(inputs.velocityRadsPerSec, prevVelocity) / 0.02);
    prevVelocity = inputs.velocityRadsPerSec;

    if (DriverStation.isDisabled()) {}

    //TODO: voltageCmdPid = armPidController.calculate(inputs.angleRads)
    // + BradyMathLib.avg(
    // ffModel.calculate(inputs.velocityRadsPerSec[0]),
    ; // ffModel.calculate(inputs.velocityRadsPerSec[1]));

    //io.setVoltage(softLimit(voltageCmdPid));
  }

  private void updateTunables() {
    // Update from tunable numbers
    if (intakeMaxVelocityRad.hasChanged(hashCode())
        || intakeMaxAccelerationRad.hasChanged(hashCode())
        /*|| positionToleranceDeg.hasChanged(hashCode())*/
        || intakeKp.hasChanged(hashCode())
        || intakeKi.hasChanged(hashCode())
        || intakeKd.hasChanged(hashCode())) {
      armPidController.setP(intakeKp.get());
      armPidController.setI(intakeKi.get());
      armPidController.setD(intakeKd.get());
      armPidController.setConstraints(
          new TrapezoidProfile.Constraints(intakeMaxVelocityRad.get(), intakeMaxAccelerationRad.get()));
      //armPidController.setTolerance(Units.degreesToRadians(positionToleranceDeg.get()));
    }
  }

  @AutoLogOutput
  public double getAcceleration() {
    return accelRad;
  }

  @AutoLogOutput
  public double getVoltageCommandPid() {
    return voltageCmdPid;
  }

  @AutoLogOutput
  public double getOutputCurrent() {
    return inputs.currentAmps;
  }
}
