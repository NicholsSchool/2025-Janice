package frc.robot.subsystems.arm;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

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

public class Arm extends SubsystemBase {
    private ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private final SimpleMotorFeedforward ffModel = new SimpleMotorFeedforward(0.0, 0.2);
  private final ProfiledPIDController armPidController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);

  private double prevVelocity = 0.0;
  private double accelRad = 0.0;

  private double targetAngleDeg = 50.0;
  private double voltageCmdPid = 0.0;
  private boolean reachedTargetPos = true;
  private boolean targetPosSet = false;

  private static final LoggedTunableNumber positionToleranceDeg =
      new LoggedTunableNumber("Arm/PositionToleranceDeg");
  private static final LoggedTunableNumber armMaxVelocityRad =
      new LoggedTunableNumber("Arm/MaxVelocityRad");
  private static final LoggedTunableNumber armMaxAccelerationRad =
      new LoggedTunableNumber("Arm/MaxAccelerationRad");
  private static final LoggedTunableNumber armKp = new LoggedTunableNumber("Arm/Kp");
  private static final LoggedTunableNumber armKi = new LoggedTunableNumber("Arm/Ki");
  private static final LoggedTunableNumber armKd = new LoggedTunableNumber("Arm/Kd");

  public Arm(ArmIO io) {
    this.io = io;

    reachedTargetPos = true;
    positionToleranceDeg.initDefault(2.0);

    armMaxAccelerationRad.initDefault(1.1);
    armMaxVelocityRad.initDefault(0.9);

    armKp.initDefault(ArmConstants.ARM_P);
    armKi.initDefault(ArmConstants.ARM_I);
    armKd.initDefault(ArmConstants.ARM_D);

    armPidController.setP(armKd.get());
    armPidController.setI(armKi.get());
    armPidController.setD(armKd.get());
    armPidController.setTolerance(Units.degreesToRadians(positionToleranceDeg.get()));
  }
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    updateTunables();

    accelRad = (BradyMathLib.avg(inputs.velocityRadsPerSec[0], prevVelocity) / 0.02);
    prevVelocity = BradyMathLib.avg(inputs.velocityRadsPerSec[0], inputs.velocityRadsPerSec[1]);

    if (DriverStation.isDisabled()) {}

    voltageCmdPid = armPidController.calculate(inputs.angleRads)
    // + BradyMathLib.avg(
    // ffModel.calculate(inputs.velocityRadsPerSec[0]),
    ; // ffModel.calculate(inputs.velocityRadsPerSec[1]));

    if (!reachedTargetPos) {
      reachedTargetPos = armPidController.atGoal();
      if (reachedTargetPos) System.out.println("Arm Move to Pos Reached Goal!");
    }

    io.setVoltage(softLimit(voltageCmdPid));
  }
//TODO: change the soft limit values to fit the new arm
  @AutoLogOutput
  public double softLimit(double voltage) {
    if ((inputs.angleDegs >= 80.0 && voltage > 0) || (inputs.angleDegs < 30.0 && voltage < 0)) {
      return 0.0;
    }
    return voltage;
  }

  public void setTargetPos(double targetAngleDeg) {
    this.targetAngleDeg = targetAngleDeg;
    armPidController.setGoal(Units.degreesToRadians(targetAngleDeg));
    armPidController.reset(inputs.angleRads);
    reachedTargetPos = false;
    targetPosSet = true;
  }

  private void updateTunables() {
    // Update from tunable numbers
    if (armMaxVelocityRad.hasChanged(hashCode())
        || armMaxAccelerationRad.hasChanged(hashCode())
        || positionToleranceDeg.hasChanged(hashCode())
        || armKp.hasChanged(hashCode())
        || armKi.hasChanged(hashCode())
        || armKd.hasChanged(hashCode())) {
      armPidController.setP(armKp.get());
      armPidController.setI(armKi.get());
      armPidController.setD(armKd.get());
      armPidController.setConstraints(
          new TrapezoidProfile.Constraints(armMaxVelocityRad.get(), armMaxAccelerationRad.get()));
      armPidController.setTolerance(Units.degreesToRadians(positionToleranceDeg.get()));
    }
  }

  public Command runGoToPosCommand(double targetAngleDeg) {
    if (targetAngleDeg > 90 || targetAngleDeg < 0) {
      System.out.println("JAMES STOP THAT BAD GO TO POS ANGLE");
      return new InstantCommand();
    }
    return new InstantCommand(() -> setTargetPos(targetAngleDeg), this);
  }

  public void setReachedTarget(boolean hasReachedTarget) {
    reachedTargetPos = hasReachedTarget;
  }

  @AutoLogOutput
  public double getAngleDeg() {
    return inputs.angleDegs;
  }

  @AutoLogOutput
  public double getTargetAngleDeg() {
    return targetAngleDeg;
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
  public double[] getOutputCurrent() {
    return inputs.currentAmps;
  }

  @AutoLogOutput
  public boolean isAtGoal() {
    return armPidController.atGoal();
  }

}
