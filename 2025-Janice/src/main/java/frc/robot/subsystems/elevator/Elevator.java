package frc.robot.subsystems.elevator;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class Elevator extends SubsystemBase{
    private ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private double prevVelocity = 0.0;
    private double accelRad = 0.0;

    private double targetHeight = 0.0;
    private double voltageCmdPid = 0.0;
    private boolean reachedTargetPos = true;
    private boolean targetPosSet = false;

     private final ProfiledPIDController elevatorPidController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);


  private static final LoggedTunableNumber elevatorMaxVelocityRad =
      new LoggedTunableNumber("elevator/MaxVelocityRad");
  private static final LoggedTunableNumber elevatorMaxAccelerationRad =
      new LoggedTunableNumber("elevator/MaxAccelerationRad");
  private static final LoggedTunableNumber elevatorKp = new LoggedTunableNumber("elevator/Kp");
  private static final LoggedTunableNumber elevatorKi = new LoggedTunableNumber("elevator/Ki");
  private static final LoggedTunableNumber elevatorKd = new LoggedTunableNumber("elevator/Kd");

    Elevator(ElevatorIO io){
        this.io = io;

        reachedTargetPos = true;

        elevatorMaxAccelerationRad.initDefault(1.1);
        elevatorMaxVelocityRad.initDefault(0.9);


        elevatorKp.initDefault(ElevatorConstants.kElevatorP);
        elevatorKi.initDefault(ElevatorConstants.kElevatorI);
        elevatorKd.initDefault(ElevatorConstants.kElevatorD);

        elevatorPidController.setP(elevatorKp.get());
        elevatorPidController.setI(elevatorKi.get());
        elevatorPidController.setD(elevatorKd.get());
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);

        updateTunables();

        Logger.processInputs("elevator", inputs);

         if (DriverStation.isDisabled()) {}

         voltageCmdPid = elevatorPidController.calculate(inputs.currentHeight);

         if (!reachedTargetPos) {
            reachedTargetPos = elevatorPidController.atGoal();
            if (reachedTargetPos) System.out.println("elevator Move to Pos Reached Goal!");
          }
        io.setVoltage(voltageCmdPid);
    }


  private void updateTunables() {
    // Update from tunable numbers
    if (elevatorMaxVelocityRad.hasChanged(hashCode())
        || elevatorMaxAccelerationRad.hasChanged(hashCode())
        || elevatorKp.hasChanged(hashCode())
        || elevatorKi.hasChanged(hashCode())
        || elevatorKd.hasChanged(hashCode())) {
      elevatorPidController.setP(elevatorKp.get());
      elevatorPidController.setI(elevatorKi.get());
      elevatorPidController.setD(elevatorKd.get());
      elevatorPidController.setConstraints(
          new TrapezoidProfile.Constraints(elevatorMaxVelocityRad.get(), elevatorMaxAccelerationRad.get()));
    }
  }


  public void setTargetPos(double targetHeight) {
    this.targetHeight = targetHeight;
    elevatorPidController.setGoal((targetHeight));
    elevatorPidController.reset(inputs.currentHeight);
    reachedTargetPos = false;
    targetPosSet = true;
  }
  
  public Command runGoToPosCommand(double targetHeight){
    if (targetHeight > ElevatorConstants.maxHeight || targetHeight < ElevatorConstants.minHeight) {
      return new InstantCommand();
    }
    return new InstantCommand(() -> setTargetPos(targetHeight), this);
  }


  public void setReachedTarget(boolean hasReachedTarget) {
    reachedTargetPos = hasReachedTarget;
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
    return elevatorPidController.atGoal();
  }

}
