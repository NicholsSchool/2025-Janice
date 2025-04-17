package frc.robot.subsystems.elevator;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class Elevator extends SubsystemBase {
    private ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private double accelRad = 0.0;

    private double targetHeight = 0.0;
    private double voltageCmdPid = 0.0;
    private boolean reachedTargetPos = true;
    private double voltageCmdManual = 0.0;
    private boolean hasHitLimitSwitch = false;
    private boolean isAuto = false;

    private enum ElevatorMode{
      kManual,
      kGoToPos
    }

    private ElevatorMode elevatorMode;

     private final ProfiledPIDController elevatorPidController =
      new ProfiledPIDController(
          0,0 ,0, new TrapezoidProfile.Constraints(0,0));


  private static final LoggedTunableNumber elevatorMaxVelocityRad =
      new LoggedTunableNumber("elevator/MaxVelocityRad");
  private static final LoggedTunableNumber elevatorMaxAccelerationRad =
      new LoggedTunableNumber("elevator/MaxAccelerationRad");
  private static final LoggedTunableNumber elevatorKp = new LoggedTunableNumber("elevator/Kp");
  private static final LoggedTunableNumber elevatorKi = new LoggedTunableNumber("elevator/Ki");
  private static final LoggedTunableNumber elevatorKd = new LoggedTunableNumber("elevator/Kd");

    public Elevator(ElevatorIO io){
        this.io = io;

        reachedTargetPos = true;

        elevatorMaxAccelerationRad.initDefault(Constants.ElevatorConstants.ElevatorMaxAccelerationRad);
        elevatorMaxVelocityRad.initDefault(Constants.ElevatorConstants.ElevatorMaxVelocityRad);


        elevatorKp.initDefault(ElevatorConstants.kElevatorP);
        elevatorKi.initDefault(ElevatorConstants.kElevatorI);
        elevatorKd.initDefault(ElevatorConstants.kElevatorD);

        elevatorPidController.setP(elevatorKp.get());
        elevatorPidController.setI(elevatorKi.get());
        elevatorPidController.setD(elevatorKd.get());

        this.elevatorMode = ElevatorMode.kGoToPos;
        setTargetPos(getHeight());
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);

        updateTunables();

        Logger.processInputs("Elevator", inputs);

        switch(elevatorMode){
          case kGoToPos:
          voltageCmdPid = (hasHitLimitSwitch || isAuto) ? elevatorPidController.calculate( this.getHeight()) : 0.0;
          voltageCmdManual = 0.0;
          break;
          case kManual:
          voltageCmdPid = 0.0;
          setTargetPos(getHeight());
          }

         if (!reachedTargetPos) {
            reachedTargetPos = elevatorPidController.atGoal();
            if (reachedTargetPos) System.out.println("elevator Move to Pos Reached Goal!");
          }
        
          if(inputs.limitSwitch){
            hasHitLimitSwitch = true;
          }

        io.setVoltage(voltageCmdManual + voltageCmdPid);
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
  }
  
  public Command runGoToPosCommand(double targetHeight){
    elevatorMode = ElevatorMode.kGoToPos;
    if ((targetHeight > ElevatorConstants.maxHeight || targetHeight < ElevatorConstants.minHeight) && hasHitLimitSwitch) {
      System.out.println("Soft Limited Elevator");
      return new InstantCommand();
    }
    System.out.println("Setting go to pos:" + targetHeight );
    isAuto = DriverStation.isAutonomous();
    return new InstantCommand(() -> setTargetPos(targetHeight), this);
  }

  public void runManualPos(double stickPosition){
    if(Math.abs(stickPosition) > Constants.JOYSTICK_DEADBAND){
      elevatorMode = ElevatorMode.kManual;
    }else{
      elevatorMode = ElevatorMode.kGoToPos;
    }
    

    if(inputs.limitSwitch && stickPosition > 0.0){
      voltageCmdManual = 0.0;
    }else{
      voltageCmdManual = stickPosition * ElevatorConstants.kElevatorManualScaler;
    }
    //}
    
  }

  public void setReachedTarget(boolean hasReachedTarget) {
    reachedTargetPos = hasReachedTarget;
  }

  @AutoLogOutput
  public ElevatorMode getElevatorMode(){
    return elevatorMode;
  }

  @AutoLogOutput
  public double getHeight(){
    return inputs.currentHeight;
  }

  @AutoLogOutput
  public double targetHeight(){
    return targetHeight;
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

  public void startGoToPos(double targetHeight){
    elevatorMode = ElevatorMode.kGoToPos;
    if ((targetHeight < ElevatorConstants.maxHeight || targetHeight > ElevatorConstants.minHeight)) {
      System.out.println("Soft Limited Elevator: " + targetHeight);
    } else {
      System.out.println("Starting go to pos:" + targetHeight );
      setTargetPos(targetHeight);
    }
  }

  public Command commandGoToPos(double targetHeight) {   
    return new FunctionalCommand(
      () -> startGoToPos(targetHeight),
      () -> elevatorMode = ElevatorMode.kGoToPos,
      interrupted -> setTargetPos(inputs.currentHeight),
      () -> this.isAtGoal(),
      this);
  }

}
