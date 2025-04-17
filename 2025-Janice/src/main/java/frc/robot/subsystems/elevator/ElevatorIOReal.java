package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class ElevatorIOReal implements ElevatorIO{
    private TalonFX lShoulder;
    private TalonFX rShoulder;
    private DigitalInput elevatorLimitSwitch;
    private final CANcoder elevatorEncoder;

    public ElevatorIOReal() {
    lShoulder = new TalonFX(CAN.kLeftChain, "Elevator");
    rShoulder = new TalonFX(CAN.kRightChain, "Elevator");
    elevatorEncoder = new CANcoder(CAN.elevatorEncoder, "Elevator");

    var talonConfig = new TalonFXConfiguration();
    talonConfig.CurrentLimits.StatorCurrentLimit = Constants.ElevatorConstants.ElevatorCurrentLimit;
    talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    lShoulder.getConfigurator().apply(talonConfig);
    rShoulder.setControl(new Follower(CAN.kLeftChain, false));

    elevatorLimitSwitch = new DigitalInput(Constants.ElevatorConstants.elevatorLimitSwitchChannel);
  }

  private double getHeight(){
    if(!elevatorLimitSwitch.get() && elevatorEncoder.getPosition().getValueAsDouble() != 0.0 ){
      elevatorEncoder.setPosition(0.0);
    }
    return elevatorEncoder.getPosition().getValueAsDouble();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs){
    inputs.currentHeight = this.getHeight();
    inputs.appliedVolts = new double[] {lShoulder.getMotorVoltage().getValueAsDouble(), rShoulder.getMotorVoltage().getValueAsDouble()};
    inputs.velocityRadPerSec = new double[] {lShoulder.getVelocity().getValueAsDouble(), rShoulder.getVelocity().getValueAsDouble()};
    inputs.currentAmps = new double[] {lShoulder.getStatorCurrent().getValueAsDouble(), lShoulder.getStatorCurrent().getValueAsDouble()};
    inputs.limitSwitch = getLimitSwitch();
  }

  @Override
  public void setVoltage(double voltage) {
    lShoulder.setVoltage(voltage);
  }

  @AutoLogOutput
  public boolean getLimitSwitch() {
    return !elevatorLimitSwitch.get();
  }

  @AutoLogOutput
  public double getElevatorHeight() {
    return elevatorEncoder.getPosition().getValueAsDouble();
  }
  
}
