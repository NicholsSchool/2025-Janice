package frc.robot.subsystems.arm;

public class ArmIOSim implements ArmIO {

  double motorVoltage = 0.0;

  public void updateInputs(ArmIOInputs inputs) {
    inputs.motorVoltage = this.motorVoltage;
  }

  public void setVoltage(double voltage) { motorVoltage = voltage; }
}