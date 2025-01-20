package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Consumer;

// Create a voltage command ramp to ramp up a motor in positive, then negative direction.
// Used for testing.
public class VoltageCommandRamp extends Command {
  private static final double kDelaySecs = 2.0;

  private final Consumer<Double> voltageConsumer;
  private final Timer timer = new Timer();
  private double rampVoltsPerSec = 0.0;
  private double endVolts = 0.0;

  private final int kStateStartDelay = 0;
  private final int kStatePositive = 1;
  private final int kStateTransitionDelay = 2;
  private final int kStateNegative = 3;
  private final int kStateEnd = 4;
  private int state = kStateStartDelay;

  public VoltageCommandRamp(
      Subsystem subsystem,
      Consumer<Double> voltageConsumer,
      double rampVoltsPerSec,
      double endVolts) {
    addRequirements(subsystem);
    this.voltageConsumer = voltageConsumer;
    this.rampVoltsPerSec = rampVoltsPerSec;
    this.endVolts = endVolts;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    this.state = kStateStartDelay;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (state) {
      case kStatePositive:
      case kStateNegative:
        double voltage = timer.get() * rampVoltsPerSec;
        double direction = (this.state == kStatePositive) ? 1.0 : -1.0;
        voltageConsumer.accept(voltage * direction);
        if (voltage >= endVolts) {
          state++;
          timer.reset();
        }
        break;
      case kStateStartDelay:
      case kStateTransitionDelay:
        if (timer.get() < kDelaySecs) {
          voltageConsumer.accept(0.0);
        } else {
          state++;
          timer.reset();
        }
        break;
      case kStateEnd:
        voltageConsumer.accept(0.0);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    voltageConsumer.accept(0.0);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (state == kStateEnd);
  }
}
