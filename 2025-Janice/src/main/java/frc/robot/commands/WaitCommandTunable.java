package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

/**
 * A command that does nothing but takes a specified amount of time to finish. Uses a double
 * supplier so that the delay time can be variable.
 */
public class WaitCommandTunable extends Command {
  /** The timer used for waiting. */
  protected Timer m_timer = new Timer();

  private double m_duration;
  private final DoubleSupplier m_doubleSupplier;

  /**
   * Creates a new WaitCommand. This command will do nothing, and end after the specified duration.
   *
   * @param seconds the time to wait, in seconds
   */
  @SuppressWarnings("this-escape")
  public WaitCommandTunable(DoubleSupplier doubleSupplier) {
    m_doubleSupplier = doubleSupplier;
    SendableRegistry.setName(this, getName() + ": supplier " + doubleSupplier);
  }

  @Override
  public void initialize() {
    m_duration = m_doubleSupplier.getAsDouble();
    m_timer.restart();
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_duration);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("duration", () -> m_duration, null);
  }
}
