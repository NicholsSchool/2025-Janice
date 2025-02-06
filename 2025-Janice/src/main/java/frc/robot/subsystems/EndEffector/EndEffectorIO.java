package frc.robot.subsystems.EndEffector;

public interface EndEffectorIO {
    
    public static class ElevatorIOInputs{
        public double[] currentAmps = {0.0, 0.0};
        public double[] appliedVolts = {0.0, 0.0};
        public double[] velocityRadPerSec = {0.0, 0.0};
        public double currentHeight = 0.0;
        public boolean inState = false;
    }
      /** Updates the set of loggable inputs. */
  public default void updateInputs(EndEffectorIOInputsAutoLogged inputs) {};
  public default void setVoltage(double voltage) {}
public void updateInputs(EndEffectorIOInputsAutoLogged inputs);;

}