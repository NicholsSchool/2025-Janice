package frc.robot.subsystems.climber;

import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

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

public class Climber extends SubsystemBase {
    private ClimberIO io;
    private final CLimberIOInputsAutoLogged inputs = new CLimberIOInputsAutoLogged();

    public Climber(ClimberIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    
        updateTunables();
    }

    private void updateTunables() {

    }

    public void setClimbState(boolean extended){
        io.setClimbState(extended);
    }
}
