package frc.robot.subsystems.Rumbler;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Rumbler extends SubsystemBase {

    CommandXboxController controller;

    public Rumbler(CommandXboxController controller){
        this.controller = controller;
    }

    @Override
    public void periodic(){
    }

    public void setRumble(RumbleType rumbleType, double strength){
        controller.setRumble(rumbleType, strength);
    }

}
