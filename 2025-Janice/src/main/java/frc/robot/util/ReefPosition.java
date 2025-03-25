package frc.robot.util;

import frc.robot.commands.DriveToReef.ReefDirection;

public class ReefPosition {
    public int tagID;
    public ReefDirection reefDirection;

    public ReefPosition(int tagID, ReefDirection reefDirection){
        this.tagID = tagID;
        this.reefDirection = reefDirection;
    }
}
