package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.DriveToReef.ReefDirection;
import frc.robot.util.ReefPosition;

public class AutoOffsets {
    public List<ReefOffset> reefOffsets = new ArrayList<>();
    
    public class ReefOffset {
        public ReefPosition reefPosition;
        public Translation2d translation2d;

        public ReefOffset (ReefPosition reefPosition, Translation2d translation2d) {
            this.reefPosition = reefPosition;
            this.translation2d = translation2d;
        }
    }

    public AutoOffsets(){
        reefOffsets.add(new ReefOffset(new ReefPosition(6, ReefDirection.LEFT), new Translation2d()));
        reefOffsets.add(new ReefOffset(new ReefPosition(6, ReefDirection.RIGHT), new Translation2d()));

        reefOffsets.add(new ReefOffset(new ReefPosition(7, ReefDirection.LEFT), new Translation2d()));
        reefOffsets.add(new ReefOffset(new ReefPosition(7, ReefDirection.RIGHT), new Translation2d()));

        reefOffsets.add(new ReefOffset(new ReefPosition(8, ReefDirection.LEFT), new Translation2d()));
        reefOffsets.add(new ReefOffset(new ReefPosition(8, ReefDirection.RIGHT), new Translation2d()));

        reefOffsets.add(new ReefOffset(new ReefPosition(9, ReefDirection.LEFT), new Translation2d()));
        reefOffsets.add(new ReefOffset(new ReefPosition(9, ReefDirection.RIGHT), new Translation2d()));

        reefOffsets.add(new ReefOffset(new ReefPosition(10, ReefDirection.LEFT), new Translation2d()));
        reefOffsets.add(new ReefOffset(new ReefPosition(10, ReefDirection.RIGHT), new Translation2d()));

        reefOffsets.add(new ReefOffset(new ReefPosition(11, ReefDirection.LEFT), new Translation2d()));
        reefOffsets.add(new ReefOffset(new ReefPosition(11, ReefDirection.RIGHT), new Translation2d()));

        reefOffsets.add(new ReefOffset(new ReefPosition(17, ReefDirection.LEFT), new Translation2d()));
        reefOffsets.add(new ReefOffset(new ReefPosition(17, ReefDirection.RIGHT), new Translation2d()));

        reefOffsets.add(new ReefOffset(new ReefPosition(18, ReefDirection.LEFT), new Translation2d()));
        reefOffsets.add(new ReefOffset(new ReefPosition(18, ReefDirection.RIGHT), new Translation2d()));

        reefOffsets.add(new ReefOffset(new ReefPosition(19, ReefDirection.LEFT), new Translation2d()));
        reefOffsets.add(new ReefOffset(new ReefPosition(19, ReefDirection.RIGHT), new Translation2d()));

        reefOffsets.add(new ReefOffset(new ReefPosition(20, ReefDirection.LEFT), new Translation2d()));
        reefOffsets.add(new ReefOffset(new ReefPosition(20, ReefDirection.RIGHT), new Translation2d()));

        reefOffsets.add(new ReefOffset(new ReefPosition(21, ReefDirection.LEFT), new Translation2d()));
        reefOffsets.add(new ReefOffset(new ReefPosition(21, ReefDirection.RIGHT), new Translation2d()));

        reefOffsets.add(new ReefOffset(new ReefPosition(22, ReefDirection.LEFT), new Translation2d()));
        reefOffsets.add(new ReefOffset(new ReefPosition(22, ReefDirection.RIGHT), new Translation2d()));
    }

    public Translation2d getTranslation(ReefPosition reefPosition){
        for(ReefOffset reefOffset : reefOffsets){
            if(reefOffset.reefPosition.tagID == reefPosition.tagID && reefOffset.reefPosition.reefDirection == reefPosition.reefDirection){
                return reefOffset.translation2d;
            }
        }
        System.out.println("૮(˶ㅠ︿ㅠ)ა   " + reefPosition.tagID + "  " + reefPosition.reefDirection);
        return new Translation2d();
    }
    
}
