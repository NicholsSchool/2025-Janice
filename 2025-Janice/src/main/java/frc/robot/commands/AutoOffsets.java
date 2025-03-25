package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.DriveToReef.ReefDirection;
import frc.robot.util.ReefPosition;

public class AutoOffsets {
    public List<ReefOffset> reefOffsets = new ArrayList<>();
    public List<ReefOffsetParallel> reefOffsetsParallel = new ArrayList<>();
    
    public class ReefOffset {
        public ReefPosition reefPosition;
        public Translation2d translation2d;

        public ReefOffset (ReefPosition reefPosition, Translation2d translation2d) {
            this.reefPosition = reefPosition;
            this.translation2d = translation2d;
        }
    }

    public class ReefOffsetParallel {
        public ReefPosition reefPosition;
        public double offset;

        public ReefOffsetParallel (ReefPosition reefPosition, double offset) {
            this.reefPosition = reefPosition;
            this.offset = offset;
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

        // new table!!!!!!

        //positive is more far from center

        reefOffsetsParallel.add(new ReefOffsetParallel(new ReefPosition(6, ReefDirection.LEFT), 0));
        reefOffsetsParallel.add(new ReefOffsetParallel(new ReefPosition(6, ReefDirection.RIGHT), 0));

        reefOffsetsParallel.add(new ReefOffsetParallel(new ReefPosition(7, ReefDirection.LEFT), 0));
        reefOffsetsParallel.add(new ReefOffsetParallel(new ReefPosition(7, ReefDirection.RIGHT), 0));

        reefOffsetsParallel.add(new ReefOffsetParallel(new ReefPosition(8, ReefDirection.LEFT), 0));
        reefOffsetsParallel.add(new ReefOffsetParallel(new ReefPosition(8, ReefDirection.RIGHT), 0));

        reefOffsetsParallel.add(new ReefOffsetParallel(new ReefPosition(9, ReefDirection.LEFT), 0));
        reefOffsetsParallel.add(new ReefOffsetParallel(new ReefPosition(9, ReefDirection.RIGHT), 0));

        reefOffsetsParallel.add(new ReefOffsetParallel(new ReefPosition(10, ReefDirection.LEFT), 0));
        reefOffsetsParallel.add(new ReefOffsetParallel(new ReefPosition(10, ReefDirection.RIGHT), 0));

        reefOffsetsParallel.add(new ReefOffsetParallel(new ReefPosition(11, ReefDirection.LEFT), 0));
        reefOffsetsParallel.add(new ReefOffsetParallel(new ReefPosition(11, ReefDirection.RIGHT), 0));

        reefOffsetsParallel.add(new ReefOffsetParallel(new ReefPosition(17, ReefDirection.LEFT), 0));
        reefOffsetsParallel.add(new ReefOffsetParallel(new ReefPosition(17, ReefDirection.RIGHT), 0));

        reefOffsetsParallel.add(new ReefOffsetParallel(new ReefPosition(18, ReefDirection.LEFT), 0));
        reefOffsetsParallel.add(new ReefOffsetParallel(new ReefPosition(18, ReefDirection.RIGHT), 0));

        reefOffsetsParallel.add(new ReefOffsetParallel(new ReefPosition(19, ReefDirection.LEFT), 0));
        reefOffsetsParallel.add(new ReefOffsetParallel(new ReefPosition(19, ReefDirection.RIGHT), 0));

        reefOffsetsParallel.add(new ReefOffsetParallel(new ReefPosition(20, ReefDirection.LEFT), 0));
        reefOffsetsParallel.add(new ReefOffsetParallel(new ReefPosition(20, ReefDirection.RIGHT), 0));

        reefOffsetsParallel.add(new ReefOffsetParallel(new ReefPosition(21, ReefDirection.LEFT), 0));
        reefOffsetsParallel.add(new ReefOffsetParallel(new ReefPosition(21, ReefDirection.RIGHT), 0));

        reefOffsetsParallel.add(new ReefOffsetParallel(new ReefPosition(22, ReefDirection.LEFT), 0));
        reefOffsetsParallel.add(new ReefOffsetParallel(new ReefPosition(22, ReefDirection.RIGHT), 0));
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

    public double getOffset(int tagID, ReefDirection reefDirection){
        for(ReefOffsetParallel reefOffsetParallel : reefOffsetsParallel){
            if(reefOffsetParallel.reefPosition.tagID == tagID && reefOffsetParallel.reefPosition.reefDirection == reefDirection){
                return reefOffsetParallel.offset;
            }
        }
        System.out.println("૮(˶ㅠ︿ㅠ)ა   " + tagID + "  " + reefDirection);
        return 0.0;
    }
    
}
