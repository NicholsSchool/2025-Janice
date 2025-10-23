package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SplineV2Math {
    
    public static Translation2d splineTwo(double x, double y, Translation2d desiredPos, Pose2d drivePos){
        if(desiredPos.equals(new Translation2d())){
            return new Translation2d(x,y);
        }

        Rotation2d theta = new Rotation2d(-Math.atan2(y,x) + Math.PI / 2);
        Translation2d offsetDriveTranslation = (drivePos.getTranslation().minus(desiredPos)).rotateBy(theta);

        Translation2d driveVector = new Translation2d(1, 2 * offsetDriveTranslation.getY() / offsetDriveTranslation.getX());

        driveVector.times(Math.hypot(x,y) / driveVector.getNorm());

        return(driveVector.rotateBy(theta.times(-1.0)));
    }

}
