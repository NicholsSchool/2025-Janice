package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SplineV2Math {

        /*
        The rotation for the drive method should be PID to center the gamepiece
        in the limelight also in here we will need a convert TA and TX to pos whether
        that needs filtering is another question, but the driving should be able to adapt
        only time will tell though and a lot of experimenting
        maybe so the driver feels more in control, add a manual rotation to the limelight PID though this is optional
         */
    public static Translation2d splineTwo(double x, double y, Translation2d desiredPos, Pose2d drivePos){
            if(desiredPos.equals(new Translation2d())){
                //joystick y and field y aren't the same
                return new Translation2d(x,y);
            }
    
            Rotation2d theta = new Rotation2d(-Math.atan2(y,x) + Math.PI / 2);
            Translation2d offsetDriveTranslation = (drivePos.getTranslation().minus(desiredPos)).rotateBy(theta);
    
            Translation2d driveVector = new Translation2d(1, 2 * offsetDriveTranslation.getY() / offsetDriveTranslation.getX());
    
            driveVector.times(Math.hypot(x,y) / driveVector.getNorm());
    
            return(driveVector.rotateBy(theta.times(-1.0)));
    }

    public static Translation2d splineTwoVision(double x, double y, double TX, double TA, Pose2d drivePos){
        if(TA == 0.0){
            //joystick y and field y aren't the same
            return new Translation2d(x,y);
        }

        Rotation2d theta = new Rotation2d(-Math.atan2(y,x) + Math.PI / 2);
        Translation2d offsetDriveTranslation = (drivePos.getTranslation().minus(objectPos(TX, TA, drivePos))).rotateBy(theta);

        Translation2d driveVector = new Translation2d(1, 2 * offsetDriveTranslation.getY() / offsetDriveTranslation.getX());

        driveVector.times(Math.hypot(x,y) / driveVector.getNorm());

        return(driveVector.rotateBy(theta.times(-1.0)));
    }

    public static Translation2d objectPos(double TX, double TA, Pose2d drivePos){
        double distance = 1; //(insert distance function here, this will be gotten by TA and some sort of regression)
        double theta = Math.toRadians(TX + drivePos.getRotation().getDegrees()); //calibration needed also check if TX is rad or deg
        return drivePos.getTranslation().plus(new Translation2d(distance * Math.cos(theta), distance * Math.sin(theta)));

    }

}
