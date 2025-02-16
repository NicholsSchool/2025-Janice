package frc.robot.util;

import java.util.ArrayDeque;
import java.util.Iterator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class BradyMathLib {
  public static double avg(double num1, double num2) {
    return (num1 + num2) / 2;
  }
  
  public static Pose2d getMean( final ArrayDeque<Pose2d> arrayDeque ) {
    double[] sums = { 0.0, 0.0, 0.0 }; // x (meters), y (meters), theta (rad)
    Iterator<Pose2d> iterator = arrayDeque.iterator();
    while( iterator.hasNext()) {
      sums[0] += iterator.next().getX();
      sums[1] += iterator.next().getY();
      sums[2] += iterator.next().getRotation().getRadians();
    }
    
    sums[0] /= arrayDeque.size();
    sums[1] /= arrayDeque.size();
    sums[2] /= arrayDeque.size();

    return new Pose2d( sums[0], sums[1], new Rotation2d( sums[2] ) );
  }

  public static Pose2d getStdDevs( final ArrayDeque<Pose2d> arrayDeque, Pose2d meanPose2d ) {
    Iterator<Pose2d> iterator = arrayDeque.iterator();

    double[] squaredDifferencesSum = { 0.0, 0.0, 0.0 }; // x (meters), y (meters), theta (rad)
    while( iterator.hasNext()) {
      squaredDifferencesSum[0] += (iterator.next().getX() - meanPose2d.getX()) * (iterator.next().getX() - meanPose2d.getX());
      squaredDifferencesSum[1] += (iterator.next().getY() - meanPose2d.getY()) * (iterator.next().getY() - meanPose2d.getY());
      squaredDifferencesSum[2] += (iterator.next().getRotation().getRadians() - meanPose2d.getRotation().getRadians())
         * (iterator.next().getRotation().getRadians() - meanPose2d.getRotation().getRadians());
    }

    squaredDifferencesSum[0] /= arrayDeque.size(); //variance
    squaredDifferencesSum[1] /= arrayDeque.size();
    squaredDifferencesSum[2] /= arrayDeque.size();
    return new Pose2d( Math.sqrt(squaredDifferencesSum[0]), Math.sqrt(squaredDifferencesSum[1]), 
      new Rotation2d(Math.sqrt(squaredDifferencesSum[2] ) ) );
  }

  public record PoseVisionStats( Pose2d meanPose2d, Pose2d stdDevPose2d ) {}
}
