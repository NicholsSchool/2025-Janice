package frc.robot.util;

import java.util.ArrayDeque;
import java.util.Iterator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Math utils
 */
public class BradyMathLib {

  /**
   * Averages two doubles because I could not find this anywhere
   * @param num1 one num
   * @param num2 second num
   * @return the average of the two numbers
   */
  public static double avg(double num1, double num2) {
    return (num1 + num2) / 2;
  }
  
  /**
   * Returns the mean of the x, y, and Rotation from an ArrayDeque of Poses.
   * @param arrayDeque the poses
   * @return the mean of the poses in a Pose2d object
   */
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

  /**
   * Returns the stdDevs in a Pose2d object of the poses given the mean of the poses.
   * @param arrayDeque the poses
   * @param meanPose2d the mean of the arrayDeque in a pose2d object. Used for faster time when already needing to get the mean. 
   *  Use {@link #getMean(ArrayDeque)} to get the mean of the arrayDeque.
   * @return the standered deviation of the poses in a Pose2d object
   */
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

  /**
   * Holds the meanPose2d and stdDeviationPose2d in an object. Records have default private final fields.
   */
  public record PoseVisionStats( Pose2d meanPose2d, Pose2d stdDevPose2d ) {}
}
