package frc.robot.util;

import java.util.Collection;
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
   * Returns the mean of the x, y, and rotation from an Collection of Poses. This mean will be in the same units as the input poses.
   * @param pose2ds the poses
   * @return the mean of the poses
   */
  public static Pose2d getMean( final Collection<Pose2d> pose2ds ) {
    double[] sums = { 0.0, 0.0, 0.0 }; // x (meters), y (meters), theta (rad)
    Iterator<Pose2d> iterator = pose2ds.iterator();
    while( iterator.hasNext()) {
      Pose2d pose = iterator.next();

      //sum of poses
      sums[0] += pose.getX();
      sums[1] += pose.getY();
      sums[2] += pose.getRotation().getRadians();
    }
    
    //div by size (n)
    sums[0] /= pose2ds.size();
    sums[1] /= pose2ds.size();
    sums[2] /= pose2ds.size();

    return new Pose2d( sums[0], sums[1], new Rotation2d( sums[2] ) );
  }

  /**
   * Returns the stdDevs in of the poses, given the mean of the poses.
   * @param pose2ds the poses
   * @param meanPose2d the mean of the collection. Used for faster time when already needing to get the mean.
   *  Use {@link #getMean(Collection)} to get the mean of the collection
   * @return the standered deviation of the poses
   */
  public static Pose2d getStdDevs( final Collection<Pose2d> pose2ds, Pose2d meanPose2d ) {
    Iterator<Pose2d> iterator = pose2ds.iterator();

    double[] squaredDifferencesSum = { 0.0, 0.0, 0.0 };
    // sum((num - mean)^2)
    while( iterator.hasNext() ) {
      Pose2d pose = iterator.next();
      squaredDifferencesSum[0] += (pose.getX() - meanPose2d.getX()) * (pose.getX() - meanPose2d.getX());
      squaredDifferencesSum[1] += (pose.getY() - meanPose2d.getY()) * (pose.getY() - meanPose2d.getY());
      squaredDifferencesSum[2] += (pose.getRotation().getRadians() - meanPose2d.getRotation().getRadians())
         * (pose.getRotation().getRadians() - meanPose2d.getRotation().getRadians());
    }

    //div by size of collection (n) for the variance
    squaredDifferencesSum[0] /= pose2ds.size();
    squaredDifferencesSum[1] /= pose2ds.size();
    squaredDifferencesSum[2] /= pose2ds.size();
    
    //sqrt of the variance for std devs
    return new Pose2d( Math.sqrt(squaredDifferencesSum[0]), Math.sqrt(squaredDifferencesSum[1]), 
      new Rotation2d(Math.sqrt(squaredDifferencesSum[2] ) ) );
  }

  /**
   * Holds the meanPose2d and stdDeviationPose2d in an object. Records have default private final fields.
   */
  public record PoseVisionStats( Pose2d meanPose2d, Pose2d stdDevPose2d ) {}


  public static double clip(double input, double lowerBound, double upperBound){
    if(input < lowerBound){
      return lowerBound;
    } else if (input > upperBound){
      return upperBound;
    }else{
      return input;
    }
  }
}
