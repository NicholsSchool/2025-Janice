package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SplineMath {

  private Translation2d waypoint;
  private double slope;
  private Translation2d slopePoint;
  private double angleMod;
  private double c1;
  private double c2;
  private double v2;
  private double v1;
  private double xr;
  private double yr;
  /**
   * creates spline v4 object
   *
   * @param waypoint desired endpoint
   * @param angle angle to be tangent with at the end
   * @param robotPose starting robot pos
   */
  public SplineMath(Translation2d waypoint, double angle, Pose2d robotPose) {
    this.waypoint = waypoint;

    angleMod = Math.abs(angle) == Math.PI / 2 ? angleMod = 0.0001 : 0;

    slope = Math.tan(angle + angleMod);

    slopePoint =
        new Translation2d(
            waypoint.getX() + Math.cos(angle + angleMod),
            waypoint.getY() + Math.sin(angle + angleMod));

    c1 = waypoint.getX();
    v1 = waypoint.getY();
    c2 = slopePoint.getX();
    v2 = slopePoint.getY();
    xr = robotPose.getX();
    yr = robotPose.getY();
  }

  // used to make the line the bot is projected on, tangent to end angle
  private double lineProjection(double x) {
    return slope * (x - waypoint.getX()) + waypoint.getY();
  }

  // closest x value
  public double optimalX() {
    return (c2 * Math.pow(v1, 2)
            + (yr * (c1 - c2) - (c1 + c2) * v2) * v1
            - yr * (c1 - c2) * v2
            + xr * Math.pow(c1 - c2, 2)
            + c1 * Math.pow(v2, 2))
        / (Math.pow(v1, 2)
            - 2 * v2 * v1
            + Math.pow(c1, 2)
            - 2 * c1 * c2
            + Math.pow(c2, 2)
            + Math.pow(v2, 2));
  }

  // distance from optimal x on the line to waypoint
  private double distanceOnLine() {
    return Math.sqrt(Math.pow(c1 - optimalX(), 2) + Math.pow(v1 - lineProjection(optimalX()), 2));
  }

  // how far the bot is from the tangent line
  private double fromLine() {
    return Math.sqrt(Math.pow(xr - optimalX(), 2) + Math.pow(yr - lineProjection(optimalX()), 2));
  }

  // desiredT for point it's correcting to
  private double desiredT() {
    return 1 - fromLine() / distanceOnLine();
  }

  // distance from projected point on the line to the waypoint
  private double projectedDistance() {
    return Math.sqrt(
        Math.pow(desiredT() * (c1 - optimalX()) + optimalX() - c1, 2)
            + Math.pow(
                desiredT() * (v1 - lineProjection(optimalX())) + lineProjection(optimalX()) - v1,
                2));
  }

  /**
   * makes drive vector to follow path
   *
   * @return vector as vector object
   */
  public Translation2d driveVector() {
    double vx;
    double vy;

    // makes sure it doesn't move away from the point, goes normal in this case
    if (distanceOnLine() > projectedDistance()) {
      vx = desiredT() * (c1 - optimalX()) + optimalX() - xr;
      vy = desiredT() * (v1 - lineProjection(optimalX())) + lineProjection(optimalX()) - yr;
    } else {
      vx = optimalX() - xr;
      vy = lineProjection(optimalX()) - yr;
    }

    return new Translation2d(vx / Math.hypot(vx, vy), vy / Math.hypot(vx, vy));
  }

  /**
   * updates with current robot pose
   *
   * @param robotPose current robot pose
   */
  public void update(Pose2d robotPose) {
    this.xr = robotPose.getX();
    this.yr = robotPose.getY();
  }
}
