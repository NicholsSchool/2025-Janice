package frc.robot.util;

public class CurrentDrawDesparity {

  public static boolean isDesparity(double current1, double current2, double check) {

    double min = Math.min(Math.abs(current1), Math.abs(current2));
    double max = Math.max(Math.abs(current1), Math.abs(current2));

    if (max == 0) {
      return false;
    } else {
      double ratio = min / max;
      if (ratio <= check) {
        return true;
      }
    }
    return false;
  }
}
