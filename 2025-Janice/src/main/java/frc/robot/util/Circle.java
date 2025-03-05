package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

public class Circle{
    public double x;
    public double y;
    public double radius;

    public Circle(Translation2d centerLocation, double radius){
        this.x = centerLocation.getX();
        this.y = centerLocation.getY();
        this.radius = radius;
    }



}