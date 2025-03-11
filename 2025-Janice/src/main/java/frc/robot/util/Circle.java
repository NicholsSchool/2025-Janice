package frc.robot.util;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;

public class Circle{
    public double x;
    public double y;
    public double radius;

    public Circle(Supplier<Translation2d> centerLocation, DoubleSupplier radius){
        this.x = centerLocation.get().getX();
        this.y = centerLocation.get().getY();
        this.radius = radius.getAsDouble();
    }

    public Circle(Translation2d centerLocation, double radius){
        this.x = centerLocation.getX();
        this.y = centerLocation.getY();
        this.radius = radius;
    }


}