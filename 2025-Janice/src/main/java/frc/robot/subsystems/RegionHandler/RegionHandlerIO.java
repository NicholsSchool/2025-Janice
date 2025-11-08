package frc.robot.subsystems.RegionHandler;

import java.util.ArrayList;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.GamepiecePose;

public interface RegionHandlerIO {
    public static class RegionHandlerIOInputs{
        //for real supply your poses in here with whatever cameras you want just make sure to overwrite the current ones
        ArrayList<GamepiecePose> detectedPoses = new ArrayList<GamepiecePose>();
    }

    public default void updateInputs(RegionHandlerIOInputs inputs) {}
}
