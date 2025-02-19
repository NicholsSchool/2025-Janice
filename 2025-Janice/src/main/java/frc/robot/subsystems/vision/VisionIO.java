package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface VisionIO {

    default void updateInputs( VisionIOInputs inputs ) {}

    @AutoLog
    class VisionIOInputs {
        public boolean[] connected = { false, false };
        public boolean[] hasDetection = { false, false };
        public PhotonTrackedTarget[] bestTarget = { new PhotonTrackedTarget(), new PhotonTrackedTarget() };
        public List<PhotonTrackedTarget> allTargets = new ArrayList<>();
    }
}