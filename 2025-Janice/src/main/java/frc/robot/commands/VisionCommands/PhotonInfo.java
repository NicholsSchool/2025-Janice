package frc.robot.commands.VisionCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.PhotonVision;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// import org.photonvision.utils.*;

public class PhotonInfo extends Command {
  PhotonVision pv1;

  public PhotonInfo() {
    pv1 = new PhotonVision("Arducam_OV2311_USB_Camera (1) (2) (3) (4)");
    // rcommnad = new RamseteCommand(null, null, null, null, null, null, null, null, null, null);
  }

  public int getIDOne() {
    PhotonPipelineResult result = pv1.getLatestPipeline();
    PhotonTrackedTarget target = result.getBestTarget();
    if (target == null) return -1;
    else {
      return target.getFiducialId();
    }
  }

  public Translation2d getXYOne(double yaw) {
    PhotonPipelineResult result = pv1.getLatestPipeline();
    PhotonTrackedTarget target = result.getBestTarget();
    double x =
        Math.hypot(target.getBestCameraToTarget().getX(), target.getBestCameraToTarget().getY());
    return new Translation2d(
        x,
        target.getBestCameraToTarget().getY()
            + x * Math.sin(target.getBestCameraToTarget().getRotation().getZ()));
  }

//   public Translation2d translateToCenter(Translation2d toCenter, double yaw, Translation2d xy) {
//     return new Translation2d(
//         xy.getX()
//             - VisionConstants.cameraOnePosition.getX() * Math.cos(yaw)
//             - VisionConstants.cameraOnePosition.getY() * Math.sin(yaw),
//         xy.getY()
//             - VisionConstants.cameraOnePosition.getY() * Math.cos(yaw)
//             - VisionConstants.cameraOnePosition.getX() * Math.sin(yaw));
//   }

//   public Translation2d makeTagPose(Translation2d xy, double yaw, int tagID) {
//     Pose2d tagPose =
//         new Pose2d(
//             FieldConstants.aprilTags.getTagPose(tagID).get().getX(),
//             FieldConstants.aprilTags.getTagPose(tagID).get().getY(),
//             new
// Rotation2d(FieldConstants.aprilTags.getTagPose(tagID).get().getRotation().getZ()));
//     return new Translation2d(
//         Math.cos(tagPose.getRotation().getRadians()) * xy.getX()
//             + Math.sin(tagPose.getRotation().getRadians()) * xy.getY()
//             + tagPose.getX(),
//         Math.cos(tagPose.getRotation().getRadians()) * xy.getY()
//             + Math.sin(tagPose.getRotation().getRadians()) * xy.getX()
//             + tagPose.getY());
//   }

//   public Pose2d getTagPose(double yaw, Pose2d robotPose) {
//     return new Pose2d(
//         makeTagPose(
//             translateToCenter(VisionConstants.cameraOnePosition, yaw, getXYOne(yaw)),
//             yaw,
//             getIDOne()),
//         new Rotation2d(yaw));
//   }
}
