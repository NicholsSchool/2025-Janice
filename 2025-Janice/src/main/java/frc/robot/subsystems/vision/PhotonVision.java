package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.Constants.VisionConstants;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class PhotonVision extends SubsystemBase {
  PhotonCamera camera;

  public PhotonVision(String key) {
    this.camera = new PhotonCamera(key);
  }

  // Have to use the same pipeline result each time you want to gather data.
  // Gets the processed data from the camera
  public PhotonPipelineResult getLatestPipeline() {
    return camera.getLatestResult();
  }
  // Checks if there is a target in vision
  public boolean hasTarget(PhotonPipelineResult result) {
    return result.hasTargets();
  }

  public List<PhotonTrackedTarget> getTargets(PhotonPipelineResult result) {
    return result.getTargets();
  }

  public PhotonTrackedTarget getBestTarget(PhotonPipelineResult result) {
    return result.getBestTarget();
  }

  // The yaw of the target in degrees (positive right)
  public double getYaw(PhotonTrackedTarget target) {
    return target.getYaw();
  }

  // The pitch of the target in degrees (positive up)
  public double getPitch(PhotonTrackedTarget target) {
    return target.getPitch();
  }

  // The area (how much of the camera feed the bounding box takes up) as a percent (0-100)
  public double getArea(PhotonTrackedTarget target) {
    return target.getArea();
  }

  // The skew of the target in degrees (counter-clockwise positive)
  public double getSkew(PhotonTrackedTarget target) {
    return target.getSkew();
  }

  // The 4 corners of the minimum bounding box rectangle
  public List<TargetCorner> getBoundingCorners(PhotonTrackedTarget target) {
    return target.getDetectedCorners();
  }

  // The camera to target transform (Pose)
  // For some reason cannot get pose for reflectiveTape
  // public Transform2d getPose(PhotonTrackedTarget target){
  //     return target.getCameraToTarget();
  // }

  // Get id of tag
  public int getTargetId(PhotonTrackedTarget target) {
    if(target == null){ return -1;} 
    return target.getFiducialId();
  }

  // How ambiguous the pose is????
  public double getPoseAbmiguity(PhotonTrackedTarget target) {
    return target.getPoseAmbiguity();
  }

  /*Get the transform that maps camera space (X = forward, Y = left, Z = up)
  to object/fiducial tag space (X forward, Y left, Z up) with the lowest reprojection error*/
  public Transform3d getBestCamera(PhotonTrackedTarget target) {
    return target.getBestCameraToTarget();
  }

  /*Get the transform that maps camera space (X = forward, Y = left, Z = up)
  to object/fiducial tag space (X forward, Y left, Z up) with the lowest highest error*/
  public Transform3d getAlternateCamera(PhotonTrackedTarget target) {
    return target.getAlternateCameraToTarget();
  }

  public Pose3d getLocalizedPose(){
    PhotonPipelineResult result = getLatestPipeline();
    PhotonTrackedTarget target = result.getBestTarget();
    if(target == null){ return new Pose3d();}
    else{
      // this yaw from them was outputting off by 180 from what it actually was, might be worth investigating if localization doesn't work
    return PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
     FieldConstants.aprilTags.getTagPose(target.getFiducialId()).orElse(new Pose3d()),
      VisionConstants.cameraOnePosition);
    }
  }


  // @Override
  // public void periodic(){
  //     DistanceFromTag d = new DistanceFromTag();
  //     d.execute();
  // }

}
