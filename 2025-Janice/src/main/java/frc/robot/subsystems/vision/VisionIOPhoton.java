package frc.robot.subsystems.vision;

import frc.robot.Constants.VisionConstants;

public class VisionIOPhoton implements VisionIO {
    public final PhotonVision[] cameras;

    public VisionIOPhoton() {
        cameras = new PhotonVision[2];
        cameras[0] = (new PhotonVision(VisionConstants.cameraKey));
        cameras[1] = new PhotonVision("INSERT KEY HERE");
    }

    public VisionIOPhoton( String... cameraKeys ) {
        cameras = new PhotonVision[2];
        for( int i = 0; i < cameraKeys.length; i++ )
            cameras[i] = new PhotonVision( cameraKeys[i] );
    }

    public void updateInputs( VisionIOInputs inputs ) {
        inputs.connected[0] = cameras[0].camera.isConnected();
        inputs.connected[1] = cameras[1].camera.isConnected();

        inputs.hasDetection[0] = cameras[0].hasValidTarget();
        inputs.hasDetection[1] = cameras[1].hasValidTarget();

        inputs.bestTarget[0] = cameras[0].getBestTarget(cameras[0].getLatestPipeline());
        inputs.bestTarget[1] = cameras[1].getBestTarget(cameras[1].getLatestPipeline());

        inputs.allTargets = cameras[0].getTargets(cameras[0].getLatestPipeline());
        inputs.allTargets.addAll(cameras[1].getTargets(cameras[1].getLatestPipeline()));
    }
}
