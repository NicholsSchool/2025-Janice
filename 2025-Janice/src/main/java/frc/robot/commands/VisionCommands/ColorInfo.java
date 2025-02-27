// package frc.robot.commands.VisionCommands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.vision.PhotonVision;

// import java.util.List;

// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;
// import org.photonvision.targeting.TargetCorner;

// // import org.photonvision.utils.*;

// public class ColorInfo extends Command {
//   PhotonVision pv1;
//   PhotonVision pv2;

//   public ColorInfo() {
//     pv1 = new PhotonVision("Microsoft_LifeCam_HD-3000 (1)");
//     pv2 = new PhotonVision("Microsoft_LifeCam_HD-3000");
//   }

//   public void pvCornerOne(){
//     PhotonPipelineResult result = pv1.getLatestPipeline();
//     PhotonTrackedTarget target = result.getBestTarget();
//          if (target == null) return;
//          else {
//            List<TargetCorner> corners = target.getMinAreaRectCorners();
//            for( TargetCorner corner : corners )
//             System.out.print( corner.x + "      ");
//        }
//     System.out.println();
//  }
// }
