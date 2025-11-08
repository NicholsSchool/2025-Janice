package frc.robot.subsystems.RegionHandler;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.RegionHandler.RegionHandlerIO.RegionHandlerIOInputs;
import frc.robot.util.GamepiecePose;


public class RegionHandler extends SubsystemBase {
    
    private RegionHandlerIO io;
    private RegionHandlerIOInputs inputs = new RegionHandlerIOInputs();

    ArrayList<GamepiecePose> gamepiecePoses = new ArrayList<GamepiecePose>();
    
    public RegionHandler (RegionHandlerIO io){
        this.io = io;
    }
    
    public void periodic(){
        io.updateInputs(inputs);
        handleRegions();
    }

    public void handleRegions(){
        for(int detectedIndex = 0; detectedIndex < inputs.detectedPoses.size(); detectedIndex++){
            for(int regionIndex = 0; regionIndex < gamepiecePoses.size(); regionIndex++){
                if((inputs.detectedPoses.get(detectedIndex).getTranslation().minus(gamepiecePoses.get(regionIndex)
                .getTranslation())).getNorm() < gamepiecePoses.get(regionIndex).refinedRegionRadius){
                    gamepiecePoses.get(regionIndex).updateRegion(inputs.detectedPoses.get(detectedIndex));
                    break;
                }
                if(regionIndex - 1 == gamepiecePoses.size() && gamepiecePoses.get(regionIndex).getTranslation().getNorm() != 0.0){
                    gamepiecePoses.add(inputs.detectedPoses.get(detectedIndex));
                }
            }
        }
        for(int regionIndex = 0; regionIndex < gamepiecePoses.size(); regionIndex++){
            if(gamepiecePoses.get(regionIndex).terminateRegion()){
                gamepiecePoses.remove(regionIndex);
            }
        }
    }
  
}