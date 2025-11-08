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
    //update method for all the regions
    public void handleRegions(){
        for(int detectedIndex = 0; detectedIndex < inputs.detectedPoses.size(); detectedIndex++){
            for(int regionIndex = 0; regionIndex < gamepiecePoses.size(); regionIndex++){
                //run through all the detections and checks if they fall in any existing regions
                if((inputs.detectedPoses.get(detectedIndex).getTranslation().minus(gamepiecePoses.get(regionIndex)
                .getTranslation())).getNorm() < gamepiecePoses.get(regionIndex).refinedRegionRadius){
                    //I have the break in here to speed up the loop 
                    gamepiecePoses.get(regionIndex).updateRegion(inputs.detectedPoses.get(detectedIndex));
                    break;
                }
                //this is just in case it doesn't fit into any existing regions, the translation norm check is to make sure it's not a null detection
                if(regionIndex - 1 == gamepiecePoses.size() && gamepiecePoses.get(regionIndex).getTranslation().getNorm() != 0.0){
                    gamepiecePoses.add(inputs.detectedPoses.get(detectedIndex));
                }
            }
        }
        // checking for whether to remove regions from the list
        for(int regionIndex = 0; regionIndex < gamepiecePoses.size(); regionIndex++){
            if(gamepiecePoses.get(regionIndex).terminateRegion()){
                gamepiecePoses.remove(regionIndex);
            }
        }
    }

    public ArrayList<GamepiecePose> getPoses(){
        return gamepiecePoses;
    }
  
}