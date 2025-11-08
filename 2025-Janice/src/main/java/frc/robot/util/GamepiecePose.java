package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.RegionConstants;

public class GamepiecePose {
    //This is the object we're using to store our gamepiece pose. Since there is ambiguity with the cameras and the bot pose,
    // I'm defining the pieces in a region that gradually improves confidence with weighted averages. When the gamepiece hasn't
    // been seen in the region for a while it destroys the region in the list.

    public Translation2d gamepieceTranslation;
    
    public double lastSeen = 0.0;
    public double refinedRegionRadius = RegionConstants.minRegionRadius;
    public double maxConfidence = 0.1;
    /**
     * constructs a gamepiece pose for region handler
     * @param translation first location of the gamepiece 
     * @param confidence TF confidence value
     */
    public GamepiecePose(Translation2d translation, double confidence){
        gamepieceTranslation = translation;
        //this is so if it gets a really unconfident first hit it doesn't make a bunch of tiny regions around it
        //as it gets more confident the region shinks to our min radius
        this.refinedRegionRadius = RegionConstants.minRegionRadius / confidence;
    }

    public void updateRegion(Translation2d translation, double confidence){
        //decrease our region if we get a good hit
        if(confidence > maxConfidence){
            refinedRegionRadius = RegionConstants.minRegionRadius / confidence;
            maxConfidence = confidence;
        }

        if(translation.getNorm() != 0.0){
            //weighted average with confidence
            gamepieceTranslation = (gamepieceTranslation.plus(translation.times(confidence))).div(confidence + 1.0);
            lastSeen = 0;
        }else{
            // TODO: update with loop time intervals
            lastSeen = lastSeen + 1.0;
        }
    }

    public Translation2d getTranslation(){
        return gamepieceTranslation;
    }
    // if it goes for too long without seeing any data remove the region
    public boolean terminateRegion(){
        return (lastSeen - RegionConstants.terminateTime) > 0.0;
    }
    


}
