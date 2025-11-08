// package frc.robot.subsystems.RegionHandler;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.FunctionalCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import java.util.function.BooleanSupplier;

// import org.littletonrobotics.junction.Logger;

// public class RegionHandler extends SubsystemBase {
    
//     private RegionHandlerIO io;
//     private final RegionHandlerIOInputsAutoLogged inputs = new RegionHandlerIOInputsAutoLogged();
    
//     public RegionHandler (RegionHandlerIO io){
//         this.io = io;
//     }
    
//     public void periodic(){
//         io.updateInputs(inputs);
//         Logger.processInputs("RegionHandler", inputs);
//         if (DriverStation.isDisabled()) {}
//     }
  
// }