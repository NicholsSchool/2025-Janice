package main.java.frc.robot.subsystems.Laterator;



public class Laterator extends SubsystemBase{
    
    private LateratorIO io;
    private final LateratorIOInputsAutoLogged inputs = new LateratorIOInputsAutoLogged(); 
    private LateratorMode lateratorMode;

    enum LateratorMode{
        OUTTAKE,
        INTAKE,
        STOP
    }

    public Laterator(LateratorIO io){
        this.io = io;
    }

    
}
