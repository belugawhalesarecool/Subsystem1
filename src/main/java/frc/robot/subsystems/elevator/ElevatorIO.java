package frc.robot.subsystems.elevator;
import org.littletonrobotics.junction.AutoLog;

// this is the interface that the Sim and SparkMax implement

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
       
    }

    public default void setVoltage(double rightVolt, double leftVolt){
    }
    
    public void updateInputs(ElevatorIOInputs inputs) {
        
    }

}