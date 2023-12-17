package frc.robot.subsystems.elevator;
import org.littletonrobotics.junction.AutoLog;

// this is the interface that the Sim and SparkMax implement

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {}

    public default void setVoltage(double rightVolt, double leftVolt){}
    
    // Updates the set of loggable inputs. 
    public void updateInputs(ElevatorIOInputs inputs) {}

    //Methods for PID 
    public default void setP(double p) {}    
    public default void setI(double i) {}
    public default void setD(double d) {}
    public default void setFF(double ff) {}
    public default double getP() { return 0.0; }
    public default double getI() { return 0.0; }
    public default double getD() { return 0.0; }
    public default double getFF() { return 0.0; }
}