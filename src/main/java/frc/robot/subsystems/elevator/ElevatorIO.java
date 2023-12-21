package frc.robot.subsystems.elevator;
import org.littletonrobotics.junction.AutoLog;

// this is the interface that the Sim and SparkMax implement

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double positionMeters = 0.0;
        public double velocityMeters = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {};
        public double[] tempCelsius = new double[] {};
    }

    public double ELEVATOR_MAX_HEIGHT = 36.0;
    public double ELEVATOR_MIN_HEIGHT = 0.0;

    //Updates all loggable inputs in case of change
    
    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    //Runs loop at designated voltage, in case of short circuiting 
    public default void setVoltage(double motorVolts) {
    }

    //Returns distance between robot and setpoint
    public default double getDistance() {
        return 0.0;
    }
    //Sets constants for PID
    public default void setPIDConstants(double p, double i, double d, double ff) {
    }

    
    public default void goToSetpoint(double setpoint) {
    }

    public default void setBrake(boolean brake) {
    }

    public default boolean atSetpoint() {
        return false;
    }
    

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