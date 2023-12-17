gpackage frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import static frc.robot.Constants.ElectricalLayout.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.Elevator.ElevatorPhysicalConstants.*;
// Used to run the subsystem physically
 
public class ElevatorIOSparkMax implements ElevatorIO{

    //Initializes motors, encoders, PID controllers
    private final CANSparkMax elevatorMotor;
    private final CANSparkMax elevatorMotorFollower;
    private RelativeEncoder elevatorEncoder;
    private SparkMaxPIDController elevatorPIDController;

    public ElevatorIOSparkMax() {
        elevatorMotor = new CANSparkMax(1, MotorType.kBrushless);
        elevatorMotorFollower = new CANSparkMax(4, MotorType.kBrushless);
        configureEncoders();

    }

    private void configureEncoders() {
        elevatorEncoder = elevatorMotor.getEncoder();
        elevatorEncoder.setPosition(0);
    }

    //Not sure if this is needed, as the methods at the bottom seem to do the same thing
    @Override
    public void setPIDConstants(double p, double i, double d, double ff) {
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setFF(ff);
    }

    //Updates the set of loggable inputs
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionMeters = encoder.getPosition();
        inputs.velocityMeters = encoder.getVelocity();
        inputs.appliedVolts = elevatorMotor.getAppliedOutput() * elevatorMotor.getBusVoltage();
        inputs.currentAmps = new double[] {elevatorMotor.getOutputCurrent()};
        inputs.tempCelsius = new double[] {elevatorMotor.getMotorTemperature()};
    }

    //Runs motors at correct voltage
    @Override
    public void setVoltage(double rightVolt, double leftVolt ) {
        elevatorMotor.setVoltage(rightVolt, leftVolt);
    }

    //Returns distance measurement
    @Override
    public double getDistance() {
        return encoder.getPosition();
    }

    
    //Setpoint variables
    @Override
    public void goToSetpoint(double setpoint) {
        this.setpoint = setpoint;
        pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void setBrake(boolean brake) {
        elevatorMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public boolean atSetpoint() {
        return Math.abs(encoder.getPosition() - setpoint) < ELEVATOR_PID_TOLERANCE;
    }


    @Override
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

