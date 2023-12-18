package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import static frc.robot.Constants.Elevator.ElevatorPhysicalConstants.*;
import static frc.robot.Constants.ElectricalLayout.*;
import static frc.robot.Constants.*;
 
public class ElevatorIOSparkMax implements ElevatorIO{

    //Initializes motors, encoders, PID controllers
    private final CANSparkMax elevatorMotor;
    private RelativeEncoder elevatorEncoder;
    private SparkMaxPIDController elevatorPIDController;
    private DutyCycleEncoder absoluteEncoder;
    public double setpoint = 0;

    public ElevatorIOSparkMax() {
        elevatorMotor = new CANSparkMax(ELEVATOR_MOTOR, MotorType.kBrushless);
        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);

        elevatorPIDController = elevatorMotor.getPIDController();
        elevatorPIDController.setOutputRange(-1, 1);

        elevatorEncoder = elevatorMotor.getEncoder();
        elevatorEncoder.setPositionConversionFactor(ELEVATOR_REV_TO_POS_FACTOR);
        elevatorEncoder.setVelocityConversionFactor(ELEVATOR_REV_TO_POS_FACTOR / 60);
        elevatorEncoder.setPosition(0.6);

        absoluteEncoder = new DutyCycleEncoder(0);
        absoluteEncoder.setDistancePerRotation(360.0 / 1024.0);
        absoluteEncoder.setDutyCycleRange(1 / 1024.0, 1023.0 / 1024.0);

        elevatorEncoder.setPosition(absoluteEncoder.getDistance() * 28.45 + 0.6);

    }

    //Not sure if this is needed, as the methods at the bottom seem to do the same thing
    public void setPIDConstants(double p, double i, double d, double ff) {
        elevatorPIDController.setP(p);
        elevatorPIDController.setI(i);
        elevatorPIDController.setD(d);
        elevatorPIDController.setFF(ff);
    }

    //Updates the set of loggable inputs
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionMeters = elevatorEncoder.getPosition();
        inputs.velocityMeters = elevatorEncoder.getVelocity();
        inputs.appliedVolts = elevatorMotor.getAppliedOutput() * elevatorMotor.getBusVoltage();
        inputs.currentAmps = new double[] {elevatorMotor.getOutputCurrent()};
        inputs.tempCelsius = new double[] {elevatorMotor.getMotorTemperature()};
    }

    //Runs motors at correct voltage
    @Override
    public void setVoltage(double voltage) {
        elevatorMotor.setVoltage(voltage);
    }

    //Returns distance measurement
    
    public double getDistance() {
        return elevatorEncoder.getPosition();
    }

    
    //Setpoint variables
    
    public void goToSetpoint(double setpoint) {
        this.setpoint = setpoint;
        elevatorPIDController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    }

    
    public void setBrake(boolean brake) {
        elevatorMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    
    public boolean atSetpoint() {
        return Math.abs(elevatorEncoder.getPosition() - setpoint) < ELEVATOR_PID_TOLERANCE;
    }


    @Override
    //Methods for PID 
    public void setP(double p) {
        elevatorPIDController.setP(p);
    }    
    public void setI(double i) {
        elevatorPIDController.setI(i);
    }
    public void setD(double d) {
        elevatorPIDController.setD(d);
    }
    public void setFF(double ff) {
        elevatorPIDController.setFF(ff);
    }
    public double getP() { return elevatorPIDController.getP();}
    public double getI() { return elevatorPIDController.getI(); }
    public double getD() { return elevatorPIDController.getD(); }
    public double getFF() { return elevatorPIDController.getFF(); }
    
}

