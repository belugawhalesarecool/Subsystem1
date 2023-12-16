gpackage frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.UPDATE_PERIOD;

public class ElevatorIOSparkMax implements ElevatorIO{
    
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
        
    
        /*leftEncoder.setPositionConversionFactor(Math.PI * DRIVE_WHEEL_DIAM_M / DRIVE_GEARBOX_REDUCTION);
        rightEncoder.setPositionConversionFactor(Math.PI * DRIVE_WHEEL_DIAM_M / DRIVE_GEARBOX_REDUCTION);
        leftEncoder.setVelocityConversionFactor(Math.PI * DRIVE_WHEEL_DIAM_M / DRIVE_GEARBOX_REDUCTION / 60.0);
        rightEncoder.setVelocityConversionFactor(Math.PI * DRIVE_WHEEL_DIAM_M / DRIVE_GEARBOX_REDUCTION / 60.0);*/
    
        elevatorEncoder.setPosition(0);
      }

    @Override
    public void setPIDConstants(double p, double i, double d, double ff) {
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setFF(ff);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionMeters = encoder.getPosition();
        inputs.velocityMeters = encoder.getVelocity();
        inputs.appliedVolts = elevatorMotor.getAppliedOutput() * elevatorMotor.getBusVoltage();
        inputs.currentAmps = new double[] {elevatorMotor.getOutputCurrent()};
        inputs.tempCelsius = new double[] {elevatorMotor.getMotorTemperature()};
    }

       
    @Override
    public void setVoltage(double motorVolts) {
        elevatorMotor.setVoltage(motorVolts);
    }

    
    @Override
    public double getDistance() {
        return encoder.getPosition();
    }

    
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
    public void setP(double p) {
        pidController.setP(p);
    }

    @Override
    public void setI(double i) {
        pidController.setI(i);
    }

    @Override
    public void setD(double d) {
        pidController.setD(d);
    }

    @Override
    public void setFF(double ff) {
        pidController.setFF(ff);
    }

    @Override
    public double getP() {
        return pidController.getP();
    }

    @Override
    public double getI() {
        return pidController.getI();
    }

    @Override
    public double getD() {
        return pidController.getD();
    }

    @Override
    public double getFF() {
        return pidController.getFF();
    }

    


}
