package frc.robot.subsystems.elevator;

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
        public void updateInputs(ElevatorIOInputs inputs) {
            inputs.positionMeters = encoder.getPosition();
            inputs.velocityMeters = encoder.getVelocity();
            inputs.appliedVolts = elevatorMotor.getAppliedOutput() * elevatorMotor.getBusVoltage();
            inputs.currentAmps = new double[] {elevatorMotor.getOutputCurrent()};
            inputs.tempCelsius = new double[] {elevatorMotor.getMotorTemperature()};
    }


}
