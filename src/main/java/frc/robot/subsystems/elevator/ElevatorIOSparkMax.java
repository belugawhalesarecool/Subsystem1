package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ElevatorIOSparkMax implements ElevatorIO{
    
    private final CANSparkMax leftLeader;
    private final CANSparkMax rightFollower;
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

}
