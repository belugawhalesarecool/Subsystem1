package frc.robot.subsystems.elevator;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ElectricalLayout.*;
import static frc.robot.Constants.Elevator.*;
import static frc.robot.Constants.NEO_CURRENT_LIMIT;



import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase {

//where is this coming from?
    private CANSparkMax elevatorMotor;
    private CANSparkMax elevatorFollowerMotor;
    
    public enum State {
        MANUAL
    }

    State state = State.MANUAL;
    private double speed = 0;
    
    public Elevator() {
        elevatorMotor = new CANSparkMax(ELEVATOR_MOTOR, MotorType.kBrushless);
        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);

        elevatorFollowerMotor = new CANSparkMax(ELEVATOR_MOTOR_FOLLOWER, MotorType.kBrushless);
        elevatorFollowerMotor.restoreFactoryDefaults();
        elevatorFollowerMotor.setIdleMode(IdleMode.kBrake);
        elevatorFollowerMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);
        elevatorFollowerMotor.follow(elevatorMotor, false); // following
    }






//where is this coming from?
     @Override
        public void periodic() {
            io.updateInputs(inputs);
            Logger.getInstance().processInputs("Elevator", inputs);
    
            ElevatorMechanism.setLength(io.getDistance());
    
            // 
            if (p.get() != io.getP()) 
                io.setP(p.get());
            
            if (i.get() != io.getI())
                io.setI(i.get());
            
            if (d.get() != io.getD())
                io.setD(d.get());
            
            if (ff.get() != io.getFF())
                io.setFF(ff.get());
            
            Logger.getInstance().processInputs("Elevator", inputs);
        }
//where is this coming from?



    public void setVoltage(double motorVolts) {
        if (io.getDistance() > io.ELEVATOR_MAX_HEIGHT && motorVolts > 0) {
            motorVolts = 0;
        } else if (io.getDistance() < io.ELEVATOR_MIN_HEIGHT && motorVolts < 0) {
            motorVolts = 0;
        }
        io.setVoltage(motorVolts);
    }
    //startPID?
    public void runPID() {
        io.goToSetpoint(setpoint);
    }   

    public boolean atSetpoint() {
        return Math.abs(io.getDistance() - setpoint) < ELEVATOR_TOLERANCE;
    }

//why?
    public void update() {
    switch(state) {
        case MANUAL:
            elevatorMotor.set(speed);
            break;
    }
    }

    public void displayShuffleboard() {

    }
    
    public void tuningInit() {
    
    }
    
    public void tuningPeriodic() {
    
    }

    public void manualControl(double speed){
        this.speed = speed;
        state = State.MANUAL;
    }
    
    public State getState() {
        return state;
    }
//why?


    public void setMechanism(MechanismLigament2d mechanism) {
        ElevatorMechanism = mechanism;
    }

    public MechanismLigament2d append(MechanismLigament2d mechanism) {
        return ElevatorMechanism.append(mechanism);
    }

    public MechanismLigament2d getElevatorMechanism() {
        return new MechanismLigament2d("Elevator", 5, 36, 5, new Color8Bit(Color.kOrange));
    }


    public Command PIDCommand(double setpoint) {
        return new FunctionalCommand(
            () -> setPID(setpoint), 
            () -> runPID(), 
            (stop) -> move(0), 
            this::atSetpoint, 
            this
        );
    }
}
