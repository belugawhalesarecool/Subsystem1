package frc.robot.subsystems.elevator;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ElectricalLayout.*;
import static frc.robot.Constants.Elevator.*;
import static frc.robot.Constants.NEO_CURRENT_LIMIT;

public class Elevator extends SubsystemBase {
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

  public void setVoltage(double motorVolts) {
    if (io.getDistance() > io.ELEVATOR_MAX_HEIGHT && motorVolts > 0) {
        motorVolts = 0;
    } else if (io.getDistance() < io.ELEVATOR_MIN_HEIGHT && motorVolts < 0) {
        motorVolts = 0;
    }

    io.setVoltage(motorVolts);
}
  
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
}
