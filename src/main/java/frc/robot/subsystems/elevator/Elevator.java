package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import static frc.robot.Constants.Elevator.*;
import static frc.robot.Constants.Elevator.ElevatorPhysicalConstants.ELEVATOR_PID;

public class Elevator extends SubsystemBase {
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

//logs information to disk
    private LoggedDashboardNumber p = new LoggedDashboardNumber("Elevator/P", ELEVATOR_PID[0]);
    private LoggedDashboardNumber i = new LoggedDashboardNumber("Elevator/I", ELEVATOR_PID[1]);
    private LoggedDashboardNumber d = new LoggedDashboardNumber("Elevator/D", ELEVATOR_PID[2]);
    private LoggedDashboardNumber ff = new LoggedDashboardNumber("Elevator/FF", ELEVATOR_PID[3]);

    private double setpoint = 0;

    private final ElevatorIO io;

    // MechanismLigament creates an "canvas" where the mechanism would be drawn in a simulation (See physics simulation)
    //For example, MechanismLigament for elevator would be where it’s attached to the robot’s base.
    private MechanismLigament2d ElevatorMechanism;

    public Elevator(ElevatorIO io) {
        this.io = io;
        SmartDashboard.putData(getName(), this);
    }

    public double highSetpoint() {
        return ElevatorIO.ELEVATOR_MAX_HEIGHT;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Elevator", inputs);

        ElevatorMechanism.setLength(io.getDistance());

        // Updates the PID constants just in case 
        if (p.get() != io.getP()) 
            io.setP(p.get());
        
        if (i.get() != io.getI())
            io.setI(i.get());
        
        if (d.get() != io.getD())
            io.setD(d.get());
        
        if (ff.get() != io.getFF())
            io.setFF(ff.get());
        
        // Logs inputs to disk
        Logger.getInstance().processInputs("Elevator", inputs);
    }

    public void setVoltage(double motorVolts) {
        // sets a limit for the elevator
        if (io.getDistance() > ElevatorIO.ELEVATOR_MAX_HEIGHT && motorVolts > 0) {
            motorVolts = 0;
        } else if (io.getDistance() < ElevatorIO.ELEVATOR_MIN_HEIGHT && motorVolts < 0) {
            motorVolts = 0;
        }

        io.setVoltage(motorVolts);
    }

    public void move(double speed) {
        setVoltage(speed * 12);
    }
//same as startPID
    public void runPID() {
        io.goToSetpoint(setpoint);
    }

    public void setPID(double setpoint) {
        this.setpoint = setpoint;
    }

    public boolean atSetpoint() {
        return Math.abs(io.getDistance() - setpoint) < ELEVATOR_TOLERANCE;
    }
    //Sets for mechanism
    public void setMechanism(MechanismLigament2d mechanism) {
        ElevatorMechanism = mechanism;
    }

    //MechanismLigament2d objects represent each "section"/"stage" of the mechanism, and are based
    // off the root node or another ligament object, as shown here. Each MechanismLigament2d object represents a stage of the mechanism. 
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