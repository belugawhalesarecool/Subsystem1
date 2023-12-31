package frc.robot.subsystems.elevator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import com.revrobotics.SparkMaxPIDController;

import static frc.robot.Constants.Elevator.ElevatorSimConstants.*;

// Used to simulate the subsystem
public class ElevatorIOSim implements ElevatorIO{
    
    private final ProfiledPIDController m_controller =
        new ProfiledPIDController(
            kElevatorKp,
            kElevatorKi,
            kElevatorKd,
            new TrapezoidProfile.Constraints(2.45, 2.45)
        );
    
    ElevatorFeedforward m_feedforward =
        new ElevatorFeedforward(
            kElevatorkS,
            kElevatorkG,
            kElevatorkV,
            kElevatorkA
        );

    //represents a gearbox containing 4 Vex 775pro motors.
    private final DCMotor m_elevatorGearbox = DCMotor.getVex775Pro(4);

    private final Encoder m_encoder = new Encoder(kEncoderAChannel, kEncoderBChannel);

    //Creates object for simulation
    private final ElevatorSim sim =
        new ElevatorSim(
            m_elevatorGearbox,
            kElevatorGearing,
            kCarriageMass,
            kElevatorDrumRadius,
            kMinElevatorHeightMeters,
            kMaxElevatorHeightMeters,
            false,
            VecBuilder.fill(0.01)
        );

    //Sets encoder up
    private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
    
    public ElevatorIOSim() {
        m_encoder.setDistancePerPulse(kElevatorEncoderDistPerPulse);
    }
    //Updates inputs 
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        sim.update(0.02);
        m_encoderSim.setDistance(sim.getPositionMeters());
        inputs.positionMeters = sim.getPositionMeters();
        inputs.velocityMeters = sim.getVelocityMetersPerSecond();
        inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
    }

    //Runs loop at designated voltage
    @Override
    public void setVoltage(double motorVolts) {
        sim.setInputVoltage(motorVolts);
    }

    //
    @Override
    public void goToSetpoint(double setpoint) {
        m_controller.setGoal(setpoint);
        double pidOutput = m_controller.calculate(m_encoder.getDistance());
        double feedForwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
        sim.setInputVoltage(feedForwardOutput + pidOutput);
    }
    
    //Returns distance between robot and setpoint
     @Override
    public double getDistance() {
        return sim.getPositionMeters();
    }

    //Sets a true or false statement for whether the robot has reached the goal or not. 
    @Override
    public boolean atSetpoint() {
        return m_controller.atGoal();
    }

 
    @Override
    public void setP(double p) {
        m_controller.setP(p);
    }

    @Override
    public void setI(double i) {
        m_controller.setI(i);
    }

    @Override
    public void setD(double d) {
        m_controller.setD(d);
    }

    @Override
    public void setFF(double ff) {
        m_feedforward = new ElevatorFeedforward(kElevatorkS, kElevatorkG, ff, kElevatorkA);
    }

    @Override
    public double getP() {
        return m_controller.getP();
    }

    @Override
    public double getI() {
        return m_controller.getI();
    }

    @Override
    public double getD() {
        return m_controller.getD();
    }

    @Override
    public double getFF() {
       return m_feedforward.calculate(m_controller.getSetpoint().velocity);
    }

    


}



