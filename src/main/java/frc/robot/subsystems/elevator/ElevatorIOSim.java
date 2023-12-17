package frc.robot.subsystems.elevator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

import static frc.robot.Constants.Elevator.ElevatorSimConstants.*;

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

    // This gearbox represents a gearbox containing 4 Vex 775pro motors.
    private final DCMotor m_elevatorGearbox = DCMotor.getVex775Pro(4);

    private final Encoder m_encoder = new Encoder(kEncoderAChannel, kEncoderBChannel);

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

    private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
    
    public ElevatorIOSim() {
        m_encoder.setDistancePerPulse(kElevatorEncoderDistPerPulse);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        sim.update(0.02);
        m_encoderSim.setDistance(sim.getPositionMeters());
        inputs.positionMeters = sim.getPositionMeters();
        inputs.velocityMeters = sim.getVelocityMetersPerSecond();
        inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
        // inputs.voltage = new double[] {sim.getBusVoltageV()};
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
