// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.SpinAuto;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOSparkMax;
import frc.robot.subsystems.drive.GyroIOReal;

import frc.robot.util.CommandSnailController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final GyroIOReal gyro = GyroIOReal.getInstance();
  // Declaring elevator
  private final Elevator elevator;


  private Mechanism2d mech = new Mechanism2d(3, 3);

  // Controllers
  private final CommandSnailController driver = new CommandSnailController(0);
  private final CommandSnailController operator = new CommandSnailController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

  private boolean isBlue = true;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        drive = new Drive(new DriveIOSparkMax(), new Pose2d());
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        drive = new Drive(new DriveIOSim(), new Pose2d());
        break;

      // Replayed robot, disable IO implementations
      default:
        drive = new Drive(new DriveIO() {
        }, new Pose2d());
        elevator = new Elevator(new ElevatorIO() {
        });
        break;
    }

    // Set up robot state manager

    MechanismRoot2d root = mech.getRoot("elevator", 1, 0.5);
    elevator.setMechanism(root.append(elevator.getElevatorMechanism()));
    laterator.setMechanism(elevator.append(laterator.getLateratorMechanism()));
    // add subsystem mechanisms
    SmartDashboard.putData("Arm Mechanism", mech);

    isBlue = DriverStation.getAlliance() == DriverStation.Alliance.Blue;

    // Set up auto routines
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("Spin", new SpinAuto(drive));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Clear old buttons
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    drive.setDefaultCommand(
        new RunCommand(() -> drive.driveArcade(driver.getDriveForward(), driver.getDriveTurn()), drive));


      //elevator configure button
    elevator.setDefaultCommand(
      new RunCommand(() -> elevator.move(operator.getElevatorSpeed()), elevator));
    operator.getY().onTrue(elevator.PIDCommand(ElevatorPhysicalConstants.ELEVATOR_SETPOINT_EXTEND));
    operator.getA().onTrue(elevator.PIDCommand(ElevatorPhysicalConstants.ELEVATOR_SETPOINT_RETRACT));

    // cancel trajectory
    driver.getY().onTrue(drive.endTrajectoryCommand());
  }


//commands to work with other subsystems

  public CommandBase scoreMid() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            claw.grab(),
            pivotArm.PIDCommand(Constants.PivotArm.PIVOT_ARM_SETPOINT_MID),
            elevator.PIDCommand(ElevatorPhysicalConstants.ELEVATOR_SETPOINT_EXTEND)),
        new WaitCommand(1),
        claw.release());
  }

  public CommandBase scoreLow() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            claw.grab(),
            pivotArm.PIDCommand(Constants.PivotArm.PIVOT_ARM_SETPOINT_BOTTOM),
            elevator.PIDCommand(ElevatorPhysicalConstants.ELEVATOR_SETPOINT_EXTEND)),
        new WaitCommand(1),
        claw.release());
  }

  public CommandBase holdPos() {
    return new RunCommand(() -> {
      claw.release().schedule();
      pivotArm.PIDCommand(Constants.PivotArm.PIVOT_ARM_SETPOINT_HOLD).schedule();
      elevator.PIDCommand(ElevatorPhysicalConstants.ELEVATOR_SETPOINT_RETRACT).schedule();
    }, claw, pivotArm, elevator);
  }

  public CommandBase grabStation() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            claw.release(),
            pivotArm.PIDCommand(Constants.PivotArm.PIVOT_ARM_SETPOINT_TOP),
            elevator.PIDCommand(ElevatorPhysicalConstants.ELEVATOR_SETPOINT_EXTEND)),
        new WaitCommand(1), // back up and then grab
        claw.grab());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
