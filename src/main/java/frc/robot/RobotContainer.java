package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick codriver = new Joystick(1);

   /* Driver Controls */
	private final int translationAxis = XboxController.Axis.kLeftY.value;
	private final int strafeAxis = XboxController.Axis.kLeftX.value;
	private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    private final JoystickButton dampen = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    private final JoystickButton DynamicLock = new JoystickButton(driver, XboxController.Button.kX.value);

    private final Trigger forwardHold = new Trigger(() -> (driver.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.6));
    private final Trigger backwardHold = new Trigger(() -> (driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.6));

    /* CoDriver Buttons */

    private final JoystickButton setTargetL1 = new JoystickButton(codriver, 1);
    private final JoystickButton setTargetL2 = new JoystickButton(codriver, 2);
    private final JoystickButton setTargetL3 = new JoystickButton(codriver, 3);
    private final JoystickButton setTargetL4 = new JoystickButton(codriver, 4);
    private final JoystickButton setTargetIntake = new JoystickButton(codriver, 5);
    private final JoystickButton toggleElevator = new JoystickButton(codriver, 6);
    private final JoystickButton climberDown = new JoystickButton(codriver, 9);
    private final JoystickButton climberUp = new JoystickButton(codriver, 10);

    // private final JoystickButton incElevatorState = new JoystickButton(codriver, 1);
    // private final JoystickButton decElevatorState = new JoystickButton(codriver, 2);
    // private final JoystickButton toggleElevator = new JoystickButton(codriver, 3);
 
    /* Subsystems */
    private final PoseEstimator s_PoseEstimator = new PoseEstimator();
    private final Swerve s_Swerve = new Swerve(s_PoseEstimator);
    private final Climber s_Climber = new Climber();
    private final Elevator s_Elevator = new Elevator();
    private final Vision s_Vision = new Vision(s_PoseEstimator);
    private final CoralIntake s_CoralIntake = new CoralIntake();
    private final AlgaeIntake s_AlgaeIntake = new AlgaeIntake();
    private final Pneumatics s_Pneumatics = new Pneumatics();

    /* AutoChooser */
    private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Pneumatics.Init();

        // Default commands
        s_Swerve.setDefaultCommand(
            new SwerveCommand(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> dampen.getAsBoolean(),
                () -> 0 // Dynamic heading placeholder
            )
        );

        s_Climber.setDefaultCommand(Commands.run(() -> s_Climber.setSpeed(0), s_Climber));

        // ELEVATOR STATES
        s_Elevator.setDefaultCommand(
            new ElevatorCommand(
                s_Elevator, 
                () -> codriver.getRawAxis(Joystick.kDefaultYChannel)
            )
        );

        s_CoralIntake.setDefaultCommand(
            new CoralCommand(
                s_CoralIntake,
                () -> (codriver.getRawAxis(Joystick.kDefaultXChannel) * 0.5) + 0.1
            )
        );

        s_AlgaeIntake.setDefaultCommand(
            new AlgaeCommand(
                s_AlgaeIntake,
                () -> (codriver.getRawAxis(Joystick.kDefaultXChannel) * 0.7) + 0.2
            )
        );

        // Configure the button bindings
        configureButtonBindings();


        //Pathplanner commands - templates
        new EventTrigger("intake coral").whileTrue(s_CoralIntake.run(() -> s_CoralIntake.setSpeed(0.7)));
        new EventTrigger("outtake coral").whileTrue(s_CoralIntake.run(() -> s_CoralIntake.setSpeed(-0.5)));
        new EventTrigger("set intake").onTrue(Commands.runOnce(() -> States.mElevatorState = States.ElevatorStates.intake));
        new EventTrigger("set l1").onTrue(Commands.runOnce(() -> States.mElevatorState = States.ElevatorStates.l1));
        new EventTrigger("set l2").onTrue(Commands.runOnce(() -> States.mElevatorState = States.ElevatorStates.l2));
        new EventTrigger("set l3").onTrue(Commands.runOnce(() -> States.mElevatorState = States.ElevatorStates.l3));
        new EventTrigger("set l4").onTrue(Commands.runOnce(() -> States.mElevatorState = States.ElevatorStates.l4));
    
        
        //Auto chooser
        autoChooser = AutoBuilder.buildAutoChooser("New Auto"); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
//////* Driver Buttons *//////
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

    //Heading lock bindings
        forwardHold.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.forwardHold)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
        );
        backwardHold.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.backwardHold)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
        );
        DynamicLock.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.DynamicLock)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
        );
//////* CoDriver Buttons *//////
        // incElevatorState.onTrue(
        //     new InstantCommand(() -> {
        //         if(States.mElevatorState != States.ElevatorStates.values()[States.ElevatorStates.values().length - 1]) {
        //             States.mElevatorState = States.ElevatorStates.values()[States.mElevatorState.ordinal() + 1];
        //         }
        //     })
        // );
        // decElevatorState.onTrue(
        //     new InstantCommand(() -> {
        //         if(States.mElevatorState != States.ElevatorStates.values()[0]) {
        //             States.mElevatorState = States.ElevatorStates.values()[States.mElevatorState.ordinal() - 1];
        //         }
        //     })
        // );
        setTargetL1.onTrue(new InstantCommand(() -> {States.mElevatorState = States.ElevatorStates.l1;}));
        setTargetL2.onTrue(new InstantCommand(() -> {States.mElevatorState = States.ElevatorStates.l2;}));
        setTargetL3.onTrue(new InstantCommand(() -> {States.mElevatorState = States.ElevatorStates.l3;}));
        setTargetL4.onTrue(new InstantCommand(() -> {States.mElevatorState = States.ElevatorStates.l4;}));
        setTargetIntake.onTrue(new InstantCommand(() -> {States.mElevatorState = States.ElevatorStates.intake;}));
        toggleElevator.onTrue(
            new InstantCommand(() -> {
                States.mElevatorToggle = !States.mElevatorToggle;
            })
        );
        climberDown.whileTrue(
            new InstantCommand(() -> {
                s_Climber.setSpeed(-0.5);
            })
        );
        climberUp.whileTrue(
            new InstantCommand(() -> {
                s_Climber.setSpeed(0.5);
            })
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }
}
