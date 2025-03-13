package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
    private final Joystick buttonPanel = new Joystick(2);

   /* Driver Controls */
	private final int translationAxis = XboxController.Axis.kLeftY.value;
	private final int strafeAxis = XboxController.Axis.kLeftX.value;
	private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    private final JoystickButton dampen = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    private final Trigger alignLeft = new Trigger(() -> (driver.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.6));
    private final Trigger alignRight = new Trigger(() -> (driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.6));

    /* CoDriver Xbox Controller */
    private final int intakeCoral = XboxController.Button.kA.value;
    private final int outtakeCoral = XboxController.Button.kB.value;
    private final int intakeAlgae = XboxController.Button.kX.value;
    private final int outtakeAlgae = XboxController.Button.kY.value;

    private final Trigger raiseAlgae = new Trigger(() -> (codriver.getPOV() == 0));
    private final Trigger lowerAlgae = new Trigger(() -> (codriver.getPOV() == 180));
    
    /* CoDriver Button Panel */
    private final JoystickButton setTargetL1 = new JoystickButton(buttonPanel, 1);
    private final JoystickButton setTargetL2 = new JoystickButton(buttonPanel, 2);
    private final JoystickButton setTargetAL = new JoystickButton(buttonPanel, 7);
    private final JoystickButton setTargetL3 = new JoystickButton(buttonPanel, 3);
    private final JoystickButton setTargetAH = new JoystickButton(buttonPanel, 8);
    private final JoystickButton setTargetL4 = new JoystickButton(buttonPanel, 4);
    private final JoystickButton setTargetIntake = new JoystickButton(buttonPanel, 5);
    private final JoystickButton toggleElevator = new JoystickButton(buttonPanel, 6);
 
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

        // ELEVATOR STATES
        s_Elevator.setDefaultCommand(
            new ElevatorCommand(
                s_Elevator, 
                () -> (codriver.getRawAxis(XboxController.Axis.kRightTrigger.value) - codriver.getRawAxis(XboxController.Axis.kLeftTrigger.value))
            )
        );

        s_CoralIntake.setDefaultCommand(
            new CoralCommand(
                s_CoralIntake,
                () -> codriver.getRawButton(intakeCoral),
                () -> codriver.getRawButton(outtakeCoral)
            )
        );

        s_AlgaeIntake.setDefaultCommand(
            new AlgaeCommand(
                s_AlgaeIntake,
                () -> codriver.getRawButton(intakeAlgae),
                () -> codriver.getRawButton(outtakeAlgae)
            )
        );

        s_Climber.setDefaultCommand(Commands.run(
            () -> s_Climber.setSpeed(-buttonPanel.getRawAxis(Joystick.kDefaultYChannel)),
            s_Climber
        ));

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

        alignLeft.whileTrue(new AprilAlign(s_Swerve, "right"));
        alignRight.whileTrue(new AprilAlign(s_Swerve, "left"));

//////* CoDriver Buttons *//////
        setTargetL1.onTrue(new InstantCommand(() -> {States.mElevatorState = States.ElevatorStates.l1;}));
        setTargetL2.onTrue(new InstantCommand(() -> {States.mElevatorState = States.ElevatorStates.l2;}));
        setTargetAL.onTrue(new InstantCommand(() -> {
            States.mElevatorState = States.ElevatorStates.aL;
            s_AlgaeIntake.setAngle(Value.kReverse);
        }));
        setTargetL3.onTrue(new InstantCommand(() -> {States.mElevatorState = States.ElevatorStates.l3;}));
        setTargetAH.onTrue(new InstantCommand(() -> {
            States.mElevatorState = States.ElevatorStates.aH;
            s_AlgaeIntake.setAngle(Value.kReverse);
        }));
        setTargetL4.onTrue(new InstantCommand(() -> {States.mElevatorState = States.ElevatorStates.l4;}));
        setTargetIntake.onTrue(new InstantCommand(() -> {States.mElevatorState = States.ElevatorStates.intake;}));
        toggleElevator.onTrue(
            new InstantCommand(() -> {
                States.mElevatorToggle = !States.mElevatorToggle;
            })
        );
        raiseAlgae.onTrue(new InstantCommand(() -> s_AlgaeIntake.setAngle(Value.kForward)));
        lowerAlgae.onTrue(new InstantCommand(() -> s_AlgaeIntake.setAngle(Value.kReverse)));
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
