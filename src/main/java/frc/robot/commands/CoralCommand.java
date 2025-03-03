package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.States;

public class CoralCommand extends Command {
    private CoralIntake s_CoralIntake;
    private DoubleSupplier intakeSpeed;

    public CoralCommand(CoralIntake s_CoralIntake, DoubleSupplier speedSup) {
        this.s_CoralIntake = s_CoralIntake;
        this.intakeSpeed = speedSup;
        addRequirements(s_CoralIntake);
    }

    @Override
    public void execute() {
        s_CoralIntake.setSpeed(intakeSpeed.getAsDouble());

        switch(States.mElevatorState){
            case intake:
                s_CoralIntake.setAngle(Value.kForward);
                break;
            case l1:
                s_CoralIntake.setAngle(Value.kReverse);
                break;
            case l2:
                s_CoralIntake.setAngle(Value.kReverse);
                break;
            case l3:
                s_CoralIntake.setAngle(Value.kReverse);
                break;
            case l4:
                s_CoralIntake.setAngle(Value.kReverse);
        }
    }
}
