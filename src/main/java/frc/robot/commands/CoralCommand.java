package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.States;

public class CoralCommand extends Command {
    private CoralIntake s_CoralIntake;
    private BooleanSupplier intakeSup;
    private BooleanSupplier outtakeSup;

    public CoralCommand(CoralIntake s_CoralIntake, BooleanSupplier intakeSup, BooleanSupplier outtakeSup) {
        this.s_CoralIntake = s_CoralIntake;
        this.intakeSup = intakeSup;
        this.outtakeSup = outtakeSup;
        addRequirements(s_CoralIntake);
    }

    @Override
    public void execute() {
        if (intakeSup.getAsBoolean()) {
            s_CoralIntake.setSpeed(1);
        } else if (outtakeSup.getAsBoolean()) {
            s_CoralIntake.setSpeed(-0.5);
        } else {
            s_CoralIntake.setSpeed(0.1);
        }

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
            case aL:
                s_CoralIntake.setAngle(Value.kForward);
                break;
            case l3:
                s_CoralIntake.setAngle(Value.kReverse);
                break;
            case aH:
                s_CoralIntake.setAngle(Value.kForward);
                break;
            case l4:
                s_CoralIntake.setAngle(Value.kReverse);
        }
    }
}
