package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.States;

public class AlgaeCommand extends Command {
    private AlgaeIntake s_AlgaeIntake;
    private DoubleSupplier intakeSpeed;

    public AlgaeCommand(AlgaeIntake s_AlgaeIntake, DoubleSupplier speedSup) {
        this.s_AlgaeIntake = s_AlgaeIntake;
        this.intakeSpeed = speedSup;
        addRequirements(s_AlgaeIntake);
    }

    @Override
    public void execute() {
        s_AlgaeIntake.setSpeed(intakeSpeed.getAsDouble());

        switch(States.mElevatorState){
            case intake:
                s_AlgaeIntake.setAngle(Value.kForward);
                break;
            case l1:
                s_AlgaeIntake.setAngle(Value.kReverse);
                break;
            case l2:
                s_AlgaeIntake.setAngle(Value.kReverse);
                break;
            case l3:
                s_AlgaeIntake.setAngle(Value.kReverse);
                break;
            case l4:
                s_AlgaeIntake.setAngle(Value.kReverse);
        }
    }
}
