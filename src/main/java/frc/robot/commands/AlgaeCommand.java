package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;

public class AlgaeCommand extends Command {
    private AlgaeIntake s_AlgaeIntake;
    private BooleanSupplier intakeSup;
    private BooleanSupplier outtakeSup;

    public AlgaeCommand(AlgaeIntake s_AlgaeIntake, BooleanSupplier intakeSup, BooleanSupplier outtakeSup) {
        this.s_AlgaeIntake = s_AlgaeIntake;
        this.intakeSup = intakeSup;
        this.outtakeSup = outtakeSup;
        addRequirements(s_AlgaeIntake);
    }

    @Override
    public void execute() {
        if (intakeSup.getAsBoolean()) {
            s_AlgaeIntake.setSpeed(-0.7);
        } else if (outtakeSup.getAsBoolean()) {
            s_AlgaeIntake.setSpeed(1);
        } else {
            s_AlgaeIntake.setSpeed(-0.1);
        }
    }
}
