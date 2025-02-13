package frc.robot.commands;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.States;
import frc.robot.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorCommand extends Command {
    private Elevator s_Elevator;
    private PIDController elevatorController;

    public ElevatorCommand(Elevator s_Elevator) {
        this.s_Elevator = s_Elevator;
        addRequirements(s_Elevator);

        elevatorController = new PIDController(Constants.Elevator.elevatorKP, Constants.Elevator.elevatorKI, Constants.Elevator.elevatorKD);
    }
    
    @Override
    public void execute() {
        double motorSpeed = 0.0;

        switch(States.mElevatorState){
            case minimum:
                motorSpeed = elevatorController.calculate(s_Elevator.getPos(), Constants.Elevator.minimumPOS);
                break;
            case intake:
                motorSpeed = elevatorController.calculate(s_Elevator.getPos(), Constants.Elevator.intakePOS);
                break;
            case l1:
                motorSpeed = elevatorController.calculate(s_Elevator.getPos(), Constants.Elevator.l1POS);
                break;
            case l2:
                motorSpeed = elevatorController.calculate(s_Elevator.getPos(), Constants.Elevator.l2POS);
                break;
            case l3:
                motorSpeed = elevatorController.calculate(s_Elevator.getPos(), Constants.Elevator.l3POS);
                break;
            case l4:
                motorSpeed = elevatorController.calculate(s_Elevator.getPos(), Constants.Elevator.l4POS);
                break;
        }
        
        s_Elevator.setSpeed(motorSpeed);
    }
}
