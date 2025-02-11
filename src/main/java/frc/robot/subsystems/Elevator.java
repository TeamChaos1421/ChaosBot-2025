package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private TalonFX mLeftElevator;
    private TalonFX mRightElevator;

    private ElevatorStates mElevatorState;

    public Elevator() {
        mLeftElevator = new TalonFX(Constants.Elevator.leftElevatorID);
        mRightElevator = new TalonFX(Constants.Elevator.rightElevatorID);

        mElevatorState = ElevatorStates.minimun;
    }

    public static enum ElevatorStates {
        minimun, intake, l1, l2, l3, l4
    }

    public Elevator setSpeed(double speed) {
        mLeftElevator.set(speed);
        mRightElevator.set(-speed);
        return this;
    }

    public Elevator increaseState() {
        if(mElevatorState != ElevatorStates.values()[ElevatorStates.values().length - 1]) {
            mElevatorState = ElevatorStates.values()[mElevatorState.ordinal() + 1];
        }
        return this;
    }

    public Elevator decreaseState() {
        if(mElevatorState != ElevatorStates.values()[0]) {
            mElevatorState = ElevatorStates.values()[mElevatorState.ordinal() - 1];
        }
        return this;
    }
}