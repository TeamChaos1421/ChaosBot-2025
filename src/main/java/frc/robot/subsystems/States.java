package frc.robot.subsystems;

public class States {
    public static enum ElevatorStates {
        intake, l1, l2, l3, l4
    }

    public static Boolean mElevatorToggle = true;
    public static ElevatorStates mElevatorState = ElevatorStates.intake;

    public static enum DriveStates {
        standard, leftHold, rightHold, forwardHold, backwardHold, DynamicLock
    }

    public static enum AlignedStates {
        aligned, unAligned, normal
    }

    public static DriveStates driveState = DriveStates.standard;
    public static AlignedStates alignedState = AlignedStates.normal;
}
