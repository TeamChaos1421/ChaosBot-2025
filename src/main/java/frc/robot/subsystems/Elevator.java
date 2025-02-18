package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private TalonFX mLeftElevator;
    private TalonFX mRightElevator;
    private CANcoder elevatorEncoder;

    public Elevator() {
        mLeftElevator = new TalonFX(Constants.Elevator.leftElevatorID);
        mRightElevator = new TalonFX(Constants.Elevator.rightElevatorID);

        elevatorEncoder = new CANcoder(Constants.Elevator.elevatorEncoderID);
    }

    public Elevator setSpeed(double speed) {
        mLeftElevator.set(speed);
        mRightElevator.set(-speed);
        return this;
    }

    public double getPos() {
        return -mRightElevator.getPosition().getValueAsDouble();
        // return elevatorEncoder.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Height", -mRightElevator.getPosition().getValueAsDouble());
    }
}