package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntake extends SubsystemBase {
    private SparkMax mAlgaeMotor;
    private DoubleSolenoid mSolenoid;

    public AlgaeIntake() {
        mAlgaeMotor = new SparkMax(Constants.AlgaeIntake.algaeIntakeCanID, SparkMax.MotorType.kBrushed);
        mSolenoid = new DoubleSolenoid(
            Constants.kPCMCANId,
            PneumaticsModuleType.CTREPCM,
            Constants.AlgaeIntake.kDumpForward, 
            Constants.AlgaeIntake.kDumpReverse
        );

        mSolenoid.set(Value.kReverse);
    }

    public void setAngle(Value newValue) {
        mSolenoid.set(newValue);
    }

    public void toggleAngle() {
        if(mSolenoid.get() == Value.kReverse) {
            mSolenoid.set(Value.kForward);
        } else {
            mSolenoid.set(Value.kReverse);
        }
    }
  
    public void setSpeed(double speed) {
        mAlgaeMotor.set(speed);
    }
}
