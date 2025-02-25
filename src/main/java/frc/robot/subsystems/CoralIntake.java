package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralIntake extends SubsystemBase {
    private SparkMax mCoralMotor;
    private Compressor mCompressor;
    private DoubleSolenoid mSolenoid;

    public CoralIntake() {
        mCoralMotor = new SparkMax(Constants.CoralIntake.coralCanID, SparkMax.MotorType.kBrushed);
        mCompressor = new Compressor(
            Constants.CoralIntake.kPCMCANId, 
            PneumaticsModuleType.CTREPCM
        );
        mSolenoid = new DoubleSolenoid(
            Constants.CoralIntake.kPCMCANId,
            PneumaticsModuleType.CTREPCM,
            Constants.CoralIntake.kDumpForward, 
            Constants.CoralIntake.kDumpReverse
        );

        mCompressor.enableDigital();
        mSolenoid.set(Value.kForward);
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
        mCoralMotor.set(speed);
    }
}
