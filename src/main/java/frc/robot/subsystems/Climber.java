package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private SparkMax mClimberMotor;

    public Climber() {
        mClimberMotor = new SparkMax(Constants.climberCanID, SparkMax.MotorType.kBrushless);
    }

    public Climber setSpeed(double speed) {
        mClimberMotor.set(speed);
        return this;
    }
    
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Climber Height", mClimberMotor.getEncoder().getPosition());
    }
}
