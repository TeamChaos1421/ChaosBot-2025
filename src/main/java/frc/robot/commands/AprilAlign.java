package frc.robot.commands;

import frc.robot.Constants;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.NetworkTableInstance;


public class AprilAlign extends Command {    
    private Swerve s_Swerve;
    private PIDController translationController;
    private PIDController strafeController;
    private PIDController rotationController;
    public String llName; 
    

    public AprilAlign(Swerve s_Swerve, String llName) {
        this.s_Swerve = s_Swerve;
        this.llName = "limelight-" + llName;
        addRequirements(s_Swerve);

        translationController = new PIDController(Constants.Swerve.HeadingKP, Constants.Swerve.HeadingKI, Constants.Swerve.HeadingKD );
        strafeController = new PIDController(Constants.Swerve.HeadingKP, Constants.Swerve.HeadingKI, Constants.Swerve.HeadingKD );
        rotationController = new PIDController(Constants.Swerve.HeadingKP, Constants.Swerve.HeadingKI, Constants.Swerve.HeadingKD );
        
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Constants.Swerve.HeadingTolerence);
    }

    @Override
    public void execute() {
        double[] t2d = NetworkTableInstance.getDefault().getTable(llName).getEntry("t2d").getDoubleArray(new double[17]);

        /* Get Values, Deadband, Dampen */
        double translationVal = translationController.calculate(t2d[4], 0);
        double strafeVal = strafeController.calculate(t2d[5], 0);
        double rotationVal = rotationController.calculate(t2d[16], 0);
        
        rotationVal = rotationVal * Constants.Swerve.maxAngularVelocity;

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed * 0.5), 
            rotationVal,
            false, 
            true
        );
    }
}