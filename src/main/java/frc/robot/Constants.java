package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.swerveUtil.COTSNeoSwerveConstants;
import frc.lib.util.swerveUtil.SwerveModuleConstants;
import frc.lib.util.swerveUtil.COTSNeoSwerveConstants.driveGearRatios;
public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final int climberCanID = 62;

    public static final int kPCMCANId = 1;
   
    public static final class CoralIntake {
        public static final int coralCanID = 53; 
        public static final int kDumpForward = 0;
        public static final int kDumpReverse = 1;
    }

    public static final class AlgaeIntake {
        public static final int algaeIntakeCanID = 50; 
        public static final int kDumpForward = 2;
        public static final int kDumpReverse = 3;
    }

    public static final class Elevator {
        public static final int leftElevatorID = 21;
        public static final int rightElevatorID = 22;
        public static final int elevatorEncoderID = 3;

        public static final double intakePOS = 2.5;
        public static final double l1POS = 0;
        public static final double l2POS = 10.5;
        public static final double aLPOS = 41;
        public static final double l3POS = 33;
        public static final double aHPOS = 62.5;
        public static final double l4POS = 70;

        public static final double elevatorKP = 0.035;
        public static final double elevatorKI = 0.005;
        public static final double elevatorKD = 0.0;
    }

    public static final class Swerve {
        public static final int pigeonID = 0;
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        public static final COTSNeoSwerveConstants chosenModule =  COTSNeoSwerveConstants.SDSMK4i(driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(24.5);
        public static final double wheelBase = Units.inchesToMeters(24.25);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        public static final double driveRevToMeters =  wheelCircumference / (chosenModule.driveGearRatio);
        public static final double driveRpmToMetersPerSecond = driveRevToMeters/60 ;
        // the number of degrees that a single rotation of the turn motor turns the wheel.
        public static final double DegreesPerTurnRotation = 360/chosenModule.angleGearRatio;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        public static final Translation2d[] moduleTranslations =
            new Translation2d[] {
            new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
            new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
            new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
            new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
        };

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

         /* Motor Inverts */
         public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
         public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 40;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive Motor to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.2;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 2;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

         /* Heading PID Values */
        public static final double HeadingKP = .5;
        public static final double HeadingKI = 0;
        public static final double HeadingKD = 0;
        public static final double HeadingTolerence = 0;

        //Motor power gain
        public static final double drivePower = 1;
        public static final double anglePower = .9;


        /* Drive Motor Characterization Values from SysID */
        public static final double driveKS = (0.32); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51);
        public static final double driveKA = (0.27);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final  IdleMode angleNuetralMode = IdleMode.kCoast;
        public static final  IdleMode driveNuetralMode = IdleMode.kBrake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 52;
            public static final int angleMotorID = 59;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(.62);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 55;
            public static final int angleMotorID = 58;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(33.75);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 56;
            public static final int angleMotorID = 61;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(88.46);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 54;
            public static final int angleMotorID = 60;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(58.32);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        //TODO: Must be tuned to specific robot
        public static final PIDConstants translationPID = new PIDConstants(5, 0, 0);/// All values weree 0. Chnaged to see if the robot moves in automous
        public static final PIDConstants rotationPID = new PIDConstants(5, 0, 0);

        //TODO: Must be tuned to specific robot
        public static final double ROBOT_MASS_KG = 56.699;
        public static final double ROBOT_MOI = 4.5;
        public static final double WHEEL_COF = 1.2;

        public static final ModuleConfig moduleConfig = new ModuleConfig(
                (Constants.Swerve.chosenModule.wheelDiameter / 2),
                (Constants.Swerve.maxSpeed),
                Constants.AutoConstants.WHEEL_COF,
                DCMotor.getNEO(1).withReduction(Constants.Swerve.chosenModule.driveGearRatio),
                Constants.Swerve.driveCurrentThreshold,
              1);
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class PoseEstimator {
        public static final Matrix<N3,N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        public static final Matrix<N3,N1> visionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    }

}
