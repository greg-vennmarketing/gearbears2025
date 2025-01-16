package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
//import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
//import com.pathplanner.lib.util.PIDConstants;



import com.pathplanner.lib.auto.CommandUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveUtils.SwerveModule;

//gary
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
 
public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule[] swerveModules;

    private static SwerveSubsystem INSTANCE;

    public static SwerveSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new SwerveSubsystem();
        }
        return INSTANCE;
    }

    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    // Constructor
    private SwerveSubsystem() {
        SwerveModule frontLeft = new SwerveModule(Constants.SwerveModuleConstantsInstances.kFrontLeftModule);
        SwerveModule frontRight = new SwerveModule(Constants.SwerveModuleConstantsInstances.kFrontRightModule);
        SwerveModule backLeft = new SwerveModule(Constants.SwerveModuleConstantsInstances.kBackLeftModule);
        SwerveModule backRight = new SwerveModule(Constants.SwerveModuleConstantsInstances.kBackRightModule);

        swerveModules = new SwerveModule[] { frontLeft, frontRight, backLeft, backRight };
    }

    public void zeroHeading() {
        gyro.reset();
        System.out.println("Gyro reset");
    }

    public double getHeading() {
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }

    public Rotation2d getHeadingRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void stopSwerveModuleMotors() {
        for (SwerveModule module : swerveModules) {
            module.stopMotors();
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.PhysicalConstants.kPhysicalMaxSpeedMetersPerSec);
        swerveModules[0].setSwerveModuleState(desiredStates[1]); // Front Left
        swerveModules[1].setSwerveModuleState(desiredStates[0]); // Front Right
        swerveModules[2].setSwerveModuleState(desiredStates[3]); // Back Left
        swerveModules[3].setSwerveModuleState(desiredStates[2]); // Back Right
    }

    public void resetAllEncoders() {
        for (SwerveModule module : swerveModules) {
            module.resetEncoders();
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro Heading", getHeading());
        SmartDashboard.putString("Robot Pose", getHeadingRotation2d().toString());
        for (int i = 0; i < swerveModules.length; i++) {
            SmartDashboard.putString("Module " + i + " State", swerveModules[i].getSwerveModuleState().toString());
        }
    }

    public void setupShuffleboard() {
        ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");

        swerveTab.addNumber("Gyro Heading", this::getHeading);
        swerveTab.addString("Robot Pose", () -> getHeadingRotation2d().toString());
        for (int i = 0; i < swerveModules.length; i++) {
            int moduleIndex = i; // Required for lambda expressions
            swerveTab.addString("Module " + moduleIndex + " State", () -> swerveModules[moduleIndex].getSwerveModuleState().toString());
        }

        swerveTab.add("Zero Gyro", new InstantCommand(this::zeroHeading))
                .withWidget("Button");
    }
}

/*KJBKUGJHCVLGVpublic class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule[] swerveModules;



    private static SwerveSubsystem INSTANCE;

    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    /**
     * The Singleton instance of this SwerveSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */


    //private static SwerveSubsystem INSTANCE;

    /**
     * Returns the Singleton instance of this SwerveSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code SwerveSubsystem.getInstance();}
     */
    
    
/* KLJHBLJHVLJGV    SwerveModule frontLeft = new SwerveModule(Constants.SwerveModuleConstantsInstances.kFrontLeftModule);
    SwerveModule frontRight = new SwerveModule(Constants.SwerveModuleConstantsInstances.kFrontRightModule);
    SwerveModule backLeft = new SwerveModule(Constants.SwerveModuleConstantsInstances.kBackLeftModule);
    SwerveModule backRight = new SwerveModule(Constants.SwerveModuleConstantsInstances.kBackRightModule);

    swerveModules = new SwerveModule[] { frontLeft, frontRight, backLeft, backRight };
    
    
   @SuppressWarnings("WeakerAccess")
    public static SwerveSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new SwerveSubsystem();
        }
        return INSTANCE;
    }
    
    
 
       
        
      


    /**
     * Creates a new instance of this SwerveSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */

    //private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    //private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

//    public void zeroHeading(){
//        gyro.reset();
//        System.out.println("Gyro reset");
//    }

//    public double getHeading(){
//        return Math.IEEEremainder(-gyro.getAngle(), 360);
//    }

//    public Rotation2d getHeadingRotation2d(){
//        return Rotation2d.fromDegrees(getHeading());
//    }

//    public void stopSwerveModuleMotors(){
//        frontLeft.stopMotors();
//        frontRight.stopMotors();
//        backLeft.stopMotors();
//        backRight.stopMotors();
//    }
//
//    public void setModuleStates(SwerveModuleState[] desiredStates){
//        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.PhysicalConstants.kPhysicalMaxSpeedMetersPerSec);
//        frontLeft.setSwerveModuleState(desiredStates[1]);
//        frontRight.setSwerveModuleState(desiredStates[0]);
//       backLeft.setSwerveModuleState(desiredStates[3]);
//        backRight.setSwerveModuleState(desiredStates[2]);
//    }
//
//    public void resetAllEncoders() {
//        for (SwerveModule module : swerveModules) {
//            module.resetEncoders();
//        }
//    }


//    @Override
//    public void periodic() {
//        // Update the Shuffleboard with useful information
//        SmartDashboard.putNumber("Gyro Heading", getHeading());
//        SmartDashboard.putString("Robot Pose", getHeadingRotation2d().toString());
//
//        SmartDashboard.putString("Front Left State", frontLeft.getSwerveModuleState().toString());
//        SmartDashboard.putString("Front Right State", frontRight.getSwerveModuleState().toString());
//       SmartDashboard.putString("Back Left State", backLeft.getSwerveModuleState().toString());
//        SmartDashboard.putString("Back Right State", backRight.getSwerveModuleState().toString());
//    }
//
//
//
//        public void setupShuffleboard() {
//            ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
//
//            swerveTab.addNumber("Gyro Heading", this::getHeading);
//            swerveTab.addString("Robot Pose", () -> getHeadingRotation2d().toString());
//            swerveTab.addString("Front Left State", () -> frontLeft.getSwerveModuleState().toString());
//            swerveTab.addString("Front Right State", () -> frontRight.getSwerveModuleState().toString());
//            swerveTab.addString("Back Left State", () -> backLeft.getSwerveModuleState().toString());
//            swerveTab.addString("Back Right State", () -> backRight.getSwerveModuleState().toString());
//
//            // Example button
//            swerveTab.add("Zero Gyro", new InstantCommand(this::zeroHeading))
//                    .withWidget("Button");
//    }

//}

