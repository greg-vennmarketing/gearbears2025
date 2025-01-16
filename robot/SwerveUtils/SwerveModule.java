package frc.robot.SwerveUtils;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
//import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.CANSparkBase.IdleMode;
//import com.revrobotics.CANSparkLowLevel.MotorType;


//new imports
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.AbsoluteEncoder;




import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
//import edu.wpi.first.units.measure.Unit;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import frc.robot.Constants;
import edu.wpi.first.units.AngleUnit;



public class SwerveModule {
    private final int moduleId;

    //changed CANSparkMax to SparkMax
    private final SparkMax driveMotor;
    private final SparkMax turnMotor;

   



    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;
    private final CANcoder absEncoder;
    private final SwerveModuleConstants swerveModuleConstants;
    private final PIDController turnPIDController;
    private final StatusSignal<Angle> absolutePositionSignal;
    private Rotation2d lastAngle = new Rotation2d(0);
    public SwerveModule(SwerveModuleConstants swerveModuleConstants){

        

        this.moduleId = swerveModuleConstants.moduleId;
        this.swerveModuleConstants = swerveModuleConstants;
    
        this.driveMotor = new SparkMax(swerveModuleConstants.driveMotorId, MotorType.kBrushless);
        this.turnMotor = new SparkMax(swerveModuleConstants.turnMotorId, MotorType.kBrushless);
    
        this.driveEncoder = driveMotor.getEncoder();
        this.turnEncoder = turnMotor.getEncoder();
       





        this.absEncoder = new CANcoder(swerveModuleConstants.absoluteEncoderId);
        this.absolutePositionSignal = absEncoder.getAbsolutePosition();
    
        this.turnPIDController = new PIDController(0.0, 0.0, 0.0);
        this.turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
    
        resetEncoders();
}

//        /*driveMotor = new /*deleted CAN*/ SparkMax(swerveModuleConstants.driveMotorId, MotorType.kBrushless);
//        //driveMotor.restoreFactoryDefaults();
//        driveMotor.getConfigurator().setIdleMode(IdleMode.kBrake);
//        driveMotor.setInverted(swerveModuleConstants.driveMotorInverted);
//        //driveMotor.setSmartCurrentLimit(20);
//
//        turnMotor = new /*deleted CAN*/ SparkMax(swerveModuleConstants.turnMotorId, MotorType.kBrushless);
//        //turnMotor.restoreFactoryDefaults();
//        turnMotor.setIdleMode(IdleMode.kBrake);
//        turnMotor.setInverted(true);
//        //turnMotor.setSmartCurrentLimit(20);
//
//        driveEncoder = driveMotor.getEncoder();
//        turnEncoder = turnMotor.getEncoder();
//
//        driveEncoder.setPositionConversionFactor(Constants.PhysicalConstants.kDriveRotationsToMeters);
//        driveEncoder.setVelocityConversionFactor(Constants.PhysicalConstants.kDriveRPMToMetersPerSec);
//        turnEncoder.setPositionConversionFactor(Constants.PhysicalConstants.kTurnRotationsToAngelDeg);
//        turnEncoder.setVelocityConversionFactor(Constants.PhysicalConstants.kTurnRPMToDegPerSec);
//
//    absEncoder = new CANcoder(swerveModuleConstants.absoluteEncoderId); //new AnalogPotentiometer(swerveModuleConstants.absoluteEncoderId, 360, swerveModuleConstants.absoluteEncoderOffset);
//     absolutePositionSignal = absEncoder.getAbsolutePosition();
//       //absEncoder.getConfigurator().apply(new MagnetSensorConfigs().)
//        turnPIDController = new PIDController(.5, 0, 0);
//
//        //This is so that the PID knows that the turning wheels should go in a loop
//        turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
//
//        moduleId = swerveModuleConstants.moduleId;
//        this.swerveModuleConstants = swerveModuleConstants;
//
//        resetEncoders();

    



    public double getAbsAngleDeg(){
        
        absolutePositionSignal.refresh();

        return Units.radiansToDegrees(absolutePositionSignal.getValueAsDouble());



        //return Units.radiansToDegrees(absolutePositionSignal.getValue().copy().in(AngleUnit.RADIANS()));
        //return Units.radiansToDegrees(absolutePositionSignal.getValue().copy());

        

        //return absolutePositionSignal.getValue().getValue();
        //return absolutePositionSignal.refresh().getValue();
    }


    public Rotation2d getAngleRotation2d(){
        return Rotation2d.fromDegrees(getAbsAngleDeg());
    }

    public double getDriveSpeedMetersPerSec(){
        return driveEncoder.getVelocity();
    }

    public void resetEncoders(){
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsAngleDeg());
    }

    public SwerveModuleState getSwerveModuleState(){
        return new SwerveModuleState(getDriveSpeedMetersPerSec(), getAngleRotation2d());
    }

    public void stopMotors(){
        driveMotor.set(0);
        turnMotor.set(0);
    }



    //If you want, you can try setting using the Neo relative encoders

    @SuppressWarnings("deprecation")
    public void setSwerveModuleState(SwerveModuleState desiredState){
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.01){
            desiredState.angle = lastAngle;
        }
        desiredState = SwerveModuleState.optimize(desiredState, getAngleRotation2d());
        driveMotor.set(desiredState.speedMetersPerSecond/Constants.PhysicalConstants.kPhysicalMaxSpeedMetersPerSec);
        turnMotor.set(turnPIDController.calculate(Units.degreesToRadians(getAbsAngleDeg()), Units.degreesToRadians(desiredState.angle.getDegrees())));
        lastAngle = desiredState.angle;
    }

}
