package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax primaryMotor;
    private final SparkMax followerMotor;
    private final RelativeEncoder encoder;
    private final PIDController pidController;
    private final TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goalState;
    private TrapezoidProfile.State currentState;
    private final TrapezoidProfile profile;

    private boolean isHomed = true;
    private double setpoint = 0.0;
    SparkMaxConfig resetConfig = new SparkMaxConfig();
    double currentPos;

    public enum ElevatorPosition { // Values must be set in ElevatorConstants
        DOWN(ElevatorConstants.downPos),
        POSITION_1(ElevatorConstants.L1),
        POSITION_2(ElevatorConstants.L2),
        POSITION_3(ElevatorConstants.L3),
        POSITION_4(ElevatorConstants.L4);

        public final double positionInches;
        
        ElevatorPosition(double positionInches) {
            this.positionInches = positionInches;
        }
    }

    public ElevatorSubsystem() {
        primaryMotor = new SparkMax(ElevatorConstants.leftElevatorID, MotorType.kBrushless);
        followerMotor = new SparkMax(ElevatorConstants.rightElevatorID, MotorType.kBrushless);
        
        // Configure follower motor to follow the primary motor
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        primaryMotor.setInverted(true);
        followerConfig.follow(primaryMotor, false); // 'true' to invert if needed
        
        followerMotor.configure(followerConfig, null, null);

        encoder = primaryMotor.getEncoder();

        resetConfig.idleMode(IdleMode.kBrake);
        resetConfig.smartCurrentLimit(40);
        resetConfig.voltageCompensation(12.0);

        constraints = new TrapezoidProfile.Constraints(
            ElevatorConstants.kMaxVelocity,
            ElevatorConstants.kMaxAcceleration
        );
        
        pidController = new PIDController(
            ElevatorConstants.kElevatorKp,
            ElevatorConstants.kElevatorKi,
            ElevatorConstants.kElevatorKd
        );
        
        pidController.setTolerance(0.5); // 0.5 inches position tolerance
        
        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);
        profile = new TrapezoidProfile(constraints);
        
        configureMotors();

        // Auto-home on startup
        //homeElevator();
    }

    private void configureMotors() {
        primaryMotor.configure(resetConfig, null, null);
        followerMotor.configure(resetConfig, null, null);
    }

    @Override
    public void periodic() {
        currentPos = getHeightInches();
        currentState = profile.calculate(0.020, currentState, goalState); // 20ms loop

        // Stop movement if past max height
        if (getHeightInches() > ElevatorConstants.maxPos) {
            stopMotors();
        }

        // Control only if homed
        if (isHomed) {
            double pidOutput = pidController.calculate(getHeightInches(), currentState.position);
            double ff = calculateFeedForward(currentState);
            
            double outputPower = MathUtil.clamp(
                pidOutput + ff,
                -ElevatorConstants.max_output,
                ElevatorConstants.max_output
            );
            
            primaryMotor.set(outputPower);
        }

        updateTelemetry();
    }

    public void stopMotors() {
        primaryMotor.set(0);
        pidController.reset();
    }

    public boolean isAtHeight(double targetHeightInches) {
        return pidController.atSetpoint() && 
               Math.abs(getHeightInches() - targetHeightInches) < ElevatorConstants.posTolerance;
    }
    
    private double calculateFeedForward(TrapezoidProfile.State state) {
        return ElevatorConstants.kElevatorkS * Math.signum(state.velocity) +
               ElevatorConstants.kElevatorkG +
               ElevatorConstants.kElevatorkV * state.velocity;
    }

    public void setPositionInches(double inches) {
        if (!isHomed && inches > 0) {
            System.out.println("Warning: Elevator not homed! Home first before moving to positions.");
            return;
        }

        setpoint = MathUtil.clamp(
            inches,
            ElevatorConstants.minPos,
            ElevatorConstants.maxPos
        );
        
        goalState = new TrapezoidProfile.State(setpoint, 0);
    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("Elevator Height", getHeightInches());
        SmartDashboard.putNumber("Elevator Target", setpoint);
        SmartDashboard.putBoolean("Elevator Homed", isHomed);
        SmartDashboard.putNumber("Elevator Current", primaryMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Velocity", currentState.velocity);
    }

    public double getHeightInches() {
        return encoder.getPosition() / ElevatorConstants.countsPerInch;
    }

    public void homeElevator() {
        System.out.println("Homing elevator...");
        primaryMotor.set(-0.1); // Slow downward movement
        while (true) {
            double currentDraw = primaryMotor.getOutputCurrent();
            if (currentDraw > ElevatorConstants.stallCurrentThreshold) {
                // Stalled at bottom
                encoder.setPosition(0);
                primaryMotor.set(0);
                isHomed = true;
                System.out.println("Elevator homed.");
                break;
            }
        }
    }

    public boolean isHomed() {
        return isHomed;
    }

    public void setManualPower(double power) {
        pidController.reset();
        currentState = new TrapezoidProfile.State(getHeightInches(), 0);
        goalState = new TrapezoidProfile.State(getHeightInches(), 0);
        
        if (!isHomed && power < 0) {
            power = 0;
        }
        
        if (getHeightInches() >= ElevatorConstants.maxPos && power > 0) {
            power = 0;
        }
        
        primaryMotor.set(MathUtil.clamp(power, -ElevatorConstants.max_output, ElevatorConstants.max_output));
    }


}

