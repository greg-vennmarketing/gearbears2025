package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class GoToPoint extends Command {
    Translation2d poseFromRobot;

    SwerveSubsystem drive;
    Pose2d destination;
    Pose2d currentPosition;
    
    PIDController xPID;
    PIDController yPID;
    PIDController thetaPID;

    double maxSpeed;
    double xOutput;
    double yOutput;

    Pose2d initialPosition;

    // still needs work
    public GoToPoint(SwerveSubsystem driveSys, Translation2d poseFromRobot, double maxOutput) {
        drive = driveSys;
        maxSpeed = maxOutput;
        this.poseFromRobot = poseFromRobot;

        addRequirements(drive);

        // get the PID controllers for each direction
        // p 0.8 i 0.05 d 0.085
        xPID = new PIDController(0.9, 0.05, 0.03);
        yPID = new PIDController(0.9, 0.05, 0.03);
        thetaPID = new PIDController(0, 0, 0);
    }

    public void initialize() {
        xPID.reset();
        yPID.reset();
        thetaPID.reset();

        initialPosition = drive.swerveDrive.getPose();
        Translation2d gettingPos = new Translation2d(initialPosition.getX() + poseFromRobot.getX(), initialPosition.getY() + poseFromRobot.getY());
        destination = new Pose2d(gettingPos.getMeasureX(), gettingPos.getMeasureY(), new Rotation2d(0));

        xPID.setSetpoint(destination.getX());
        yPID.setSetpoint(destination.getY());
        thetaPID.setSetpoint(destination.getRotation().getRadians());
    }

    public void execute() {
        currentPosition = drive.swerveDrive.getPose();

        xOutput = xPID.calculate(currentPosition.getX());
        yOutput = yPID.calculate(currentPosition.getY());

        // System.out.println("y error: " + yPID.getError());
        // System.out.println("y output: " + yOutput);
        
        double hyp = Math.hypot(xOutput, yOutput);
        if (hyp > maxSpeed) {
            xOutput = xOutput / hyp * maxSpeed;
            yOutput = yOutput / hyp * maxSpeed;
        }
        
        if (hyp < 0) {
            xOutput = 0;
            yOutput = 0;
        }


        drive.swerveDrive.drive(new Translation2d(xOutput, yOutput), 0, false, false);
    }

    public void end(boolean interupt) {
        System.out.println(xPID.getError() + " " + yPID.getError());
        drive.swerveDrive.drive(new Translation2d(0, 0), 0, false, true);
    }

    public boolean isFinished() {
        return xOutput == 0 && yOutput == 0;
    }
}
