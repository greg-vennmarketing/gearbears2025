package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class TurnTest extends Command{
    SwerveSubsystem drive;
    int counter;
    public TurnTest(SwerveSubsystem drive) {
        this.drive = drive;
    }

    public void initialize() {
        counter = 0;
    }

    public void execute() {
        drive.swerveDrive.drive(new Translation2d(0,0), (Math.PI / 2), false, false);
        counter++;
    }

    public void end() {
        counter = 0;
    }

    public boolean isFinished() {
        return counter >= 60;
    }
}