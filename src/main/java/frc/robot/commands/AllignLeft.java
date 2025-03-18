package frc.robot.commands;


import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Client;
import frc.robot.subsystems.SwerveSubsystem;

public class AllignLeft extends Command {
    public double[] position;
    public int id;
    public Client cli;
    public SwerveSubsystem drive;

    public AllignLeft(int i, double[] pos, Client c, SwerveSubsystem driveSys) {
        this.id = i;
        this.position = pos;
        this.cli = c;
        this.drive = driveSys;
    }

    public void initialize() {
        id = cli.id;
        position = cli.position;
    }

    public void execute() {
        System.out.println("hello");
        try {
            cli.read();
        } catch (IOException e) {
            System.out.println("client vision failed");
        }
        id = cli.id;
        position = cli.position;

        /*SwerveInputStream in = new SwerveInputStream(drive.getSwerveDrive(), () -> 0.0, () -> -0.1,() -> 0.0);
        drive.driveFieldOriented(in);
    */
        drive.getSwerveDrive().addVisionMeasurement(
            new Pose2d(
                1, 1, Rotation2d.fromDegrees(65)
            ), 
            Timer.getFPGATimestamp()
        );
    }


    public void end() {
        System.out.println("it worked: #vision");
    }

    public boolean isFinished() {
        return position[1] <= -0.1778 || id == 0;
    }
}
