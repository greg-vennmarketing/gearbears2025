package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Client;
import frc.robot.subsystems.SwerveSubsystem;

public class AllignLeft extends Command {
    public double[] position;
    public int id;
    public Client cli;
    public SwerveSubsystem drive;
    int x = 0;
    int y = 1;
    int theta = 2;
    double speed = 0.4;
    int lastDir = 0;

    public AllignLeft(int i, double[] pos, Client c, SwerveSubsystem driveSys) {
        this.id = i;
        this.position = pos;
        this.cli = c;
        this.drive = driveSys;
    }

    public void initialize() {
        id = cli.id;
        position = cli.position;
        lastDir = (position[y] < -0.11) ? -1 : 1;
    }

    public void execute() {
        id = cli.id;
        position = cli.position;

        int dirY = (position[y] < -0.11) ? -1 : 1;

        if (dirY != lastDir) {
            speed = speed / 2;
        }
        lastDir = dirY;

        drive.swerveDrive.drive(new Translation2d(0, speed * dirY), 0, false, false);
    }


    public void end(boolean interupt) {
        if (!interupt) {
            speed = 0.4;
            System.out.println("it worked: #vision");
            drive.swerveDrive.drive(new Translation2d(0.0, 0.0), 0.0, false, false);
        }
    }

    public boolean isFinished() {
        // 0.11 dif = 0.0254
        return (speed < 0.1) || (id == 0); // if in specific range or it doesn't pick up an apriltag
    }
}
