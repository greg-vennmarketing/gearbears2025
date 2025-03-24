package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Client;
import frc.robot.subsystems.SwerveSubsystem;

public class AllignRight extends Command {
    Client cli;
    int id;
    double[] position;
    SwerveSubsystem drive;

    int x = 0;
    int y = 1;
    int theta = 2;

    int dir;
    int lastDir;

    double speed = 0.4;

    public AllignRight(int i, double[] pos, Client cli, SwerveSubsystem driveSys) {
        this.cli = cli;
        this.id = i;
        this.position = pos;
        this.drive = driveSys;
    }

    public void initialize() {
        id = cli.id;
        position = cli.position;
        lastDir = (position[y] < 0.2) ? -1 : 1;
    }
    public void execute() {
        id = cli.id;
        position = cli.position;

        dir = (position[y] < 0.2) ? -1 : 1;
        if (dir != lastDir) {
            speed /= 2;
        }
        drive.swerveDrive.drive(new Translation2d(0, speed * dir), 0, false, false);
        lastDir = dir;
    }
    public void end() {
        speed = 0.4;
        drive.swerveDrive.drive(new Translation2d(0, 0), 0, false,false);
        System.out.println("it worked #Vision");
    }
    public boolean isFinished() {
        return (speed < 0.1) || id == 0;
    }
}
