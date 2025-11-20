package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutonRed", group = "Autonomous")
public class AutonRed extends AutonBase {

    @Override
    protected void runAuton() {
        // Use drive subsystem methods
        drive.driveForwardInches(50, 1.0);
        drive.driveForwardInches(13, 0.5);
        drive.gyroTurn(-45, 0.4);
        drive.driveForwardInches(24, 0.5);

        // Shooter sequence
        shooter.shootForward();
        sleep(500);

        // Feed 6 game pieces
        for (int i = 0; i < 6; i++) {
            feeder.feedForward();
            sleep(500);
        }

        // Stop subsystems
        feeder.stop();
        sleep(1000);
        shooter.stop();
    }
}