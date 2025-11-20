package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutonBlue", group = "Autonomous")
public class AutonBlue extends AutonBase {

    @Override
    protected void runAuton() {
        // Use drive subsystem methods
        drive.driveForwardInches(40, 1.0);
        drive.gyroTurn(45, 0.4);
        drive.driveForwardInches(30, 0.5);

        // Shooter sequence
        shooter.shootForward();
        sleep(500);

        // Feed 6 game pieces
        for (int i = 6; i > 0; i--) {
            feeder.feedForward();
            sleep(500);
        }

        // Stop subsystems
        feeder.stop();
        sleep(1000);
        shooter.stop();
    }
}