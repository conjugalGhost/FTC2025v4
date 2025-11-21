package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auton Red", group="Auton")
public class AutonRed extends AutonBase {

    @Override
    protected void runAuton() {
        // Spin up shooter
        shooter.shootForward();
        sleep(500);

        // Log velocity before feeding
        logShooterVelocity();

        // Feed 6 artifacts
        for (int i = 0; i < 6; i++) {
            if (shooter.isReady()) {
                feeder.advanceOneStep();
            }
            sleep(500);

            // Log velocity after each shot
            logShooterVelocity();
        }

        // Stop shooter at the end
        shooter.stop();
    }
}