package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auton Red", group="Auton")
public class AutonRed extends AutonBase {

    @Override
    protected void runAuton() {
        // Enable detailed telemetry for this run
        detailMode = true;

        // Reset heading before starting
        resetHeading();

        // Movement before shooting
        driveForwardInches(72, 0.5);   // drive forward in inches
        turnToHeading(45);            // turn to heading
        driveForwardInches(52, 0.5);   // drive forward in inches

        // Spin up shooter (70% power via helper)
        spinShooterForward();
        sleep(500);

        // Log velocity before feeding
        logShooterVelocity();

        // Feed 9 artifacts (loop count matches code, comment said 6)
        for (int i = 0; i < 9; i++) {
            feedForwardStep();
            sleep(750);
            logShooterVelocity();
        }

        // Stop shooter and feeder at the end
        stopShooter();
        stopFeeder();

        // Optional: back up after shooting
        driveForwardInches(-12, 0.5);
    }
}