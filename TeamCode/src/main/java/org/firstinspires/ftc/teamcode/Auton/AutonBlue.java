package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auton Blue", group="Auton")
public class AutonBlue extends AutonBase {

    @Override
    protected void runAuton() {
        // Enable detailed telemetry for this run
        detailMode = true;

        // Reset heading before starting
        resetHeading();

        // Example movement before shooting
        driveForwardInches(72, 0.5);   // drive forward in inches
        turnToHeading(45);             // turn to (input) degrees
        driveForwardInches(52, 0.5);   // drive forward in inches

        // Spin up shooter (70% power via helper)
        spinShooterForward();
        sleep(500);

        // Log velocity before feeding
        logShooterVelocity();

        // Feed 9 artifacts (loop count matches comment)
        for (int i = 0; i < 9; i++) {
            feedForwardStep();
            sleep(500);
            logShooterVelocity();
        }

        // Stop shooter and feeder at the end
        stopShooter();
        stopFeeder();

        // Optional: back up after shooting
        driveForwardInches(-12, 0.5);
    }
}