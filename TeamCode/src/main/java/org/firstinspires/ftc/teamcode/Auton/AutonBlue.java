package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="**********  Auton Blue  **********", group="Auton")
public class AutonBlue extends AutonBase {

    @Override
    protected void runAuton() {
        detailMode = true;
        resetHeading();

        // Step 1: Drive forward 72 inches, hold heading 0°
        driveStraightWithHeading(72, 0.5, 0);
        sleep(100); // settle pause

        // Step 2: Turn to -45° (short path CCW)
        turnToHeading(-45);
        xBrake(0.18, 120);   // clamp momentum to prevent overshoot
        sleep(150);          // longer settle pause
        imu.logTelemetry(telemetry); // log IMU heading after turn

        // Step 3: Drive forward 52 inches, hold heading -45°
        driveStraightWithHeading(52, 0.5, -45);
        sleep(100); // settle pause

        // Step 4: Back up 12 inches, hold heading -45°
        driveStraightWithHeading(-12, -0.5, -45);
        sleep(100); // settle pause

        // Step 5: Strafe left 18 inches, hold heading -45°
        driveStrafeWithHeading(-18, 0.5, -45); // negative = left, positive = right
        sleep(100); // settle pause

        // Step 6: Shooter sequence
        spinShooterForward();
        sleep(500); // spin-up delay
        logShooterVelocity();

        // Feed 9 artifacts
        for (int i = 0; i < 9; i++) {
            feedForwardStep();
            sleep(750); // match Red’s feed timing
            logShooterVelocity();
        }

        // Stop shooter and feeder
        stopShooter();
        stopFeeder();
    }
}