package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="*********  Auton Red  *********", group="Auton")
public class AutonRed extends AutonBase {

    @Override
    protected void runAuton() {
        detailMode = true;
        resetHeading();

        // Step 1: Drive forward 72 inches, hold heading 0°
        driveStraightWithHeading(72, 0.5, 0);
        sleep(100); // NOTE: settle pause

        // Step 2: Turn 45° right
        turnToHeading(45);
        sleep(100); // NOTE: settle pause

        // Step 3: Drive forward 52 inches, hold heading 45°
        driveStraightWithHeading(52, 0.5, 45);
        sleep(100); // NOTE: settle pause

        // Step 4: Back up 12 inches, hold heading 45°
        driveStraightWithHeading(-12, -0.5, 45);
        sleep(100); // NOTE: settle pause

        // Step 5: Strafe left 18 inches
        // NOTE: You’ll need a strafe helper in AutonBase using mecanum math.
        // Example implementation:
        driveStrafeWithHeading(-18, 0.5, 45); // negative = left, positive = right
        sleep(100); // NOTE: settle pause

        // Shooter sequence
        spinShooterForward();
        sleep(500);

        logShooterVelocity();

        for (int i = 0; i < 9; i++) {
            feedForwardStep();
            sleep(750);
            logShooterVelocity();
        }

        stopShooter();
        stopFeeder();
    }
}