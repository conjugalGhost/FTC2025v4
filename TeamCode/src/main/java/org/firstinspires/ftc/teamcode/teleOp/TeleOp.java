package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.SubSystem.Feeder;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter;
import org.firstinspires.ftc.teamcode.SubSystem.Drive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Scrap Cat", group = "TeleOp")
public class TeleOp extends OpMode {

    private Drive drive;
    private Shooter shooter;
    private Feeder feeder;

    private boolean telemetryEnabled = true;
    private boolean togglePressed = false;

    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        feeder = new Feeder(hardwareMap);
    }

    @Override
    public void loop() {
        // Drive system
        drive.driveWithGamepad(gamepad1);

        // Shooter control on right trigger
        if (gamepad2.right_trigger > 0.1) {   // threshold to avoid accidental touch
            shooter.shootForward();
        } else {
            shooter.stop();
        }

        // Feeder control: A forward, B reverse
        if (gamepad2.a) {
            if (shooter.isReady()) {
                feeder.feedForward();
            } else {
                feeder.stop(); // prevent feeding until shooter is spun up
            }
        } else if (gamepad2.b) {
            feeder.feedReverse();
        } else {
            feeder.stop();
        }

        // Telemetry toggle with dpad_left
        if (gamepad2.dpad_left && !togglePressed) {
            telemetryEnabled = !telemetryEnabled;
            togglePressed = true;
        } else if (!gamepad2.dpad_left) {
            togglePressed = false;
        }

        // Telemetry output
        if (telemetryEnabled) {
            telemetry.addLine("=== DRIVE ===");
            drive.updateTelemetry(telemetry);

            telemetry.addLine("=== SHOOTER ===");
            shooter.updateTelemetry(telemetry);

            telemetry.addLine("=== FEEDER ===");
            feeder.updateTelemetry(telemetry);

            // Shooter velocity check
            String shooterStatus = "OK";
            double targetVel = shooter.getTargetVelocity();
            double leftVel = shooter.getLeftVelocity();
            double rightVel = shooter.getRightVelocity();
            if (targetVel > 0 &&
                    (Math.abs(leftVel - targetVel) > targetVel * 0.1 ||
                            Math.abs(rightVel - targetVel) > targetVel * 0.1)) {
                shooterStatus = "Warning (Velocity Low)";
            }

            telemetry.addLine("=== SYSTEM STATUS ===")
                    .addData("Drive", "OK")
                    .addData("Shooter", shooterStatus)
                    .addData("Feeder", "OK");

            telemetry.update();
        }
    }
}