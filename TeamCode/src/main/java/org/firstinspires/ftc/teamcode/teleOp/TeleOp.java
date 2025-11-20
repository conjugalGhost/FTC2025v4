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
        drive.driveWithGamepad(gamepad1);

        if (gamepad2.a) shooter.shootForward();
        else if (gamepad2.b) shooter.stop();


        if (gamepad2.x) {
            if (shooter.isReady()) {
                feeder.feedForward();
            } else {
                feeder.stop(); // prevent feeding until shooter is spun up
            }
        } else if (gamepad2.y) {
            feeder.feedReverse();
        } else {
            feeder.stop();
        }

        // Toggle telemetry with dpad_left
        if (gamepad2.dpad_left && !togglePressed) {
            telemetryEnabled = !telemetryEnabled;
            togglePressed = true;
        } else if (!gamepad2.dpad_left) {
            togglePressed = false;
        }

        if (telemetryEnabled) {
            telemetry.addLine("=== DRIVE ===");
            drive.updateTelemetry(telemetry);

            telemetry.addLine("=== SHOOTER ===");
            shooter.updateTelemetry(telemetry);

            telemetry.addLine("=== FEEDER ===");
            feeder.updateTelemetry(telemetry);

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
