package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.SubSystem.Feeder;
import org.firstinspires.ftc.teamcode.SubSystem.IMU;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter;
import org.firstinspires.ftc.teamcode.SubSystem.Drive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Scrap Cat", group = "TeleOp")
public class TeleOp extends OpMode {

    private Drive drive;
    private Shooter shooter;
    private Feeder feeder;
    private IMU imu;   // IMU subsystem

    private boolean telemetryEnabled = true;
    private boolean togglePressed = false;

    // Track button press states for edge-triggering
    private boolean aWasPressed = false;
    private boolean bWasPressed = false;

    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        feeder = new Feeder(hardwareMap);
        imu = new IMU(hardwareMap);   // initialize IMU
        feeder.resetEncoders();
    }

    @Override
    public void loop() {
        // Drive system
        drive.driveWithGamepad(gamepad1);

        // Shooter control on right trigger
        if (gamepad2.right_trigger > 0.1) {
            shooter.shootForward();
        } else {
            shooter.stop();
        }

        // Feeder control: A = step forward, B = step reverse
        if (gamepad2.a && !aWasPressed) {
            if (shooter.isReady()) {
                feeder.advanceOneStep();
            }
            aWasPressed = true;
        } else if (!gamepad2.a) {
            aWasPressed = false;
        }

        if (gamepad2.b && !bWasPressed) {
            if (shooter.isReady()) {
                feeder.reverseOneStep();
            }
            bWasPressed = true;
        } else if (!gamepad2.b) {
            bWasPressed = false;
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

            telemetry.addData("Heading", imu.getHeading());  // show IMU heading


            // --- Shooter velocity conversion to ft/s ---
            double D_in = 4.0;                  // wheel diameter in inches
            double ticksPerRev = 560.0;         // encoder ticks per revolution
            double leftTicksPerSec = shooter.getLeftVelocity();
            double rightTicksPerSec = shooter.getRightVelocity();

            double leftRPM = (leftTicksPerSec / ticksPerRev) * 60.0;
            double rightRPM = (rightTicksPerSec / ticksPerRev) * 60.0;
            double avgRPM = (leftRPM + rightRPM) / 2.0;

            double vWheelFtPerSec = (Math.PI * D_in / 12.0) * (avgRPM / 60.0);

            double k = 0.85;                    // slip/compression factor
            double vExitFtPerSec = k * vWheelFtPerSec;

            telemetry.addData("Shooter v_wheel (ft/s)", vWheelFtPerSec);
            telemetry.addData("Shooter v_exit (ft/s)", vExitFtPerSec);

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