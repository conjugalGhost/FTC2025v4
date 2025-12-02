package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.SubSystem.Drive;
import org.firstinspires.ftc.teamcode.SubSystem.Feeder;
import org.firstinspires.ftc.teamcode.SubSystem.IMU;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Scrap Cat", group = "TeleOp")
public class TeleOp extends OpMode {

    private Drive drive;
    private Shooter shooter;
    private Feeder feeder;
    private IMU imu;

    private boolean telemetryEnabled = true;
    private boolean telemetryTogglePressed = false;

    private boolean detailMode = false;
    private boolean detailTogglePressed = false;

    private boolean aWasPressed = false;
    private boolean bWasPressed = false;

    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        feeder = new Feeder(hardwareMap);
        imu = new IMU(hardwareMap);
        feeder.resetEncoders();
    }

    @Override
    public void loop() {
        // --- Mecanum drive control using gamepad1 ---
        double y  = -gamepad1.left_stick_y; // forward/back
        double x  = gamepad1.left_stick_x;  // strafe
        double rx = gamepad1.right_stick_x; // rotation

        double frontLeftPower  = y + x + rx;
        double backLeftPower   = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower  = y + x - rx;

        double max = Math.max(1.0, Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(backLeftPower),
                        Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)))));

        drive.getFrontLeft().setPower(frontLeftPower / max);
        drive.getBackLeft().setPower(backLeftPower / max);
        drive.getFrontRight().setPower(frontRightPower / max);
        drive.getBackRight().setPower(backRightPower / max);

        // --- Shooter control ---
        if (gamepad2.right_trigger > 0.1) {
            shooter.shootForward();
        } else if (gamepad2.left_trigger > 0.1) {
            shooter.shootReverse();
        } else {
            shooter.stop();
        }

        // --- Feeder control (only if shooter active) ---
        if (gamepad2.right_trigger > 0.1 || gamepad2.left_trigger > 0.1) {
            if (gamepad2.a && !aWasPressed) {
                feeder.advanceOneStep();
                aWasPressed = true;
            } else if (!gamepad2.a) {
                aWasPressed = false;
            }

            if (gamepad2.b && !bWasPressed) {
                feeder.reverseOneStep();
                bWasPressed = true;
            } else if (!gamepad2.b) {
                bWasPressed = false;
            }
        } else {
            feeder.stop();
            aWasPressed = false;
            bWasPressed = false;
        }

        // --- Telemetry toggles ---
        if (gamepad2.dpad_left && !telemetryTogglePressed) {
            telemetryEnabled = !telemetryEnabled;
            telemetryTogglePressed = true;
        } else if (!gamepad2.dpad_left) {
            telemetryTogglePressed = false;
        }

        if (gamepad2.dpad_right && !detailTogglePressed) {
            detailMode = !detailMode;
            detailTogglePressed = true;
        } else if (!gamepad2.dpad_right) {
            detailTogglePressed = false;
        }

        // --- Telemetry output ---
        if (telemetryEnabled) {
            double heading = imu.getHeading();
            if (heading < 0) heading += 360;
            telemetry.addData("Heading", "%.1f°", heading);

            if (detailMode) {
                double D_in = 3.54;
                double ticksPerRev = 28.0 * 4.0;
                double leftTicksPerSec = shooter.getLeftVelocity();
                double rightTicksPerSec = shooter.getRightVelocity();
                double avgTicksPerSec = (leftTicksPerSec + rightTicksPerSec) / 2.0;

                double rpm = (avgTicksPerSec / ticksPerRev) * 60.0;
                double circumferenceFt = Math.PI * D_in / 12.0;
                double vWheelFtPerSec = circumferenceFt * (rpm / 60.0);

                double k = 0.85;
                double vExitFtPerSec = k * vWheelFtPerSec;

                telemetry.addData("Shooter (ft/s)", "Wheel=%.2f Exit=%.2f", vWheelFtPerSec, vExitFtPerSec);
                telemetry.addData("Shooter RPM", "%.0f", rpm);
            }

            telemetry.update();
        }
    }

    @Override
    public void stop() {
        shooter.stop();
        drive.setDrivePower(0);
        feeder.stop();
    }
}