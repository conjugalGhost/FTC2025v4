package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.SubSystem.Drive;
import org.firstinspires.ftc.teamcode.SubSystem.Feeder;
import org.firstinspires.ftc.teamcode.SubSystem.IMU;      // updated IMU subsystem
import org.firstinspires.ftc.teamcode.SubSystem.Shooter;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Scrap Cat", group = "TeleOp")
public class TeleOp extends OpMode {

    private Drive drive;
    private Shooter shooter;
    private Feeder feeder;
    private IMU imu;   // IMU subsystem

    private boolean telemetryEnabled = true;
    private boolean telemetryTogglePressed = false;

    private boolean detailMode = false;
    private boolean detailTogglePressed = false;

    // Track button press states for edge-triggering
    private boolean aWasPressed = false;
    private boolean bWasPressed = false;

    // Initialize subsystems
    @Override
    public void init() {
        drive = new Drive(hardwareMap);     // initialize drive
        shooter = new Shooter(hardwareMap); // initialize shooter
        feeder = new Feeder(hardwareMap);   // initialize feeder
        imu = new IMU(hardwareMap);   // initialize IMU
        feeder.resetEncoders();
    }
    // Main loop
    @Override
    public void loop() {
        // Drive system
        drive.driveWithGamepad(gamepad1);

        // Shooter control: right trigger = forward, left trigger = reverse
        if (gamepad2.right_trigger > 0.1) {
            shooter.shootForward();
        } else if (gamepad2.left_trigger > 0.1) {
            shooter.shootReverse();
        } else {
            shooter.stop();
        }

        // Feeder control: only works if shooter is active (forward OR reverse)
        if (gamepad2.right_trigger > 0.1 || gamepad2.left_trigger > 0.1) {
            if (gamepad2.a && !aWasPressed) {
                feeder.advanceOneStep();   // forward step
                aWasPressed = true;
            } else if (!gamepad2.a) {
                aWasPressed = false;
            }

            if (gamepad2.b && !bWasPressed) {
                feeder.reverseOneStep();   // reverse step
                bWasPressed = true;
            } else if (!gamepad2.b) {
                bWasPressed = false;
            }
        } else {
            // Shooter not active → feeder locked out
            feeder.stop();
            aWasPressed = false;
            bWasPressed = false;
        }

        // Telemetry toggle with dpad_left
        if (gamepad2.dpad_left && !telemetryTogglePressed) {
            telemetryEnabled = !telemetryEnabled;
            telemetryTogglePressed = true;
        } else if (!gamepad2.dpad_left) {
            telemetryTogglePressed = false;
        }

        // Detail mode toggle with dpad_right
        if (gamepad2.dpad_right && !detailTogglePressed) {
            detailMode = !detailMode;
            detailTogglePressed = true;
        } else if (!gamepad2.dpad_right) {
            detailTogglePressed = false;
        }

        // Telemetry output
        if (telemetryEnabled) {
            double heading = imu.getHeading();
            if (heading < 0) heading += 360; // normalize to 0–360
            telemetry.addData("Heading", "%.1f°", heading);

            // Optional detail mode
            if (detailMode) {
                double D_in = 3.54;                  // REV 90 mm grip wheels
                double ticksPerRev = 28.0 * 4.0;          // encoder ticks per revolution * gear ratio, ie: 28*4=112 ticks
                double leftTicksPerSec = shooter.getLeftVelocity();
                double rightTicksPerSec = shooter.getRightVelocity();
                double avgTicksPerSec = (leftTicksPerSec + rightTicksPerSec) / 2.0;

                double rpm = (avgTicksPerSec / ticksPerRev) * 60.0;
                double circumferenceFt = Math.PI * D_in / 12.0;
                double vWheelFtPerSec = circumferenceFt * (rpm / 60.0);

                double k = 0.85;                     // slip/compression factor
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