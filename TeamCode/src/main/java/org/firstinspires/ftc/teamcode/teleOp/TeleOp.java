package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.SubSystem.Feeder;
import org.firstinspires.ftc.teamcode.SubSystem.IMU;      // <-- corrected import
import org.firstinspires.ftc.teamcode.SubSystem.Shooter;
import org.firstinspires.ftc.teamcode.SubSystem.Drive;

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
            telemetry.addData("Heading", "%.1f°", imu.getHeading());

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

            telemetry.addData("System", "Drive=OK Shooter=%s Feeder=OK", shooterStatus);

            // Optional detail mode
            if (detailMode) {
                double D_in = 3.54;                  // REV 90 mm grip wheels
                double ticksPerRev = 560.0;          // encoder ticks per revolution
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
}