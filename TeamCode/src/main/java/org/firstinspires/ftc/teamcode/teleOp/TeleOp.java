package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
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

    private boolean aWasPressed = false;
    
    // Timer for non-blocking feeder
    private final ElapsedTime feederTimer = new ElapsedTime();
    private boolean feederRunning = false;

    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        // Applying trim: backRight is slow, so we reduce the others to match it
        // Adjust these numbers (e.g., 0.90) until the robot drives straight
        drive.setTrim(0.95, 0.95, 0.95, 1.0); 

        shooter = new Shooter(hardwareMap);
        feeder = new Feeder(hardwareMap);
        try {
            imu = new IMU(hardwareMap, "Caracal");
        } catch (Exception e) {
            imu = null;
        }
        feeder.resetEncoders();
    }

    @Override
    public void loop() {
        double y  = -gamepad1.left_stick_y;
        double x  = gamepad1.left_stick_x;  
        double rx = gamepad1.right_stick_x; 

        // Standard Mecanum Drive Formula
        double frontLeftPower  = y + x + rx;
        double backLeftPower   = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower  = y + x - rx;

        double max = Math.max(1.0, Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(backLeftPower),
                        Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)))));

        if (drive.getFrontLeft()  != null) drive.setMotorPowers(frontLeftPower / max, frontRightPower / max, backLeftPower / max, backRightPower / max);

        // --- Shooter control (Gamepad 2) ---
        if (gamepad2.right_trigger > 0.1) {
            shooter.shootForward();
        } else if (gamepad2.left_trigger > 0.1) {
            shooter.shootReverse();
        } else {
            shooter.stop();
        }

        // --- Non-blocking Feeder control ---
        if (gamepad2.a && !aWasPressed) {
            feeder.advanceOneStep(); // This now starts the movement
            feederTimer.reset();
            feederRunning = true;
            aWasPressed = true;
        } else if (!gamepad2.a) {
            aWasPressed = false;
        }

        // Stop feeder after 0.5 seconds automatically to avoid blocking
        if (feederRunning && feederTimer.seconds() > 0.5) {
            feeder.stop();
            feederRunning = false;
        }

        // --- Telemetry output ---
        telemetry.addData("--- DRIVE DIAGNOSTICS ---", "");
        telemetry.addData("Stick Y", "%.2f", gamepad1.left_stick_y);
        telemetry.addData("Calculated Y", "%.2f", y);
        telemetry.addData("Motors Found", "%s %s %s %s",
                drive.getFrontLeft() != null ? "FL" : "--",
                drive.getFrontRight() != null ? "FR" : "--",
                drive.getBackLeft() != null ? "BL" : "--",
                drive.getBackRight() != null ? "BR" : "--");

        if (imu != null) {
            try {
                telemetry.addData("Heading", "%.1f", imu.getHeading());
            } catch (Exception e) {
                telemetry.addData("IMU", "Not Responding");
            }
        }
        
        if (shooter != null) {
            shooter.updateTelemetry(telemetry);
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        if (shooter != null) shooter.stop();
        if (drive != null) drive.stop();
        if (feeder != null) feeder.stop();
    }
}
