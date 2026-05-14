package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.SubSystem.Drive;

/**
 * Drive Calibration OpMode
 * 
 * Spin all motors at 1.0 power and measure their actual velocity.
 * It will then calculate the suggested Trim Factors to make them match.
 */
@TeleOp(name = "Drive Calibration", group = "Diagnostics")
public class DriveCalibration extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap);
        
        telemetry.addLine("DRIVE CALIBRATION");
        telemetry.addLine("-----------------");
        telemetry.addLine("Hold the robot in the air or place on blocks!");
        telemetry.addLine("Press START to begin 3-second test.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Run at full power
        drive.setDrivePower(1.0);
        
        // Wait for motors to reach steady state
        sleep(1000);

        ElapsedTime timer = new ElapsedTime();
        double flVel = 0, frVel = 0, blVel = 0, brVel = 0;
        int samples = 0;

        // Sample for 2 seconds
        while (opModeIsActive() && timer.seconds() < 2.0) {
            flVel += drive.getFrontLeft().getVelocity();
            frVel += drive.getFrontRight().getVelocity();
            blVel += drive.getBackLeft().getVelocity();
            brVel += drive.getBackRight().getVelocity();
            samples++;
            
            telemetry.addData("Status", "Sampling... %.1f s", 2.0 - timer.seconds());
            telemetry.update();
            sleep(20);
        }

        drive.stop();

        // Calculate Averages
        flVel /= samples;
        frVel /= samples;
        blVel /= samples;
        brVel /= samples;

        // Find the slowest motor (this will be our baseline = 1.0)
        double minVel = Math.min(Math.min(flVel, frVel), Math.min(blVel, brVel));

        // Calculate Trim (min / current)
        // This scales everything down to the slowest motor
        double tFL = minVel / flVel;
        double tFR = minVel / frVel;
        double tBL = minVel / blVel;
        double tBR = minVel / brVel;

        while (opModeIsActive()) {
            telemetry.addLine("CALIBRATION COMPLETE");
            telemetry.addLine("--------------------");
            telemetry.addData("FL Avg Vel", "%.1f", flVel);
            telemetry.addData("FR Avg Vel", "%.1f", frVel);
            telemetry.addData("BL Avg Vel", "%.1f", blVel);
            telemetry.addData("BR Avg Vel", "%.1f", brVel);
            telemetry.addLine("--------------------");
            telemetry.addLine("SUGGESTED TRIM FACTORS:");
            telemetry.addData("FL", "%.3f", tFL);
            telemetry.addData("FR", "%.3f", tFR);
            telemetry.addData("BL", "%.3f", tBL);
            telemetry.addData("BR", "%.3f", tBR);
            telemetry.addLine("--------------------");
            telemetry.addLine("Copy these into TeleOp drive.setTrim()");
            telemetry.update();
        }
    }
}
