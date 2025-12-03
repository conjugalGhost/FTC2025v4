package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.SubSystem.Drive;
import org.firstinspires.ftc.teamcode.SubSystem.Feeder;
import org.firstinspires.ftc.teamcode.SubSystem.IMU;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter;

public abstract class AutonBase extends LinearOpMode {
    protected Drive drive;
    protected IMU imu;
    protected Shooter shooter;
    protected Feeder feeder;

    protected boolean detailMode = false;
    protected boolean isRedAlliance = true; // override in child classes

    // Calibrated constant from field test
    protected static final double TICKS_PER_INCH = 7.207; // adjust if wheel/gear ratio changes

    @Override
    public void runOpMode() throws InterruptedException {
        drive   = new Drive(hardwareMap);
        imu     = new IMU(hardwareMap, "Bobcat"); // change to "Caracal" for that robot
        shooter = new Shooter(hardwareMap);
        feeder  = new Feeder(hardwareMap);

        imu.resetYaw();
        telemetry.addLine("Autonomous Initialized - Yaw reset to 0");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            runAuton();
        }
    }

    protected abstract void runAuton();

    // --- Drive forward/backward with heading hold (corrected) ---
    protected void driveStraightWithHeading(double inches, double power, double targetHeading) {
        int targetTicks = (int)(inches * TICKS_PER_INCH);

        DcMotorEx fl = drive.getFrontLeft();
        DcMotorEx fr = drive.getFrontRight();
        DcMotorEx bl = drive.getBackLeft();
        DcMotorEx br = drive.getBackRight();

        // Reset encoders
        fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Run open-loop
        fl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Tunable constants
        double base = power;          // positive = forward, negative = reverse
        final double kP = 0.015;      // heading correction gain
        final double biasCap = 0.25;  // max heading bias
        final double deadbandDeg = 2.0; // tolerance in degrees
        final double minMag = 0.10;   // minimum motor power
        final double timeoutSec = 5.0; // safety timeout

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < timeoutSec) {
            int avgTicks = (fl.getCurrentPosition() + fr.getCurrentPosition()
                    + bl.getCurrentPosition() + br.getCurrentPosition()) / 4;

            // Exit when distance reached
            if (base > 0 && avgTicks >= targetTicks) break;
            if (base < 0 && avgTicks <= -Math.abs(targetTicks)) break;

            // Normalize headings
            double currentHeading = imu.getHeading();
            currentHeading = ((currentHeading + 540) % 360) - 180;
            targetHeading  = ((targetHeading + 540) % 360) - 180;

            // Shortest-path error
            double err = targetHeading - currentHeading;
            err = ((err + 540) % 360) - 180;

            // Heading bias
            double bias = (Math.abs(err) < deadbandDeg) ? 0 : err * kP;
            bias = Math.max(-biasCap, Math.min(biasCap, bias));

            // Apply bias to sides
            double left  = base * (1.0 + bias);
            double right = base * (1.0 - bias);

            // Prevent unintended reversal
            if (Math.abs(left)  < minMag) left  = Math.copySign(minMag, left);
            if (Math.abs(right) < minMag) right = Math.copySign(minMag, right);

            fl.setPower(left);
            bl.setPower(left);
            fr.setPower(right);
            br.setPower(right);

            if (detailMode) {
                telemetry.addData("AvgTicks", avgTicks);
                telemetry.addData("TargetTicks", targetTicks);
                telemetry.addData("Heading", currentHeading);
                telemetry.addData("Error", err);
                telemetry.addData("Bias", bias);
                telemetry.addData("L/R", "%.2f / %.2f", left, right);
                telemetry.update();
            }
        }

        drive.stop();
    }

    // --- Turn to heading with IMU (corrected) ---
    protected void turnToHeading(double targetHeading) {
        targetHeading = ((targetHeading + 540) % 360) - 180;

        drive.setRunModeAll(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        final double kP = 0.04;
        final double minPower = 0.12;
        final double maxPower = 0.5;
        final double tolDeg = 2.0;
        final double settleHold = 0.15;
        final double timeoutSec = 2.5;

        ElapsedTime t = new ElapsedTime();
        ElapsedTime settle = new ElapsedTime();
        boolean settling = false;

        while (opModeIsActive() && t.seconds() < timeoutSec) {
            double current = imu.getHeading();
            current = ((current + 540) % 360) - 180;

            double err = targetHeading - current;
            err = ((err + 540) % 360) - 180;

            double cmd = err * kP;

            if (Math.abs(cmd) < minPower && Math.abs(err) > tolDeg) {
                cmd = Math.copySign(minPower, err);
            }
            cmd = Math.max(-maxPower, Math.min(maxPower, cmd));

            drive.turn(cmd);

            boolean insideTol = Math.abs(err) <= tolDeg;
            if (insideTol) {
                if (!settling) {
                    settling = true;
                    settle.reset();
                } else if (settle.seconds() >= settleHold) {
                    break;
                }
            } else {
                settling = false;
            }

            if (detailMode) {
                telemetry.addData("Target", targetHeading);
                telemetry.addData("Current", current);
                telemetry.addData("Error", err);
                telemetry.addData("TurnCmd", cmd);
                telemetry.update();
            }
        }

        drive.stop();
        sleep(100); // settle pause
    }

    // --- Strafe with heading hold (corrected) ---
    protected void driveStrafeWithHeading(double inches, double power, double targetHeading) {
        int targetTicks = (int)(Math.abs(inches) * TICKS_PER_INCH);

        DcMotorEx fl = drive.getFrontLeft();
        DcMotorEx fr = drive.getFrontRight();
        DcMotorEx bl = drive.getBackLeft();
        DcMotorEx br = drive.getBackRight();

        // Reset encoders
        fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Run open-loop
        fl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Tunable constants
        double base = power;          // positive = strafe right, negative = strafe left
        final double kP = 0.015;      // heading correction gain
        final double biasCap = 0.25;  // max heading bias
        final double deadbandDeg = 2.0; // tolerance in degrees
        final double minMag = 0.10;   // minimum motor power
        final double timeoutSec = 5.0; // safety timeout

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < timeoutSec) {
            // Average absolute ticks for strafing distance
            int avgTicks = (Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition())
                    + Math.abs(bl.getCurrentPosition()) + Math.abs(br.getCurrentPosition())) / 4;

            // Exit when distance reached
            if (avgTicks >= targetTicks) break;

            // Normalize headings
            double currentHeading = imu.getHeading();
            currentHeading = ((currentHeading + 540) % 360) - 180;
            targetHeading  = ((targetHeading + 540) % 360) - 180;

            // Shortest-path error
            double err = targetHeading - currentHeading;
            err = ((err + 540) % 360) - 180;

            // Heading bias
            double bias = (Math.abs(err) < deadbandDeg) ? 0 : err * kP;
            bias = Math.max(-biasCap, Math.min(biasCap, bias));

            // Mecanum strafe pattern with bias
            double flPower = base * (1.0 + bias);
            double frPower = -base * (1.0 - bias);
            double blPower = -base * (1.0 + bias);
            double brPower = base * (1.0 - bias);

            // Prevent unintended reversal
            if (Math.abs(flPower) < minMag) flPower = Math.copySign(minMag, flPower);
            if (Math.abs(frPower) < minMag) frPower = Math.copySign(minMag, frPower);
            if (Math.abs(blPower) < minMag) blPower = Math.copySign(minMag, blPower);
            if (Math.abs(brPower) < minMag) brPower = Math.copySign(minMag, brPower);

            fl.setPower(flPower);
            fr.setPower(frPower);
            bl.setPower(blPower);
            br.setPower(brPower);

            if (detailMode) {
                telemetry.addData("AvgTicks", avgTicks);
                telemetry.addData("TargetTicks", targetTicks);
                telemetry.addData("Heading", currentHeading);
                telemetry.addData("Error", err);
                telemetry.addData("Bias", bias);
                telemetry.addData("FL/FR/BL/BR", "%.2f / %.2f / %.2f / %.2f", flPower, frPower, blPower, brPower);
                telemetry.update();
            }
        }

        drive.stop();
    }

    // --- Subsystem helpers ---
    protected void resetHeading() { imu.resetYaw(); }

    protected void spinShooterForward() { shooter.shootForward(); }
    protected void spinShooterReverse() { shooter.shootReverse(); }
    protected void stopShooter() { shooter.stop(); }

    protected void feedForwardStep() { feeder.advanceOneStep(); }
    protected void feedReverseStep() { feeder.reverseOneStep(); }
    protected void stopFeeder() { feeder.stop(); }

    protected void logShooterVelocity() {
        telemetry.addData("Shooter Left Vel", shooter.getLeftVelocity());
        telemetry.addData("Shooter Right Vel", shooter.getRightVelocity());
        telemetry.update();
    }
}