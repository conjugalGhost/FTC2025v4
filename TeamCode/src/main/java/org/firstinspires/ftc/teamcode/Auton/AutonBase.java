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
    protected static final double TICKS_PER_INCH = 7.207; // NOTE: adjust if wheel/gear ratio changes

    @Override
    public void runOpMode() throws InterruptedException {
        drive   = new Drive(hardwareMap);
        imu     = new IMU(hardwareMap);
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

    // --- Open-loop drive with heading hold ---
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

        // Run open-loop (no internal PID)
        fl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        double base = power; // NOTE: positive = forward, negative = reverse
        double kP = 0.02;    // NOTE: heading correction gain
        double biasCap = 0.35; // NOTE: max heading bias
        double deadbandDeg = 2.0; // NOTE: heading tolerance
        double minMag = 0.12; // NOTE: minimum motor power to prevent stall
        double timeoutSec = 5.0; // NOTE: safety timeout

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < timeoutSec) {
            int avgTicks = (fl.getCurrentPosition() + fr.getCurrentPosition()
                    + bl.getCurrentPosition() + br.getCurrentPosition()) / 4;

            // Exit when distance reached
            if (base > 0 && avgTicks >= targetTicks) break;
            if (base < 0 && avgTicks <= -Math.abs(targetTicks)) break;

            double currentHeading = imu.getHeading();
            double err = targetHeading - currentHeading;
            err = (err + 540) % 360 - 180;

            double bias = err * kP;
            if (Math.abs(err) < deadbandDeg) bias = 0;
            bias = Math.max(-biasCap, Math.min(biasCap, bias));

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

    // --- Corrected turnToHeading ---
    protected void turnToHeading(double targetHeading) {
        targetHeading = ((targetHeading + 540) % 360) - 180;

        drive.setRunModeAll(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        final double kP = 0.05;       // NOTE: turn gain
        final double minPower = 0.18; // NOTE: minimum turning power
        final double maxPower = 0.6;  // NOTE: maximum turning power
        final double tolDeg = 1.5;    // NOTE: tolerance in degrees
        final double settleHold = 0.12; // NOTE: hold time on target (s)
        final double timeoutSec = 3.0;  // NOTE: turn timeout

        ElapsedTime t = new ElapsedTime();
        ElapsedTime settle = new ElapsedTime();
        boolean settling = false;

        while (opModeIsActive() && t.seconds() < timeoutSec) {
            double current = imu.getHeading();
            current = ((current + 540) % 360) - 180;

            double err = targetHeading - current;
            err = ((err + 540) % 360) - 180;

            double cmd = err * kP;

            boolean insideTol = Math.abs(err) <= tolDeg;
            if (!insideTol && Math.abs(cmd) < minPower) {
                cmd = Math.copySign(minPower, cmd == 0 ? 1 : cmd);
            }

            cmd = Math.max(-maxPower, Math.min(maxPower, cmd));
            drive.turn(cmd);

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
        sleep(80); // NOTE: small pause to damp motion
    }

    protected void driveStrafeWithHeading(double inches, double power, double targetHeading) {
        // Similar to driveStraightWithHeading, but motor power pattern:
        // FL = +power, FR = -power, BL = -power, BR = +power (for right strafe)
        // Reverse signs for left strafe
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