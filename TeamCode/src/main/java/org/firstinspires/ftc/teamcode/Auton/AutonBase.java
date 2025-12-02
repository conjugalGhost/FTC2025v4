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
    protected static final double TICKS_PER_INCH = 7.207;

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

    // --- Drive helpers ---
    protected void driveForwardInches(double inches, double power) {
        int ticks = (int)(inches * TICKS_PER_INCH);

        DcMotorEx fl = drive.getFrontLeft();
        DcMotorEx fr = drive.getFrontRight();
        DcMotorEx bl = drive.getBackLeft();
        DcMotorEx br = drive.getBackRight();

        fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        fl.setTargetPosition(ticks);
        fr.setTargetPosition(ticks);
        bl.setTargetPosition(ticks);
        br.setTargetPosition(ticks);

        fl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        drive.setDrivePower(power);

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() &&
                (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy()) &&
                timer.seconds() < 5) {
            if (detailMode) {
                telemetry.addData("FL pos", fl.getCurrentPosition());
                telemetry.addData("FR pos", fr.getCurrentPosition());
                telemetry.addData("BL pos", bl.getCurrentPosition());
                telemetry.addData("BR pos", br.getCurrentPosition());
                telemetry.update();
            }
        }
        drive.stop();
    }

    protected void driveStraightWithHeading(double inches, double power, double targetHeading) {
        int ticks = (int)(inches * TICKS_PER_INCH);

        DcMotorEx fl = drive.getFrontLeft();
        DcMotorEx fr = drive.getFrontRight();
        DcMotorEx bl = drive.getBackLeft();
        DcMotorEx br = drive.getBackRight();

        fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        fl.setTargetPosition(ticks);
        fr.setTargetPosition(ticks);
        bl.setTargetPosition(ticks);
        br.setTargetPosition(ticks);

        fl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        drive.setDrivePower(power);

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() &&
                (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy()) &&
                timer.seconds() < 5) {
            double currentHeading = imu.getHeading();
            double error = targetHeading - currentHeading;
            error = (error + 540) % 360 - 180;

            double kP = 0.06;
            double correction = error * kP;
            if (Math.abs(correction) < 0.12) {
                correction = Math.copySign(0.12, correction);
            }

            fl.setPower(power + correction);
            bl.setPower(power + correction);
            fr.setPower(power - correction);
            br.setPower(power - correction);

            if (detailMode) {
                telemetry.addData("Heading", currentHeading);
                telemetry.addData("Error", error);
                telemetry.addData("Correction", correction);
                telemetry.update();
            }
        }
        drive.stop();
    }

    // --- Corrected turnToHeading ---
    protected void turnToHeading(double targetHeading) {
        // Normalize target to [-180, 180]
        targetHeading = ((targetHeading + 540) % 360) - 180;

        // Ensure open-loop control
        drive.setRunModeAll(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        final double kP = 0.05;
        final double minPower = 0.18;
        final double maxPower = 0.6;
        final double onTargetError = 1.5;
        final double settleTime = 0.15;

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime settleTimer = new ElapsedTime();
        boolean settling = false;

        while (opModeIsActive() && timer.seconds() < 3.0) {
            double current = imu.getHeading();
            current = ((current + 540) % 360) - 180;

            double error = targetHeading - current;
            error = ((error + 540) % 360) - 180;

            double turnPower = error * kP;

            if (Math.abs(turnPower) < minPower && Math.abs(error) > onTargetError) {
                turnPower = Math.copySign(minPower, turnPower == 0 ? 1 : turnPower);
            }

            if (turnPower > maxPower) turnPower = maxPower;
            if (turnPower < -maxPower) turnPower = -maxPower;

            drive.turn(turnPower);

            if (Math.abs(error) <= onTargetError) {
                if (!settling) {
                    settling = true;
                    settleTimer.reset();
                } else if (settleTimer.seconds() >= settleTime) {
                    break;
                }
            } else {
                settling = false;
            }

            if (detailMode) {
                telemetry.addData("Target", targetHeading);
                telemetry.addData("Current", current);
                telemetry.addData("Error", error);
                telemetry.addData("TurnPower", turnPower);
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