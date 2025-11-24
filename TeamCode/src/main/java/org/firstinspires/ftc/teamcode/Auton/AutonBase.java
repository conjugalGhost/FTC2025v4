package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.SubSystem.Drive;
import org.firstinspires.ftc.teamcode.SubSystem.IMU;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter;
import org.firstinspires.ftc.teamcode.SubSystem.Feeder;

public abstract class AutonBase extends LinearOpMode {
    protected Drive drive;
    protected IMU imu;
    protected Shooter shooter;
    protected Feeder feeder;

    // Flag for optional detailed telemetry
    protected boolean detailMode = false;

    // Calibration constant for encoder conversion
    protected static final double TICKS_PER_INCH = 7.6;

    // Run the auton routine
    @Override
    public void runOpMode() throws InterruptedException {
        drive   = new Drive(hardwareMap);
        imu     = new IMU(hardwareMap);
        shooter = new Shooter(hardwareMap);
        feeder  = new Feeder(hardwareMap);

        telemetry.addLine("Autonomous Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            runAuton();
        }
    }

    // Each auton routine must implement this
    protected abstract void runAuton();

    /** Encoder-based forward drive with telemetry */
    protected void driveForwardInches(double inches, double power) {
        int ticks = (int)(inches * TICKS_PER_INCH);

        DcMotorEx fl = drive.getFrontLeft();
        DcMotorEx fr = drive.getFrontRight();
        DcMotorEx bl = drive.getBackLeft();
        DcMotorEx br = drive.getBackRight();

        // Reset encoders
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

        while (opModeIsActive() &&
                (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy())) {
            telemetry.addData("FL pos", fl.getCurrentPosition());
            telemetry.addData("FR pos", fr.getCurrentPosition());
            telemetry.addData("BL pos", bl.getCurrentPosition());
            telemetry.addData("BR pos", br.getCurrentPosition());
            telemetry.update();
        }

        drive.stop();
    }

    /** Drive forward with IMU heading correction */
    protected void driveStraightWithHeading(double inches, double power, double targetHeading) {
        int ticks = (int)(inches * TICKS_PER_INCH);

        DcMotorEx fl = drive.getFrontLeft();
        DcMotorEx fr = drive.getFrontRight();
        DcMotorEx bl = drive.getBackLeft();
        DcMotorEx br = drive.getBackRight();

        // Reset encoders
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

        while (opModeIsActive() &&
                (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy())) {
            double currentHeading = imu.getHeading();
            double error = targetHeading - currentHeading;
            error = (error + 540) % 360 - 180; // normalize to -180..180
            double correction = error * 0.02;  // proportional gain

            fl.setPower(power + correction);
            bl.setPower(power + correction);
            fr.setPower(power - correction);
            br.setPower(power - correction);

            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Error", error);
            telemetry.addData("FL pos", fl.getCurrentPosition());
            telemetry.addData("FR pos", fr.getCurrentPosition());
            telemetry.update();
        }

        drive.stop();
    }

    /** Turn helper using IMU heading with normalization + timeout */
    protected void turnToHeading(double targetHeading) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < 5) {
            double currentHeading = imu.getHeading();
            double error = targetHeading - currentHeading;
            error = (error + 540) % 360 - 180; // normalize

            if (Math.abs(error) < 2) break;

            double turnPower = error > 0 ? 0.3 : -0.3;
            drive.turn(turnPower);

            telemetry.addData("Target", targetHeading);
            telemetry.addData("Current", currentHeading);
            telemetry.addData("Error", error);
            telemetry.update();
        }
        drive.stop();
    }

    /** Reset IMU yaw (only if you explicitly want to re-zero) */
    protected void resetHeading() {
        imu.resetYaw();
    }

    /** Shooter helpers */
    protected void spinShooterForward() { shooter.shootForward(); }
    protected void spinShooterReverse() { shooter.shootReverse(); }
    protected void stopShooter() { shooter.stop(); }

    /** Feeder helpers */
    protected void feedForwardStep() { feeder.advanceOneStep(); }
    protected void feedReverseStep() { feeder.reverseOneStep(); }
    protected void stopFeeder() { feeder.stop(); }

    /** Shooter telemetry helper */
    protected void logShooterVelocity() {
        telemetry.addData("Shooter Left Vel", shooter.getLeftVelocity());
        telemetry.addData("Shooter Right Vel", shooter.getRightVelocity());
        telemetry.update();
    }
}