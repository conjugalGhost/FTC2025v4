package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
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

    /** Forward drive helper */
    protected void driveForwardInches(double inches, double power) {
        drive.driveForwardInches(inches, power, this);
    }

    /** Heading-corrected drive helper */
    protected void driveStraightWithHeading(double inches, double power, double targetHeading) {
        drive.driveStraightWithHeading(inches, power, targetHeading, this, imu);
    }

    /** Turn helper using IMU heading with normalization + timeout */
    protected void turnToHeading(double targetHeading) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < 5) {
            double currentHeading = imu.getHeading();
            double error = targetHeading - currentHeading;
            error = (error + 540) % 360 - 180; // normalize to -180..180

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

    /** Reset IMU yaw */
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