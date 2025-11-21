package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SubSystem.Drive;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter;
import org.firstinspires.ftc.teamcode.SubSystem.Feeder;

public abstract class AutonBase extends LinearOpMode {

    protected Drive drive;
    protected Shooter shooter;
    protected Feeder feeder;
    protected BNO055IMU imu;

    // detail telemetry toggle
    protected boolean detailMode = false;

    @Override
    public void runOpMode() {
        // Initialize subsystems
        drive = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        feeder = new Feeder(hardwareMap);

        // Initialize IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(params);

        // Wait for start
        waitForStart();

        if (opModeIsActive()) {
            // Run the autonomous routine implemented in subclasses
            runAuton();

            // Telemetry loop during autonomous
            while (opModeIsActive()) {
                telemetry.addData("Heading (deg)", imu.getAngularOrientation().firstAngle);

                // System Status Summary
                String driveStatus = "OK";
                String shooterStatus = (shooter != null) ? "OK" : "Stopped";
                String feederStatus = (feeder != null) ? "OK" : "Stopped";

                telemetry.addData("System", "Drive=%s Shooter=%s Feeder=%s",
                        driveStatus, shooterStatus, feederStatus);

                // Optional detail mode
                if (detailMode) {
                    logShooterVelocity();
                }

                telemetry.update();
            }
        }

        // Stop all subsystems safely
        drive.setDrivePower(0.0);
        shooter.stop();
        feeder.stop();
    }

    // This must be implemented in AutonRed and AutonBlue
    protected abstract void runAuton();

    /** Compact + optional detail shooter telemetry */
    protected void logShooterVelocity() {
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
}