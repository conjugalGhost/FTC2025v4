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
                telemetry.addLine("=== DRIVE ===");
                drive.updateTelemetry(telemetry);

                telemetry.addLine("=== SHOOTER ===");
                shooter.updateTelemetry(telemetry);

                telemetry.addLine("=== FEEDER ===");
                feeder.updateTelemetry(telemetry);

                telemetry.addLine("=== IMU ===")
                        .addData("Heading (deg)", imu.getAngularOrientation().firstAngle);

                // System Status Summary
                String driveStatus = "OK";
                String shooterStatus = (shooter != null) ? "OK" : "Stopped";
                String feederStatus = (feeder != null) ? "OK" : "Stopped";

                telemetry.addLine("=== SYSTEM STATUS ===")
                        .addData("Drive", driveStatus)
                        .addData("Shooter", shooterStatus)
                        .addData("Feeder", feederStatus);

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
}