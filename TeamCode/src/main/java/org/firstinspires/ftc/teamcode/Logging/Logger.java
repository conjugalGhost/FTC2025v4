package org.firstinspires.ftc.teamcode.Logging;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter;
import org.firstinspires.ftc.teamcode.SubSystem.Drive;
import org.firstinspires.ftc.teamcode.SubSystem.IMU;
import org.firstinspires.ftc.teamcode.SubSystem.Feeder;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name="Unified Logger", group="Logging")
public class Logger extends LinearOpMode {
    private Shooter shooter;
    private Drive drive;
    private IMU imu;
    private Feeder feeder;
    private FileWriter writer;
    private VoltageSensor battery;

    // Example telemetry flags (replace with your actual toggles if needed)
    private boolean telemetryEnabled = true;
    private boolean detailMode = true;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Shooter(hardwareMap);
        drive   = new Drive(hardwareMap);
        imu     = new IMU(hardwareMap);
        feeder  = new Feeder(hardwareMap);

        // Grab first voltage sensor (usually Expansion Hub)
        battery = hardwareMap.voltageSensor.iterator().next();

        try {
            // Create CSV file on Robot Controller phone
            writer = new FileWriter("/sdcard/FIRST/match_log.csv");

            // Expanded header row
            writer.write("Time(ms),ShooterRPM,ExitVelocityFtPerSec,ShooterTargetPower,BatteryVoltage,"
                    + "FL_ticks,FR_ticks,BL_ticks,BR_ticks,"
                    + "FL_power,FR_power,BL_power,BR_power,"
                    + "Heading,FeederState,FeederTicks,"
                    + "ShooterLeftVel,ShooterRightVel,ShooterAvgVel,"
                    + "TelemetryEnabled,DetailMode\n");
        } catch (IOException e) {
            telemetry.addLine("Failed to open log file");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            // Shooter values
            double rpm = shooter.getRPM();
            double exitVel = shooter.getExitVelocityFtPerSec();
            double targetPower = shooter.getTargetPower();

            // Shooter velocities
            double leftVel = shooter.getLeftVelocity();
            double rightVel = shooter.getRightVelocity();
            double avgVel = (leftVel + rightVel) / 2.0;

            // Drive encoder positions
            int fl = drive.getFrontLeft().getCurrentPosition();
            int fr = drive.getFrontRight().getCurrentPosition();
            int bl = drive.getBackLeft().getCurrentPosition();
            int br = drive.getBackRight().getCurrentPosition();

            // Drive motor powers
            double flPower = drive.getFrontLeft().getPower();
            double frPower = drive.getFrontRight().getPower();
            double blPower = drive.getBackLeft().getPower();
            double brPower = drive.getBackRight().getPower();

            // IMU heading
            double heading = imu.getHeading();

            // Feeder state and ticks
            int feederState = feeder.getState();
            int feederTicks = feeder.getCurrentPosition();

            // Battery voltage
            double voltage = battery.getVoltage();

            // Log to CSV
            try {
                writer.write(String.format("%d,%.0f,%.2f,%.2f,%.2f,"
                                + "%d,%d,%d,%d,"
                                + "%.2f,%.2f,%.2f,%.2f,"
                                + "%.2f,%d,%d,"
                                + "%.2f,%.2f,%.2f,"
                                + "%b,%b\n",
                        System.currentTimeMillis(), rpm, exitVel, targetPower, voltage,
                        fl, fr, bl, br,
                        flPower, frPower, blPower, brPower,
                        heading, feederState, feederTicks,
                        leftVel, rightVel, avgVel,
                        telemetryEnabled, detailMode));
                writer.flush();
            } catch (IOException e) {
                telemetry.addLine("Failed to write log");
            }

            // Show live telemetry too
            shooter.updateTelemetry(telemetry);
            feeder.updateTelemetry(telemetry);
            telemetry.addData("Heading", heading);
            telemetry.addData("Battery", voltage);
            telemetry.update();
        }

        // Close file when done
        try {
            writer.close();
        } catch (IOException e) {
            telemetry.addLine("Failed to close log file");
            telemetry.update();
        }
    }
}