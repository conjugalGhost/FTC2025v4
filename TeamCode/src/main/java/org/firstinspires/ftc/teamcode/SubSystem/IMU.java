package org.firstinspires.ftc.teamcode.SubSystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IMU {
    private final com.qualcomm.robotcore.hardware.IMU imu;

    public IMU(HardwareMap hardwareMap) {
        // Get the IMU from the hardware map (BHI260AP inside Control Hub)
        imu = hardwareMap.get(com.qualcomm.robotcore.hardware.IMU.class, "imu");

        // Define the orientation of the Hub on the robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Initialize the IMU with the specified orientation
        imu.initialize(new com.qualcomm.robotcore.hardware.IMU.Parameters(orientationOnRobot));

        // Reset yaw to 0 at startup
        resetYaw();
    }

    /** Returns the robot's yaw (heading) in degrees. */
    public double getHeading() {
        return AngleUnit.DEGREES.normalize(
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)
        );
    }

    /** Returns the robot's pitch in degrees. */
    public double getPitch() {
        return imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
    }

    /** Returns the robot's roll in degrees. */
    public double getRoll() {
        return imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
    }

    /** Resets the yaw so the current heading becomes 0°. */
    public void resetYaw() {
        imu.resetYaw();
    }

    /** Logs yaw/pitch/roll to telemetry (Option 1: pass Telemetry in). */
    public void logTelemetry(Telemetry telemetry) {
        telemetry.addData("Heading", getHeading());
        telemetry.addData("Pitch", getPitch());
        telemetry.addData("Roll", getRoll());
        telemetry.update();
    }
}