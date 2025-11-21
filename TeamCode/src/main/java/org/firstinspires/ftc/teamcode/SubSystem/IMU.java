package org.firstinspires.ftc.teamcode.SubSystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IMU {
    private final com.qualcomm.robotcore.hardware.IMU imu;

    public IMU(HardwareMap hardwareMap) {
        // Get the IMU from the hardware map
        imu = hardwareMap.get(com.qualcomm.robotcore.hardware.IMU.class, "imu");

        // Define the orientation of the Hub on the robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Initialize the IMU with the specified orientation
        imu.initialize(new com.qualcomm.robotcore.hardware.IMU.Parameters(orientationOnRobot));

        // Reset the yaw to a heading of 0
        resetYaw();
    }

    /**
     * Gets the current heading of the robot in degrees.
     * @return The robot's yaw angle in degrees.
     */
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * Resets the robot's yaw. This will set the current heading to 0 degrees.
     */
    public void resetYaw() {
        imu.resetYaw();
    }
}
