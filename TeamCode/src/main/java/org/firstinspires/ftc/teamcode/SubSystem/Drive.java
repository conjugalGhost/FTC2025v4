package org.firstinspires.ftc.teamcode.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Drive {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private BNO055IMU imu;
    private Orientation angles;
    private double velocityScale = 2000.0;
    private double targetFL, targetFR, targetBL, targetBR;

    // Adjust this constant based on your wheel diameter and encoder ticks per revolution
    private static final double TICKS_PER_INCH = 45.0; // placeholder value

    public Drive(HardwareMap hardwareMap) {
        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRight");

        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        PIDFCoefficients drivePIDF = new PIDFCoefficients(10.0, 0.0, 0.0, 2.0);
        frontLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, drivePIDF);
        frontRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, drivePIDF);
        backLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, drivePIDF);
        backRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, drivePIDF);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.mode = BNO055IMU.SensorMode.IMU;
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(params);
    }

    // TeleOp control with gamepad
    public void driveWithGamepad(com.qualcomm.robotcore.hardware.Gamepad gamepad) {
        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x;

        angles = imu.getAngularOrientation();
        double heading = angles.firstAngle;
        double correction = heading * 0.05;

        double fl = y + x + rx + correction;
        double bl = y - x + rx + correction;
        double fr = y - x - rx + correction;
        double br = y + x - rx + correction;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br))));
        if (max > 1.0) { fl/=max; bl/=max; fr/=max; br/=max; }

        targetFL = fl * velocityScale;
        targetFR = fr * velocityScale;
        targetBL = bl * velocityScale;
        targetBR = br * velocityScale;

        frontLeft.setVelocity(targetFL);
        frontRight.setVelocity(targetFR);
        backLeft.setVelocity(targetBL);
        backRight.setVelocity(targetBR);
    }

    // Stop all motors with a single call
    public void setDrivePower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    // Simple encoder-based forward drive
    public void driveForwardInches(double inches, double power) {
        int ticks = (int)(inches * TICKS_PER_INCH);

        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            // Optionally add telemetry updates here
        }

        setDrivePower(0.0);
    }

    // Simple IMU-based turn
    public void gyroTurn(double targetAngle, double power) {
        double currentAngle = imu.getAngularOrientation().firstAngle;

        while (Math.abs(currentAngle - targetAngle) > 2) {
            if (currentAngle < targetAngle) {
                frontLeft.setPower(power);
                backLeft.setPower(power);
                frontRight.setPower(-power);
                backRight.setPower(-power);
            } else {
                frontLeft.setPower(-power);
                backLeft.setPower(-power);
                frontRight.setPower(power);
                backRight.setPower(power);
            }
            currentAngle = imu.getAngularOrientation().firstAngle;
        }

        setDrivePower(0.0);
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("=== DRIVE ===")
                .addData("Target FL", targetFL).addData("Actual FL", frontLeft.getVelocity())
                .addData("Target FR", targetFR).addData("Actual FR", frontRight.getVelocity())
                .addData("Target BL", targetBL).addData("Actual BL", backLeft.getVelocity())
                .addData("Target BR", targetBR).addData("Actual BR", backRight.getVelocity());
    }
}