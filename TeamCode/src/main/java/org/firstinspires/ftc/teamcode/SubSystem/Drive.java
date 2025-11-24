package org.firstinspires.ftc.teamcode.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Drive {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private static final double TICKS_PER_INCH = 7.6; // calculated value, fine-tune with calibration

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
    }

    /** TeleOp drive with gamepad sticks (robot-centric mecanum) */
    public void driveWithGamepad(com.qualcomm.robotcore.hardware.Gamepad gamepad) {
        double y  = -gamepad.left_stick_y;  // forward/back
        double x  = gamepad.left_stick_x;   // strafe
        double rx = gamepad.right_stick_x;  // rotation

        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(bl),
                Math.max(Math.abs(fr), Math.abs(br))));
        if (max > 1.0) {
            fl /= max; bl /= max; fr /= max; br /= max;
        }

        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }

    /** Simple power set for all motors */
    public void setDrivePower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    /** Encoder-based forward drive with telemetry */
    public void driveForwardInches(double inches, double power, LinearOpMode opMode) {
        int ticks = (int)(inches * TICKS_PER_INCH);

        // Reset encoders before each move
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

        while (opMode.opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
            opMode.telemetry.addData("FL pos", frontLeft.getCurrentPosition());
            opMode.telemetry.addData("FR pos", frontRight.getCurrentPosition());
            opMode.telemetry.addData("BL pos", backLeft.getCurrentPosition());
            opMode.telemetry.addData("BR pos", backRight.getCurrentPosition());
            opMode.telemetry.update();
        }

        stop();
    }

    /** Drive forward with IMU heading correction */
    public void driveStraightWithHeading(double inches, double power, double targetHeading,
                                         LinearOpMode opMode, IMU imu) {
        int ticks = (int)(inches * TICKS_PER_INCH);

        // Reset encoders
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

        while (opMode.opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
            double currentHeading = imu.getHeading();
            double error = targetHeading - currentHeading;
            double correction = error * 0.02; // proportional gain

            frontLeft.setPower(power + correction);
            backLeft.setPower(power + correction);
            frontRight.setPower(power - correction);
            backRight.setPower(power - correction);

            opMode.telemetry.addData("Heading", currentHeading);
            opMode.telemetry.addData("Error", error);
            opMode.telemetry.addData("FL pos", frontLeft.getCurrentPosition());
            opMode.telemetry.addData("FR pos", frontRight.getCurrentPosition());
            opMode.telemetry.update();
        }

        stop();
    }

    /** Simple in-place turn */
    public void turn(double power) {
        double p = Math.max(-1.0, Math.min(1.0, power));
        frontLeft.setPower(p);
        backLeft.setPower(p);
        frontRight.setPower(-p);
        backRight.setPower(-p);
    }

    /** Stop all motors */
    public void stop() {
        setDrivePower(0);
    }
}