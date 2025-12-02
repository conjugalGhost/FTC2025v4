package org.firstinspires.ftc.teamcode.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Feeder {
    private DcMotorEx feederLeft;
    private DcMotorEx feederRight;

    // Flip this to true/false until forward feed matches your physical forward.
    // TRUE  => Forward feed uses Left:+, Right:- (left forward, right reverse)
    // FALSE => Forward feed uses Left:-, Right:+ (left reverse, right forward)
    private static final boolean FORWARD_IS_LEFT_PLUS_RIGHT_MINUS = false;

    private static final int STEP_TICKS = 144; // ~180°
    private int state = 0; // 1 = forward, -1 = reverse, 0 = stopped

    public Feeder(HardwareMap hardwareMap) {
        try {
            feederLeft = hardwareMap.get(DcMotorEx.class, "feederLeft");
            feederLeft.setDirection(DcMotorEx.Direction.FORWARD);
            feederLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            feederLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            feederLeft = null;
        }

        try {
            feederRight = hardwareMap.get(DcMotorEx.class, "feederRight");
            feederRight.setDirection(DcMotorEx.Direction.FORWARD);
            feederRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            feederRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            feederRight = null;
        }
    }

    public void resetEncoders() {
        if (feederLeft != null) {
            feederLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            feederLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        if (feederRight != null) {
            feederRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            feederRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        state = 0;
    }

    // Forward feed
    public void advanceOneStep() {
        int startLeft  = feederLeft  != null ? feederLeft.getCurrentPosition()  : 0;
        int startRight = feederRight != null ? feederRight.getCurrentPosition() : 0;

        double leftPower  = FORWARD_IS_LEFT_PLUS_RIGHT_MINUS ?  1.0 : -1.0;
        double rightPower = FORWARD_IS_LEFT_PLUS_RIGHT_MINUS ? -1.0 :  1.0;

        if (feederLeft  != null) feederLeft.setPower(leftPower);
        if (feederRight != null) feederRight.setPower(rightPower);

        ElapsedTime timer = new ElapsedTime();
        while ((feederLeft  != null && Math.abs(feederLeft.getCurrentPosition()  - startLeft)  < STEP_TICKS) ||
                (feederRight != null && Math.abs(feederRight.getCurrentPosition() - startRight) < STEP_TICKS)) {
            if (timer.seconds() > 1.0) break;
        }

        stop();
        state = 1;
    }

    // Reverse feed
    public void reverseOneStep() {
        int startLeft  = feederLeft  != null ? feederLeft.getCurrentPosition()  : 0;
        int startRight = feederRight != null ? feederRight.getCurrentPosition() : 0;

        double leftPower  = FORWARD_IS_LEFT_PLUS_RIGHT_MINUS ? -1.0 :  1.0;
        double rightPower = FORWARD_IS_LEFT_PLUS_RIGHT_MINUS ?  1.0 : -1.0;

        if (feederLeft  != null) feederLeft.setPower(leftPower);
        if (feederRight != null) feederRight.setPower(rightPower);

        ElapsedTime timer = new ElapsedTime();
        while ((feederLeft  != null && Math.abs(feederLeft.getCurrentPosition()  - startLeft)  < STEP_TICKS) ||
                (feederRight != null && Math.abs(feederRight.getCurrentPosition() - startRight) < STEP_TICKS)) {
            if (timer.seconds() > 1.0) break;
        }

        stop();
        state = -1;
    }

    public void stop() {
        if (feederLeft  != null) feederLeft.setPower(0.0);
        if (feederRight != null) feederRight.setPower(0.0);
        state = 0;
    }

    public int getState() { return state; }

    public int getLeftPosition() {
        return feederLeft != null ? feederLeft.getCurrentPosition() : 0;
    }

    public int getRightPosition() {
        return feederRight != null ? feederRight.getCurrentPosition() : 0;
    }

    public int getAveragePosition() {
        int left = feederLeft != null ? feederLeft.getCurrentPosition() : 0;
        int right = feederRight != null ? feederRight.getCurrentPosition() : 0;
        return (left + right) / 2;
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("=== FEEDER ===");
        telemetry.addData("ForwardMap", FORWARD_IS_LEFT_PLUS_RIGHT_MINUS ? "L:+ R:-" : "L:- R:+");
        telemetry.addData("State", state == 1 ? "Forward" : state == -1 ? "Reverse" : "Stopped");
        if (feederLeft  != null) telemetry.addData("Left Pos (ticks)", feederLeft.getCurrentPosition());
        else                     telemetry.addData("Left Motor", "Not Found");
        if (feederRight != null) telemetry.addData("Right Pos (ticks)", feederRight.getCurrentPosition());
        else                     telemetry.addData("Right Motor", "Not Found");
    }
}