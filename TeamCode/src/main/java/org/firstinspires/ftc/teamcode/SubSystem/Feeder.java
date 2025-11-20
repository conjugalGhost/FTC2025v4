package org.firstinspires.ftc.teamcode.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Feeder {
    private DcMotorEx feederLeft;
    private DcMotorEx feederRight;

    public Feeder(HardwareMap hardwareMap) {
        try {
            feederLeft = hardwareMap.get(DcMotorEx.class, "feederLeft");
        } catch (Exception e) {
            feederLeft = null;
        }

        try {
            feederRight = hardwareMap.get(DcMotorEx.class, "feederRight");
        } catch (Exception e) {
            feederRight = null;
        }
    }

    // Spins outward from center: left forward, right reverse
    public void feedForward() {
        if (feederLeft != null) feederLeft.setPower(1.0);
        if (feederRight != null) feederRight.setPower(-1.0);
    }

    // Spins inward toward center: left reverse, right forward
    public void feedReverse() {
        if (feederLeft != null) feederLeft.setPower(-1.0);
        if (feederRight != null) feederRight.setPower(1.0);
    }

    public void stop() {
        if (feederLeft != null) feederLeft.setPower(0.0);
        if (feederRight != null) feederRight.setPower(0.0);
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("=== FEEDER ===");
        if (feederLeft != null) {
            telemetry.addData("Left Power", feederLeft.getPower());
            telemetry.addData("Left Position", feederLeft.getCurrentPosition());
        } else {
            telemetry.addData("Left Motor", "Not Found");
        }
        if (feederRight != null) {
            telemetry.addData("Right Power", feederRight.getPower());
            telemetry.addData("Right Position", feederRight.getCurrentPosition());
        } else {
            telemetry.addData("Right Motor", "Not Found");
        }
    }
}