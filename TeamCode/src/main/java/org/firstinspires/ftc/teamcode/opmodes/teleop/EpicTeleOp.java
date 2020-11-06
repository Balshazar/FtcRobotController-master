package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Basic Iterative Op", group = "Iterative Op")
@Disabled
public class EpicTeleOp extends OpMode {

    DcMotor lf;
    DcMotor lr;
    DcMotor rf;
    DcMotor rr;

    @Override
    public void init() {
        lf = hardwareMap.dcMotor.get("left front");
        lr = hardwareMap.dcMotor.get("left back");
        rf = hardwareMap.dcMotor.get("right front");
        rr = hardwareMap.dcMotor.get("right back");

        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = gamepad1.left_stick_y;
        double rightStickX = gamepad1.right_stick_x;
        double wheelPower;
        double stickAngleRadians;
        double rightX;
        double lfPower;
        double rfPower;
        double lrPower;
        double rrPower;

        wheelPower = Math.hypot(leftStickX, leftStickY);
        stickAngleRadians = Math.atan2(leftStickY, leftStickX);

        stickAngleRadians = stickAngleRadians - Math.PI / 4; //adjust by 45 degrees

        double sinAngleRadians = Math.sin(stickAngleRadians);
        double cosAngleRadians = Math.cos(stickAngleRadians);
        double factor = 1 / Math.max(Math.abs(sinAngleRadians), Math.abs(cosAngleRadians));

        rightX = rightStickX * .5;

        lfPower = wheelPower * cosAngleRadians * factor + rightX;
        rfPower = wheelPower * sinAngleRadians * factor - rightX;
        lrPower = wheelPower * sinAngleRadians * factor + rightX;
        rrPower = wheelPower * cosAngleRadians * factor - rightX;

        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lr.setPower(lrPower);
        rr.setPower(rrPower);

    }
}
