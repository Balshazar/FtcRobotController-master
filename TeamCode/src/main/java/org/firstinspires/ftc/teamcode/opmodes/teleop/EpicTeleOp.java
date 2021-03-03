package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Basic Iterative Op", group = "Iterative Op")

public class EpicTeleOp extends OpMode {
    private float up_down = 20;
    private int closed_int = 0;
    private boolean closed = true;
    private int arm_boolean = 3;
    DcMotor lf;
    DcMotor lb;
    DcMotor rf;
    DcMotor rb;

    DcMotor arm_motor;
    private Servo servoone = null;

    public void move(int arm_rotations) {

        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm_rotations = arm_rotations * -1;

        if (arm_rotations <= 0) {
            arm_motor.setPower(-0.80);
        } else {
            arm_motor.setPower(0.80);
        }




        while (true) {
            telemetry.addData("Arm Rotations", arm_rotations);
            telemetry.addData("arm_motor", arm_motor.getCurrentPosition());
            telemetry.update();
            arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (arm_rotations >= 0) {
                if (arm_motor.getCurrentPosition() >= arm_rotations) {
                    arm_motor.setPower(0);

                }

            } else {
                if (arm_motor.getCurrentPosition() <= arm_rotations) {
                    arm_motor.setPower(0);

                }

            }




            if (arm_motor.getPower() == 0.0) {
                break;
            }
        }

    }
    @Override
    public void init() {
        lf = hardwareMap.dcMotor.get("Left Front");
        lb = hardwareMap.dcMotor.get("Left Back");
        rf = hardwareMap.dcMotor.get("Right Front");
        rb = hardwareMap.dcMotor.get("Right Back");

        arm_motor = hardwareMap.dcMotor.get("Arm Motor");
        servoone = hardwareMap.servo.get("servoone");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        double leftStickX = -gamepad1.left_stick_x;
        double leftStickY = gamepad1.left_stick_y;
        double rightStickX = -gamepad1.right_stick_x;
        double wheelPower;
        double stickAngleRadians;
        double rightX;
        double lfPower;
        double rfPower;
        double lbPower;
        double rbPower;

        wheelPower = Math.hypot(leftStickX, leftStickY);
        stickAngleRadians = Math.atan2(leftStickY, leftStickX);

        stickAngleRadians = stickAngleRadians - Math.PI / 4; //adjust by 45 degrees

        double sinAngleRadians = Math.sin(stickAngleRadians);
        double cosAngleRadians = Math.cos(stickAngleRadians);
        double factor = 1 / Math.max(Math.abs(sinAngleRadians), Math.abs(cosAngleRadians));

        rightX = rightStickX * .5;

        lfPower = wheelPower * cosAngleRadians * factor + rightX;
        rfPower = wheelPower * sinAngleRadians * factor - rightX;
        lbPower = wheelPower * sinAngleRadians * factor + rightX;
        rbPower = wheelPower * cosAngleRadians * factor - rightX;

        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lb.setPower(lbPower);
        rb.setPower(rbPower);


        if (gamepad1.dpad_up && arm_boolean == 3) {
            move(150);
            arm_boolean = 0;


        }
        if (gamepad1.dpad_down && arm_boolean == 2) {
            move(-100);
            arm_boolean = 1;

        }
        if (arm_boolean == 0 && gamepad1.dpad_up == false) {
            arm_boolean = 2;

        }
        if (arm_boolean == 1 && gamepad1.dpad_down == false) {
            arm_boolean = 3;

        }

        if (arm_boolean == 0 || arm_boolean == 2) {
            if (arm_motor.getCurrentPosition() - 5 >= -145 && arm_motor.getCurrentPosition() + 5 <= -155) {
                move((-150 - arm_motor.getCurrentPosition()) * -1);
            }
        }

        if (gamepad1.left_trigger > 0.0 && closed == false && closed_int == 0) {
            closed_int = 1;
            closed = true;
            servoone.setPosition(-0.25);



        }
        else if (gamepad1.left_trigger > 0.0 && closed == true && closed_int == 0) {
            closed_int = 1;
            closed = false;
            servoone.setPosition(0.25);



        }

        if (gamepad1.left_trigger == 0.0 && closed_int == 1) {
            closed_int = 0;
        }

    }
}
