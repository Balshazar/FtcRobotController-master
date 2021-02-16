/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name="Omni TeleOp", group="Controller")
/*
_________________________________
|   Name   | Port | Name on Hub |
|----------|------|-------------|
|Left Back |  1   | "left back" |
|----------|------|-------------|
|Right Back|  3   | "right back"|
|_______________________________|

  _____     _____                                      _______      ______
 /____ /]  /____/                       _____         /______/\    /_____/
 [    ] ] /    /          __           /____/ [        \     \ \  /     /             __                     ______
 [    ] ]/    /          /\ \          [    [ [         \     \ \/     /             /\ \                   /       \
 [    ] /    /          /  \ \         [    [ [          \     \/     /             /  \ \                  |       |
 [          /\         /  _ \ \        [    [ [           \          /             /  _ \ \                  \     /
 [    ]\    \ \       /  /_\ \ \       [    [ [____        \        /[            /  /_\ \ \                  \   /
 [    ] \    \ \     /  ____  \ \      [    [/____/[        [      [ [           /  ____  \ \                  \_/
 [    ]] \    \ \   /  / /  \  \ \     [         [ /        [      [ /          /  / /  \  \ \                  _
 [____]/  \____\/  /__/_/    \__\_\    [_________[/         [______[/          /__/_/    \__\_\   is the best  (_)

 */
public class masterTeleOp extends LinearOpMode {

    // Declare OpMode members.
    //Elapsed time variable
    private ElapsedTime runtime = new ElapsedTime();
    //Rear left motor
    private DcMotor leftBack = null;
    //Rear right motor
    private DcMotor rightBack = null;
    //Intake Left
    //private DcMotor left_intake = null;
    //Intake Right
    private DcMotor arm_motor = null;
    //Unused motors
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    private Servo servoone = null;
    private Servo servotwo = null;
    private int intake_num = 0;
    private float up_down = 20;
    private int closed_int = 0;
    private boolean closed = true;
    private int arm_boolean = 3;
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
    public void runOpMode() {

        //Print stuff to Driver Station phone
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftBack  = hardwareMap.get(DcMotor.class, "Left Back");
        rightBack = hardwareMap.get(DcMotor.class, "Right Back");
        //Set the right back motor to reverse
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        //left_intake  = hardwareMap.get(DcMotor.class, "Left Intake");
        arm_motor = hardwareMap.get(DcMotor.class, "Arm Motor");
        //Right motor reversed

        //left_intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm_motor.setDirection(DcMotor.Direction.REVERSE);

        servoone = hardwareMap.servo.get("servoone");
        servotwo = hardwareMap.servo.get("servotwo");


        //Wait for start
        waitForStart();
        //Reset elapsed time
        runtime.reset();
        servoone.setPosition(0.5);

        while (opModeIsActive()) {

            //power to rear back motor
            double leftPower;
            //Power to rear back motor
            double rightPower;
            //Forward force
            double drive = -gamepad1.right_stick_x;
            //Turning Froce

            double turn  =  gamepad1.left_stick_y;


            //Convert turning and forward force into one that is then set to left power and right power
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            leftBack.setPower(leftPower);
            rightBack.setPower(rightPower);



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
}
