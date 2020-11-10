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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name="Omni TeleOp(With \"Shafeing\")", group="Controller")
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
public class omniTeleOp_shafe extends LinearOpMode {

    // Declare OpMode members.
    //Elapsed time variable
    private ElapsedTime runtime = new ElapsedTime();
    //Rear left motor
    private DcMotor leftBack = null;
    //Rear right motor
    private DcMotor rightBack = null;
    //Unused motors
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    //Loopy left for "shafeing" left
    private int loopyleft = 0;
    //Loopy right for "shafeing" right
    private int loopyright = 0;

    @Override
    public void runOpMode() {

        //Print stuff to Driver Station phone
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftBack  = hardwareMap.get(DcMotor.class, "Left Back");
        rightBack = hardwareMap.get(DcMotor.class, "Right Back");
        //Set the right back motor to reverse
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        //Wait for start
        waitForStart();
        //Reset elapsed time
        runtime.reset();

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
            //For "shafeing". Phase 1: right motor forward
            if (loopyleft > 0 && loopyleft <= 20) {
                rightBack.setPower(0.56);
            }
            //Phase 2: left motor forward
            if (loopyleft > 20 && loopyleft <= 40) {
                leftBack.setPower(-0.6);
            }
            //Phase 3: both back
            if (loopyleft > 40 && loopyleft <= 60) {
                leftBack.setPower(0.35);
                rightBack.setPower(-0.35);
            }
            //If the left button of the dpad is pressed:
            if (gamepad1.dpad_left == true) {
                //Phases of "shafeing" right set to 0
                loopyright = 0;
                //Phases of "shafeing" left add 1 each time
                loopyleft += 1;
                //If phases are greater that 60 phases set to 1
                if (loopyleft > 60) {
                    loopyleft = 1;
                }
            }
            //If the left button of the dpad isn't pressed:
            else {
                if (loopyleft >= 1) {
                    loopyleft += 1;
                }
                if (loopyleft >= 60) {
                    loopyleft = 0;
                    rightBack.setPower(0);
                    leftBack.setPower(0);
                }
            }
            if (loopyright > 0 && loopyright <= 20) {
                leftBack.setPower(-0.56);
            }
            if (loopyright > 20 && loopyright <= 40) {
                rightBack.setPower(0.6);
            }
            if (loopyright > 40 && loopyright <= 60) {
                leftBack.setPower(0.35);
                rightBack.setPower(-0.35);
            }

            if (gamepad1.dpad_right == true) {
                loopyleft = 0;
                loopyright += 1;
                if (loopyright > 60) {
                    loopyright = 1;
                }
            }
            else {
                if (loopyright >= 1) {
                    loopyright += 1;
                }
                if (loopyright >= 60) {
                    loopyright = 0;
                    rightBack.setPower(0);
                    leftBack.setPower(0);
                }
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Dpad" + gamepad1.dpad_left, "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();

        }
    }
}
