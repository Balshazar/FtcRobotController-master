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

package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Basic", group="Controller")
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
public class BasicOpMode_Linear extends LinearOpMode {

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
    private DcMotor right_intake = null;
    //Unused motors
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    private Servo servoone = null;
    private Servo servotwo = null;
    private int intake_num = 0;

    public void move(int secs) {
        rightBack.setPower(0.70);
        leftBack.setPower(-0.70);
        sleep(secs);
        rightBack.setPower(0);
        leftBack.setPower(0);
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
        right_intake = hardwareMap.get(DcMotor.class, "Right Intake");
        //Right motor reversed
        right_intake.setDirection(DcMotor.Direction.REVERSE);
        servoone = hardwareMap.servo.get("servoone");
        servotwo = hardwareMap.servo.get("servotwo");


        //Wait for start
        waitForStart();
        //Reset elapsed time
        runtime.reset();
        move(120);




    }
}
