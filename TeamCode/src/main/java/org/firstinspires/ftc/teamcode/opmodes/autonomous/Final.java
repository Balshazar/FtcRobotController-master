/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "Final_Autonomous", group = "Camera")
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
public class Final extends LinearOpMode
{
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
    private int square = 0;
    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationPipeline pipeline;
    public void goToSquare(int field_square) {
        if (field_square == 1) {
            move(-700, 700);
            move(700, 700);
            move(800, -800);
            move(1000, 1000);
        }
        if (field_square == 2) {
            move(200, 300);
            move(500, 500);
            move(300, 200);
            move(700, 700);

        }
        if (field_square == 3) {
            move(200, 300);
            move(500, 500);
            move(300, 200);
            move(700, 700);

        }
    }
    public void move(int rotations_left, int rotations_right) {

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotations_left = rotations_left * -1;
        rotations_right = rotations_right * -1;
        if (rotations_left <= 0) {
            leftBack.setPower(-0.25);
        }
        else {
            leftBack.setPower(0.25);
        }

        if (rotations_right <= 0) {
            rightBack.setPower(0.25);
        }
        else {
            rightBack.setPower(-0.25);
        }



        while (true) {
            telemetry.addData("Left", leftBack.getCurrentPosition());
            telemetry.addData("Right", rightBack.getCurrentPosition());
            telemetry.update();
            if (rotations_left <= 0) {
                if (leftBack.getCurrentPosition() <= rotations_left) {
                    leftBack.setPower(0);
                }

            }
            else {
                if (leftBack.getCurrentPosition() >= rotations_left) {
                    leftBack.setPower(0);
                }

            }

            if (rotations_right <= 0) {
                if (rightBack.getCurrentPosition() <= rotations_right) {
                    rightBack.setPower(0);
                }

            }
            else {
                if (rightBack.getCurrentPosition() >= rotations_right) {
                    rightBack.setPower(0);
                }
            }


            if (leftBack.getPower() == 0.0 && rightBack.getPower() == 0.0) {
                break;
            }
        }

    }


    @Override
    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });
        //Print stuff to Driver Station phone
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftBack  = hardwareMap.get(DcMotor.class, "Left Back");
        rightBack = hardwareMap.get(DcMotor.class, "Right Back");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //Set the right back motor to reverse









        //negative
        //left_intake  = hardwareMap.get(DcMotor.class, "Left Intake");
        right_intake = hardwareMap.get(DcMotor.class, "Right Intake");
        //Right motor reversed
        right_intake.setDirection(DcMotor.Direction.REVERSE);
        servoone = hardwareMap.servo.get("servoone");
        servotwo = hardwareMap.servo.get("servotwo");


        //Wait for start
        waitForStart();
        runtime.reset();





        move(1500, 15);
        sleep(1000000);
        //get ring height and but that in square
        /*telemetry.addData("Position", pipeline.position);
        telemetry.addData("num", SkystoneDeterminationPipeline.avg1);
        if (pipeline.position.equals("None")) {
            square = 1;
        }

        if (pipeline.position.equals("One")) {
            square = 2;
        }
        if (pipeline.position.equals("Four")) {
            square = 3;
        } */
        telemetry.addData("Status", square);
        telemetry.update();
        goToSquare(1);
        telemetry.addData("Status", "Done");
        telemetry.update();

        //Reset elapsed times
        sleep(10000);

        //move(120);





        //telemetry.addData("Position", pipeline.position);

        //telemetry.addData("num", SkystoneDeterminationPipeline.avg1);





        //telemetry.update();
        //sleep(50);




    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181,98);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 170;
        final int ONE_RING_THRESHOLD = 150;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        static int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis

            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }


    }
}
