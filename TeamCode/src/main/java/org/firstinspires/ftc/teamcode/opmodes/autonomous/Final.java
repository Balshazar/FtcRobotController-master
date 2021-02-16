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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import java.util.Locale;


@Autonomous(name = "Final_Autonomous", group = "Camera")
/*
_________________________________
|   Name   | Port | Name on Hub |
|----------|------|-------------|
|Left Back |  1   | "left back" |
|----------|------|-------------|
|Right Back|  0   | "right back"|
|_______________________________|

  _____     _____                                      _______      ______
 /____ /]  /____/                       _____         /______/\    /_____/
 [    ] ] /    /          __           /____/ [        \     \ \  /     /             __                          ______
 [    ] ]/    /          /\ \          [    [ [         \     \ \/     /             /\ \                        /       \
 [    ] /    /          /  \ \         [    [ [          \     \/     /             /  \ \                       |       |
 [          /\         /  _ \ \        [    [ [           \          /             /  _ \ \                       \     /
 [    ]\    \ \       /  /_\ \ \       [    [ [____        \        /[            /  /_\ \ \                       \   /
 [    ] \    \ \     /  ____  \ \      [    [/____/[        [      [ [           /  ____  \ \                       \_/
 [    ]] \    \ \   /  / /  \  \ \     [         [ /        [      [ /          /  / /  \  \ \                       _
 [____]/  \____\/  /__/_/    \__\_\    [_________[/         [______[/          /__/_/    \__\_\   is the very cool  (_)

 */
public class Final extends LinearOpMode {
    // Declare OpMode members.
    //Elapsed time variable
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            "AUkG2oP/////AAABmQGYVdglZUJLn8w+eHLCAYVO1mp8Lu7z9SojA4Fm/w61UizkAnX3WrdTfIfPUpWQq6XGPhNVSMx5ey7H3j5/cYdga6WcJ4Gcx3Y12f7FylQzNwROkZQf1OdqIw9dQl5fDYUJjg7EWr3I5rPa1a56KrLYAHWORm/bWmOUmUf/jPuC11IJ24W+ZXCamgPEvx5xqSWdQuhLLIqHzVSUVOugt8O4uPwpheueN5HqY5f+CgkAqtdnOPUJT2Os1rRIlXaBN8LP0s1lTGZPL1BGBa19YYTT3rySTMi1ExTF62MX2qI106uD3S0eAK8R/er5ohD4pW/QAOaa65p37nboeP8i6WAImA6L/epx0UdhYw2UXbed";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    private ElapsedTime runtime = new ElapsedTime();
    //Rear left motor
    private DcMotor leftBack = null;
    //Rear right motor
    private DcMotor rightBack = null;
    //Intake Left
    //private DcMotor left_intake = null;
    //Intake Right

    //Unused motors
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    private Servo servoone = null;
    private Servo servotwo = null;
    private int intake_num = 0;
    private int square = 0;
    private int counter = 0;

    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;


    public void turn(int angleTo, double left, double right) {
        left = left * -1;


        leftBack.setPower(left);
        rightBack.setPower(right);
        while (true) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Angle", (int) angles.firstAngle);
            telemetry.update();
            if ((int) angles.firstAngle == angleTo) {
                leftBack.setPower(0);
                rightBack.setPower(0);
                break;
            }


                /*telemetry.addData("Angle", (int) angles.firstAngle);
                telemetry.update();*/


        }
    }



    //Notes about angles:
        /*
        1st square:
        -46
        1500
        3

        2nd square:
        -13

        3rd square:
        -25
        -132
        159
        0

         */
    public void goToSquare(int field_square) {
        if (field_square == 1) {
            turn(-48, 0.30, -0.30);
            move(2000, 2000);
            move(-800, -800);
            turn(3, -0.30, 0.30);
            move(200, 200);
            //end
        }
        if (field_square == 2) {
            turn(-13, 0.30, -0.30);
            move(2400, 2400);
            move(-1300, -1300);
            //end

        }
        if (field_square == 3) {
            turn(-22, 0.30, -0.30);
            move(3700, 3700);
            move(-2300, -2300);
            //end
            /*turn(-132, 0.30, -0.30);
            move(1300, 1300);
            turn(160, 0.30, -0.30);
            move(800, 800);
            turn(0, 0.30, -0.30);
            move(5000, 5000);*/

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
            leftBack.setPower(-0.75);
        } else {
            leftBack.setPower(0.75);
        }

        if (rotations_right <= 0) {
            rightBack.setPower(0.75);
        } else {
            rightBack.setPower(-0.75);
        }


        while (true) {
            telemetry.addData("Left", leftBack.getCurrentPosition());
            telemetry.addData("Right", rightBack.getCurrentPosition());
            telemetry.update();
            if (rotations_left <= 0) {
                if (leftBack.getCurrentPosition() <= rotations_left) {
                    leftBack.setPower(0);
                }

            } else {
                if (leftBack.getCurrentPosition() >= rotations_left) {
                    leftBack.setPower(0);
                }

            }

            if (rotations_right <= 0) {
                if (rightBack.getCurrentPosition() <= rotations_right) {
                    rightBack.setPower(0);
                }

            } else {
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
    public void runOpMode() {
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
        }
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);





        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftBack = hardwareMap.get(DcMotor.class, "Left Back");
        rightBack = hardwareMap.get(DcMotor.class, "Right Back");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servotwo = hardwareMap.servo.get("servotwo");
        //Set the right back motor to reverse


        //negative
        //left_intake  = hardwareMap.get(DcMotor.class, "Left Intake");

        servoone = hardwareMap.servo.get("servoone");
        servotwo = hardwareMap.servo.get("servotwo");


        //Wait for start
        servotwo.setPosition(0.90);
        waitForStart();
        runtime.reset();


        move(700, 700);
        turn(-6, 0.30, -0.30);


        //get ring height and but that in square

        while (counter <= 20) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {

                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    counter += 1;

                    if ((int) updatedRecognitions.size() == 0) {

                        square = 1;

                    }


                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        if ((int) recognition.getTop() < 80) {
                            square = 3;
                        }
                        if ((int) recognition.getTop() >= 80) {
                            square = 2;
                        }

                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());

                    }
                    telemetry.addData("Counter", counter);
                    telemetry.update();
                }
            }
        }
        //set square
        telemetry.addData("Status", square);
        telemetry.update();
        sleep(1000);

        turn(31, -0.30, 0.30);
        move(700, 700);

        telemetry.addData("Status", square);
        telemetry.update();

        goToSquare(square);


        telemetry.addData("Status", "Done");
        telemetry.update();





        if (tfod != null) {
            tfod.shutdown();
        }
    }


    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }


}
