/*
 * Copyright (c) 2021 OpenFTC Team
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

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Auto Red tag")
public class Auto_Red_tag extends LinearOpMode
{
    // Telemetry
    private FtcDashboard dashboard;
    private MultipleTelemetry tel;

    // Camera
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    AprilTagDetection tagOfInterest = null;

    // FSM
    private int previous;
    private int state;
    private double timer;

    // Motors
    private Drivetrain drivetrain;
    private Servo right;
    private Servo left;
    private DcMotorEx slide;


    // Sensor
    private ColorSensor colorSensor;
    private String color;


    @Override
    public void runOpMode() {
        // Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
        telemetry.setMsTransmissionInterval(50);

        // Telemetry
        dashboard = FtcDashboard.getInstance();
        tel = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Motors
        drivetrain = new Drivetrain(
                hardwareMap.get(DcMotorEx.class, "M1"), // top left wheel
                hardwareMap.get(DcMotorEx.class, "M2"), // bottom left wheel
                hardwareMap.get(DcMotorEx.class, "M3"), // top right wheel
                hardwareMap.get(DcMotorEx.class, "M4")  // bottom right wheel
        );
        drivetrain.resetEncoders();
        drivetrain.setTolerance(5);
        /* drivetrain.setPIDF(1.26, 0.13, 0, 12.6, 10.0); *///NOT TESTED YET

        slide = hardwareMap.get(DcMotorEx.class, "slide");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setVelocityPIDFCoefficients(1.26, 0.13, 0, 12.6);
        slide.setPositionPIDFCoefficients(13.0);

        right = hardwareMap.get(Servo.class, "right");
        left = hardwareMap.get(Servo.class, "left");

        // Sensor
        colorSensor = hardwareMap.colorSensor.get("color");

        // FSM
        previous = 0;
        state = 0;

        // Init Loop
        while (!isStarted() && !isStopRequested()) {
            getTag();
        }

        //Update Telemetry for tag
        tagTelemetry();

        // Main Loop
        while (opModeIsActive()) {
            runTelemetry();
            mainFSM();
            if (state == -1) { break; }
        }
    }

    private void mainFSM() {
        switch (state) {
            case 0:
                // close clamp around pre-loaded cone
                timer = getRuntime();
                left.setPosition(0.2);
                right.setPosition(0.55);
                previous = state;
                state = -3;
                break;
            case 1:
                // slide goes up
                slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide.setTargetPosition(-3900);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(-0.7);
                // probably detect the signal sleeve color here too. like this:
                //color = getBestRecognition();
                // but since the machine learning is broken let's just set it to this for now:
                color = "Green";
                state++;
                break;
            case 2:
                // go forward
                drivetrain.runMotorDistance(0.6, -2430, -2430, 2430, 2430);
                previous = state;
                state = -2;
                break;
            case 3:
                // go right
                drivetrain.runMotorDistance(0.4, -655, 655, -655, 655);
                previous = state;
                state = -2;
                break;
            case 4:
                // go forward again
                drivetrain.runMotorDistance(0.4, -150, -150, 150, 150);
                previous = state;
                state = -2;
                break;
            case 5:
                // start timer for clamp opening (& dropping pre-loaded cone)
                timer = getRuntime();
                left.setPosition(0.39);
                right.setPosition(0.39);
                previous = state;
                state = -3;
                break;
            case 6:
                // go backward
                drivetrain.runMotorDistance(0.4, 250, 250, -250, -250);
                previous = state;
                state = -2;
                break;
            case 7:
                // turn left
                drivetrain.runMotorDistance(0.4, 970, 970, 970, 970);
                previous = state;
                state = -2;
                break;
            case 8:
                // go forward & run into wall; slide goes down a little
                drivetrain.runMotorDistance(0.4, -2000, -2000, 2000, 2000);
                slide.setTargetPosition(-500);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(0.6);
                previous = state;
                state = -2;
                break;
            case 9:
                // go backward a little
                drivetrain.runMotorDistance(0.4, 40, 40, -40, -40);
                previous = state;
                state = -2;
                break;
            case 10:
                // start timer for clamp closing
                timer = getRuntime();
                left.setPosition(0.2);
                right.setPosition(0.55);
                previous = state;
                state = -3;
                break;
            case 11:
                // go right until color sensor sees blue line; slide goes back up
                slide.setTargetPosition(-3900);
                slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slide.setPower(-0.8);
                if (colorSensor.alpha() < 400) {
                    drivetrain.runMotorPower(0, 0, 0, 0);
                    state = 12;
                }
                else {
                    drivetrain.runMotorPower(-0.2, 0.2, -0.2, 0.2);
                }
                break;
            case 12:
                // go backward
                drivetrain.runMotorDistance(0.4, 1800, 1800, -1800, -1800);
                previous = state;
                state = -2;
                break;
            case 13:
                // turn right
                drivetrain.runMotorDistance(0.4, -945, -945, -945, -945);
                previous = state;
                state = -2;
                break;
            case 14:
                // go forward
                drivetrain.runMotorDistance(0.4, -150, -150, 150, 150);
                previous = state;
                state = -2;
                break;
            case 15:
                // start timer for clamp opening (& dropping cone we just got)
                timer = getRuntime();
                left.setPosition(0.39);
                right.setPosition(0.39);
                previous = state;
                state = -3;
                break;
            case 16:
                // go backward
                drivetrain.runMotorDistance(0.4, 250, 250, -250, -250);
                previous = state;
                state = -2;
                break;
            case 17:
                // determine which signal zone to go to based on color detected
                if (tagOfInterest == null || tagOfInterest.id == LEFT) {
                    state = 18;
                }
                else if (tagOfInterest.id == MIDDLE) {
                    state = 19;
                }
                else {
                    state = 20;
                }
                break;
            case 18:
                // signal zone 1 -- go right
                drivetrain.runMotorDistance(0.4, -600, 600, -600, 600);
                previous = state;
                state = -4;
                break;
            case 19:
                // signal zone 2 -- go left
                drivetrain.runMotorDistance(0.4, 700, -700, 700, -700);
                previous = state;
                state = -4;
                break;
            case 20:
                // signal zone 3 -- go VERY left
                // (the zone goes all the way to the wall so more is better)
                drivetrain.runMotorDistance(0.4, 1500, -1500, 1500, -1500);
                previous = state;
                state = -4;
                break;
            case 21:
                left.setPosition(0.2);
                right.setPosition(0.55);
                slide.setTargetPosition(0);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(0.6);
                state = 21;
                tel.addLine("Hold");
                break;
            case 22:
                state = -1;
                break;
            case -2:
                // go to this state after doing anything motor-related so the motor can
                // do its thing before it goes to the next state
                if (drivetrain.stopMotor()) {
                    state = previous + 1;
                }
                break;
            case -3:
                // wait for 1.5 seconds while clamp operates
                if (getRuntime() >= timer + 1.5){
                    tel.addLine("waiting");
                    state = previous + 1;
                }
                break;
            case -4:
                if (drivetrain.stopMotor()) {
                    state = 21;
                }
                break;
        }
    }

    private void tagToTelemetry(AprilTagDetection detection)
    {
        tel.addLine(String.format("\nDetected tag ID=%d", detection.id));
        tel.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        tel.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        tel.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        tel.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        tel.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        tel.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    private void runTelemetry() {
        tel.addData("state", state);
        tel.addData("p1", drivetrain.getEncoderPosition("m1"));
        tel.addData("p2", drivetrain.getEncoderPosition("m2"));
        tel.addData("p3", drivetrain.getEncoderPosition("m3"));
        tel.addData("p4", drivetrain.getEncoderPosition("m4"));
        tel.addData("v1", drivetrain.getEncoderVelocity("m1"));
        tel.addData("v2", drivetrain.getEncoderVelocity("m2"));
        tel.addData("v3", drivetrain.getEncoderVelocity("m3"));
        tel.addData("v4", drivetrain.getEncoderVelocity("m4"));
        tel.addData("color alpha", colorSensor.alpha());
        tel.update();
    }

    private void getTag() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if(currentDetections.size() != 0) {
            boolean tagFound = false;

            for(AprilTagDetection tag : currentDetections) {
                if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

            if(tagFound) {
                tel.addLine("Tag of interest is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
            }
            else {
                tel.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null) {
                    tel.addLine("(The tag has never been seen)");
                }
                else {
                    tel.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }
        }
        else {
            tel.addLine("Don't see tag of interest :(");

            if(tagOfInterest == null) {
                tel.addLine("(The tag has never been seen)");
            }
            else {
                tel.addLine("\nBut we HAVE seen the tag before; last seen at:");
                tagToTelemetry(tagOfInterest);
            }
        }

        tel.update();
        sleep(20);
    }

    private void tagTelemetry() {
        if(tagOfInterest != null) {
            tel.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            tel.update();
        }
        else {
            tel.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            tel.update();
        }
    }
}