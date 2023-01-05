package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "Auto Tests")
public class Auto_Tests extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";
    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };
    private static final String VUFORIA_KEY =
            "AW7MS5P/////AAABmbhCSjNBJkfFs+kp+0SOiHFqZSTkYVDULdxP11ncxw4EQzSyRq4EOiB4GBhwHNTrpMnzpmW6xnjHx4W9Z+wQrT7fMevji9eaAX/Zn+LQwm3VrXcZLz1qmqswkdRrEgea+8tLIfLGqlnPLTHyvFcQwI21X2nM9DPIOPgFX1H+mrJetXYSe5DcM6B1kkLMSP/Y4j6dtX4FADWxblGiTrryqV0D5r7B1OMTPMydiqbta46QVSm8CrDhP88TGZ6bvnPtlPli8PTev/CWl7qihRyh8U3I6J4CifMfNOF/fMfSIsho91WhZi3T6OB0ulsHtTxQrrVPte5SIBm7Vtstx05z4KcUnSZ4rybico/ME9juFkMO";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    // combines dashboard telemetry and ds telemetry
    private FtcDashboard dashboard;
    private MultipleTelemetry tel;
    private int previous;
    private int state;
    private Drivetrain drivetrain;
    private Servo right;
    private Servo left;
    private double timer;

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 16.0/9.0);
        }

        dashboard = FtcDashboard.getInstance();
        tel = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.startCameraStream(tfod, 30);

        drivetrain = new Drivetrain(
                hardwareMap.get(DcMotorEx.class, "M1"), // top left wheel
                hardwareMap.get(DcMotorEx.class, "M2"), // bottom left wheel
                hardwareMap.get(DcMotorEx.class, "M3"), // top right wheel
                hardwareMap.get(DcMotorEx.class, "M4")  // bottom right wheel
        );
        drivetrain.resetEncoders();
        drivetrain.setTolerance(10);
//        drivetrain.setPIDF(1.26, 0.13, 0, 12.6, 7.0);

        right = hardwareMap.get(Servo.class, "right");
        left = hardwareMap.get(Servo.class, "left");

        previous = 0;
        state = 0;

        waitForStart();

        while (opModeIsActive()) {
            runTelemetry();
            mainFSM();
            if (state == -1) { break; }
        }
    }

    private void mainFSM() {
        switch (state) {
            case 0:
                left.setPosition(0.39);
                right.setPosition(0.39);

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        tel.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2;
                            double row = (recognition.getTop() + recognition.getBottom()) / 2;
                            double width = Math.abs(recognition.getRight() - recognition.getLeft());
                            double height = Math.abs(recognition.getTop() - recognition.getBottom());

                            tel.addData("", " ");
                            tel.addData("Image", "%s (%.0f%% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                            tel.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                            tel.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);

                            // detect what type of image & decide what to do depending on this
                        }
                        tel.update();
                    }
                }
                state++;
                break;
            case 1:
                drivetrain.runMotorDistance(0.4, -2435, -2435, 2435, 2435);
                previous = state;
                state = -2;
                break;
            case 2:
                drivetrain.runMotorDistance(0.4, 600, -600, 600, -600);
                previous = state;
                state = -2;
                break;
            case 3:
                drivetrain.runMotorDistance(0.4, 110, 110, -110, -110);
                previous = state;
                state = -2;
                break;
            case 4:
                drivetrain.runMotorDistance(0.4, -970, -970, -970, -970);
                previous = state;
                state = -2;
                break;
            case 5:
                drivetrain.runMotorDistance(0.4, -1730, -1730, 1730, 1730);
                previous = state;
                state = -2;
                break;
            case 6:
                timer = getRuntime();
                left.setPosition(0.2);
                right.setPosition(0.55);
                state++;
                break;
            case 7:
                if (getRuntime() >= timer + 2){
                    tel.addLine("waiting");
                    previous = state;
                    state++;
                }
                break;
            case 8:
                drivetrain.runMotorDistance(0.4, 1750, 1750, -1750, -1750);
                previous = state;
                state = -2;
                break;
            case 9:
                drivetrain.runMotorDistance(0.4, 945, 945, 945, 945);
                previous = state;
                state = -2;
                break;
            case 10:
                state = -1;
                break;
            case -2:
                if (drivetrain.stopMotor()) {
                    state = previous + 1;
                }
                break;
        }
    }

    // Initialize the Vuforia localization engine.
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    // Initialize the TensorFlow Object Detection engine.
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
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
        tel.update();
    }
}
