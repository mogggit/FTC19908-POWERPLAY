package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "Auto Blue")
public class Auto_Blue extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "model_20230113_102400.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";
    private static final String[] LABELS = {
            "Red",
            "Green",
            "Blue"
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
    private ColorSensor colorSensor;

    private Servo right;
    private Servo left;

    private DcMotorEx slide;

    private double timer;

    private String color;

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);
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
        drivetrain.setTolerance(5);
//        drivetrain.setPIDF(1.26, 0.13, 0, 12.6, 10.0); // NOT TESTED YET

        slide = hardwareMap.get(DcMotorEx.class, "slide");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setVelocityPIDFCoefficients(1.26, 0.13, 0, 12.6);
        slide.setPositionPIDFCoefficients(13.0);

        right = hardwareMap.get(Servo.class, "right");
        left = hardwareMap.get(Servo.class, "left");

        colorSensor = hardwareMap.colorSensor.get("color");

        previous = 0;
        state = 0;

        color = "";

        waitForStart();

        while (opModeIsActive()) {
            runTelemetry();
            mainFSM();
            if (state == -1) { break; }
        }
    }

    //color sensor alpha: less than 490 = red or blue line

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
                slide.setPower(-1);
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
                // go left
                drivetrain.runMotorDistance(0.4, 655, -655, 655, -655);
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
                drivetrain.runMotorDistance(0.4, 280, 280, -280, -280);
                previous = state;
                state = -2;
                break;
            case 7:
                // turn right
                drivetrain.runMotorDistance(0.4, -970, -970, -970, -970);
                previous = state;
                state = -2;
                break;
            case 8:
                // go forward & run into wall; slide goes down a little
                drivetrain.runMotorDistance(0.6, -2000, -2000, 2000, 2000);
                slide.setTargetPosition(-500);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1);
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
                // go left until color sensor sees blue line; slide goes back up
                slide.setTargetPosition(-3900);
                slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slide.setPower(-1);
                if (colorSensor.alpha() < 400) {
                    drivetrain.runMotorPower(0, 0, 0, 0);
                    state = 12;
                }
                else {
                    drivetrain.runMotorPower(0.2, -0.2, 0.2, -0.2);
                }
                break;
            case 12:
                // go backward
                drivetrain.runMotorDistance(0.6, 1800, 1800, -1800, -1800);
                previous = state;
                state = -2;
                break;
            case 13:
                // turn left
                drivetrain.runMotorDistance(0.4, 945, 945, 945, 945);
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
                if (color.equals("Red")) {
                    state = 18;
                }
                else if (color.equals("Green")) {
                    state = 19;
                }
                else if (color.equals("Blue")) {
                    state = 20;
                }
                else {
                    // uh do something else? maybe?
                    // i'm making the default #2 for now
                    state = 19;
                }
                break;
            case 18:
                // signal zone 1 -- go left
                drivetrain.runMotorDistance(0.4, 600, -600, 600, -600);
                previous = state;
                state = -4;
                break;
            case 19:
                // signal zone 2 -- go right
                drivetrain.runMotorDistance(0.4, -700, 700, -700, 700);
                previous = state;
                state = -4;
                break;
            case 20:
                // signal zone 3 -- go VERY right
                // (the zone goes all the way to the wall so more is better)
                drivetrain.runMotorDistance(0.4, -1500, 1500, -1500, 1500);
                previous = state;
                state = -4;
                break;
            case 21:
                left.setPosition(0.2);
                right.setPosition(0.55);
                slide.setTargetPosition(0);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1);
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
        tfodParameters.minResultConfidence = 0.5f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
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

    // get label of recognition with highest confidence level (TensorFlow Object Detection)
    private String getBestRecognition() {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        double maxConf = 0;
        String bestRecLabel = "";

        if (updatedRecognitions != null) {
            tel.addData("# Objects Detected", updatedRecognitions.size());

            // step through the list of recognitions and display image position/size information for each one
            // Note: "Image number" refers to the randomized image orientation/number
            for (Recognition recognition : updatedRecognitions) {
                double col = (recognition.getLeft() + recognition.getRight()) / 2;
                double row = (recognition.getTop() + recognition.getBottom()) / 2;
                double width = Math.abs(recognition.getRight() - recognition.getLeft());
                double height = Math.abs(recognition.getTop() - recognition.getBottom());
                double confidence = recognition.getConfidence() * 100;
                String recLabel = recognition.getLabel();

                tel.addData("", " ");
                tel.addData("Image", "%s (%.0f%% Conf.)", recLabel, confidence);
                tel.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                tel.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);

                if (confidence >= maxConf) {
                    maxConf = confidence;
                    bestRecLabel = recLabel;
                }
            }
            tel.update();
        }
        return bestRecLabel;
    }
}
