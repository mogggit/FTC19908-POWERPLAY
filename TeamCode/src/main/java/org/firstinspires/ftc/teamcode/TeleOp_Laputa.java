package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "TeleOp_Laputa") // name of this program shown in ds
public class TeleOp_Laputa extends LinearOpMode {

    private Drivetrain drivetrain;
    private Servo right;
    private Servo left;
    private DcMotorEx slide;
    private double leftPos;
    private double rightPos;
    private boolean clampToggle;
    private int slidePos;
    private boolean slideToggle;
    // combines dashboard telemetry and ds telemetry
    private MultipleTelemetry tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    private TouchSensor slideButton;

    @Override
    public void runOpMode() {

        drivetrain = new Drivetrain(
                hardwareMap.get(DcMotorEx.class, "M1"), // top left wheel
                hardwareMap.get(DcMotorEx.class, "M2"), // bottom left wheel
                hardwareMap.get(DcMotorEx.class, "M3"), // top right wheel
                hardwareMap.get(DcMotorEx.class, "M4")  // bottom right wheel
        );

        Gamepad curGamepad1 = new Gamepad();
        Gamepad curGamepad2 = new Gamepad();
        Gamepad prevGamepad1 = new Gamepad();
        Gamepad prevGamepad2 = new Gamepad();

        right = hardwareMap.get(Servo.class, "right");
        left = hardwareMap.get(Servo.class, "left");

        slide = hardwareMap.get(DcMotorEx.class, "slide");
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setVelocityPIDFCoefficients(1.26, 0.13, 0, 12.6);
        slide.setPositionPIDFCoefficients(5.0);

        slideButton = hardwareMap.touchSensor.get("slidebutton"); // touch sensor for slide
        // value for the button is in reverse (pressed is released; released in pressed)

        drivetrain.driveMode();

        //right: increase = close
        //left: decrease = close
        rightPos = 0.39;
        leftPos = 0.39;
        clampToggle = false;

        // slide: -4354 at max
        // slidePos = 0; idk yet test later
        // slideToggle = false;

        waitForStart();

        while (opModeIsActive()) {

            drivetrain.drive(gamepad1, 1);

// -------- single press to clamp ------------------------------------------------------------------
            prevGamepad1.copy(curGamepad1);
            prevGamepad2.copy(curGamepad2);
            curGamepad1.copy(gamepad1);
            curGamepad2.copy(gamepad2);

            if ((curGamepad2.right_bumper && !prevGamepad2.right_bumper) || (curGamepad1.right_bumper && !prevGamepad1.right_bumper)) {
                clampToggle = true;
            }
            else if ((curGamepad2.left_bumper && !prevGamepad2.left_bumper) || (curGamepad1.left_bumper && !prevGamepad1.left_bumper)) {
                clampToggle = false;
            }

            if (clampToggle) {
                rightPos = 0.55;
                leftPos = 0.2;
            }
            else {
                rightPos = 0.39;
                leftPos = 0.39;
            }

            left.setPosition(leftPos);
            right.setPosition(rightPos);

// -------- hold to lift slide ---------------------------------------------------------------------
            if (gamepad2.y || gamepad1.y) {
                slide.setTargetPosition(-4200);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(-1);
            }
            else if (gamepad2.x || gamepad1.x) {
                slide.setTargetPosition(-3000);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(-1);
            }
            else if (gamepad2.b || gamepad1.b) {
                slide.setTargetPosition(-1800);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(-1);
            }

            else if (gamepad2.a || gamepad1.b) {
                slide.setTargetPosition(-300);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(-1);
            }
            else if (!slideButton.isPressed()) {
                slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide.setPower(0);
            }
            else {
                slide.setTargetPosition(0);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(0.65);
            }

//            if (slideToggle) {
//                slidePos = -4300;
//                tel.addData("y pressed", "(to make slide go up)");
//            }
//            else {
//                slidePos = 0;
//                tel.addData("y pressed", "(to make slide return down)");
//            }

// -------- telemetry ------------------------------------------------------------------------------
            tel.addData("slidePos", slide.getCurrentPosition());
            tel.addData("SlideVelocity", slide.getVelocity());
            tel.addData("slideButton", slideButton.isPressed());
            tel.update();

        }
    }
}