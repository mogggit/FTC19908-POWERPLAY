package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Prototype Tests")
public class Prototype_Tests extends LinearOpMode {

    private Servo cLeft;
    private Servo cRight;
    private MultipleTelemetry tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() {

        cLeft = hardwareMap.get(Servo.class, "cLeft");
        cRight = hardwareMap.get(Servo.class, "cRight");

        waitForStart();

        while (opModeIsActive()) {

            //servo: 0 ~ 0.4

            if (gamepad2.b) {
                cLeft.setPosition(0.4);
                cRight.setPosition(0.4);
            }
            else if (gamepad2.a) {
                cLeft.setPosition(0);
                cRight.setPosition(0);
            }
            else {
                cLeft.setPosition(0.05);
                cRight.setPosition(0.05);
            }

            tel.update();

        }
    }

}
