package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Drivetrain {

    /*
    Motor positions:
       ------
    m1|      |m3
      |      |
      |      |
    m2|      |m4
       ------
     */

    private final DcMotorEx m1;
    private final DcMotorEx m2;
    private final DcMotorEx m3;
    private final DcMotorEx m4;

    // Constructor
    public Drivetrain(DcMotorEx m1, DcMotorEx m2, DcMotorEx m3, DcMotorEx m4) {
        this.m1 = m1;
        this.m2 = m2;
        this.m3 = m3;
        this.m4 = m4;
        // going forward is NEGATIVE for m1/2, POSITIVE for m3/4
    }

    public void setTolerance(int tolerance) {
        m1.setTargetPositionTolerance(tolerance);
        m2.setTargetPositionTolerance(tolerance);
        m3.setTargetPositionTolerance(tolerance);
        m4.setTargetPositionTolerance(tolerance);
    }

    public void resetEncoders() {
        m1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        m1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    // Set the target power and distance and start moving
    public void runMotorDistance(double power, int p1, int p2, int p3, int p4) {
        resetEncoders();

        // PID target
        m1.setTargetPosition(p1);
        m2.setTargetPosition(p2);
        m3.setTargetPosition(p3);
        m4.setTargetPosition(p4);

        m1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        m2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        m3.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        m4.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // PID Output
        m1.setPower(power);
        m2.setPower(power);
        m3.setPower(power);
        m4.setPower(power);
    }

    public void runMotorPower(double pow1, double pow2, double pow3, double pow4) {
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m1.setPower(pow1);
        m2.setPower(pow2);
        m3.setPower(pow3);
        m4.setPower(pow4);
    }

    // Check if motor reached target position then stop motor
    public boolean stopMotor() {
        if (!m1.isBusy() && !m2.isBusy() && !m3.isBusy() && !m4.isBusy()) {
            m1.setPower(0);
            m2.setPower(0);
            m3.setPower(0);
            m4.setPower(0);
            m1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            m2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            m3.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            m4.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            return true;
        }
        else {
            return false;
        }
    }

    // Drive for TeleOp
    public void drive(Gamepad gamepad2, double sensitivity) {
        m1.setPower((-gamepad2.left_stick_x + gamepad2.left_stick_y + gamepad2.left_trigger - gamepad2.right_trigger) * sensitivity);
        m2.setPower((gamepad2.left_stick_x + gamepad2.left_stick_y + gamepad2.left_trigger - gamepad2.right_trigger) * sensitivity);
        m3.setPower((-gamepad2.left_stick_x - gamepad2.left_stick_y + gamepad2.left_trigger - gamepad2.right_trigger) * sensitivity);
        m4.setPower((gamepad2.left_stick_x - gamepad2.left_stick_y + gamepad2.left_trigger - gamepad2.right_trigger) * sensitivity);
    }

    // Set the motor modes for TeleOp
    public void driveMode() {
        m1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        m1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        m2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m3.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        m3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m4.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        m4.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    // Return encoder velocity values for selected motor
    public double getEncoderVelocity(String motorName) {
        if (motorName.equals("m1")) {
            return m1.getVelocity();
        }
        else if (motorName.equals("m2")) {
            return m2.getVelocity();
        }
        else if (motorName.equals("m3")) {
            return m3.getVelocity();
        }
        else if (motorName.equals("m4")) {
            return m4.getVelocity();
        }
        else {
            return 0;
        }
    }

    // Return encoder position values for selected motor
    public double getEncoderPosition(String motorName) {
        if (motorName.equals("m1")) {
            return m1.getCurrentPosition();
        }
        else if (motorName.equals("m2")) {
            return m2.getCurrentPosition();
        }
        else if (motorName.equals("m3")) {
            return m3.getCurrentPosition();
        }
        else if (motorName.equals("m4")) {
            return m4.getCurrentPosition();
        }
        else {
            return 0;
        }
    }

    //set PIDF coefficients for all motors
    public void setPIDF(double p, double i, double d, double f, double pos_p) {
        m1.setVelocityPIDFCoefficients(p, i, d, f);
        m1.setPositionPIDFCoefficients(pos_p);
        m2.setVelocityPIDFCoefficients(p, i, d, f);
        m2.setPositionPIDFCoefficients(pos_p);
        m3.setVelocityPIDFCoefficients(p, i, d, f);
        m3.setPositionPIDFCoefficients(pos_p);
        m4.setVelocityPIDFCoefficients(p, i, d, f);
        m4.setPositionPIDFCoefficients(pos_p);
    }

//    public void runSingleMotor()
}
