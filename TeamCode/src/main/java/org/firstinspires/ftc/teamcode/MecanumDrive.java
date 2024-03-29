package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * This is an example minimal implementation of the mecanum drivetrain
 * for demonstration purposes.  Not tested and not guaranteed to be bug free.
 *
 * @author Brandon Gong
 */
@TeleOp(name="Tin Diesel Drive", group="Iterative Opmode")
public class MecanumDrive extends OpMode {

    /*
     * The mecanum drivetrain involves four separate motors that spin in
     * different directions and different speeds to produce the desired
     * movement at the desired speed.
     */

    // declare and initialize four DcMotors.
    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;
    private DcMotor arm_slider = null;
    private Servo grabber;

    public final static double SERVO_HOME = 0.5;
    public final static double SERVO_MIN_RANGE = 0.45;
    public final static double SERVO_MAX_RANGE = 0.63;

    public double multiplier = 1;

    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        // app.
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        arm_slider = hardwareMap.get(DcMotor.class, "arm_slider");
        grabber = hardwareMap.get(Servo.class, "arm_grabber");

        grabber.setPosition(SERVO_HOME);
    }

    @Override
    public void loop() {


        double y2 = -gamepad2.right_stick_y;

        if(y2 > -0.25 || y2 < 0.25){

            arm_slider.setPower(-y2);
        }

        if (gamepad2.a == true) {
            grabber.setPosition(SERVO_MAX_RANGE);

        }
        else if(gamepad2.b == true) {
            grabber.setPosition(SERVO_MIN_RANGE);
        }

        if(gamepad1.right_bumper == true){

            multiplier = 0.5;

        }
        else if(gamepad1.left_bumper == true){

            multiplier = 1;

        }

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = -gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        if(v1 < 0.5 || v1 > -0.5 && v2 < 0.5 || v2 > -0.5 && v3 < 0.5 || v3 > -0.5 && v4 < 0.5 || v4 > -0.5) {
            front_left.setPower(-v1*multiplier);
            front_right.setPower(v2*multiplier);
            back_left.setPower(-v3*multiplier);
            back_right.setPower(v4*multiplier);
        }

    }
}


