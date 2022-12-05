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
@TeleOp(name="Mecanum Drive Example", group="Iterative Opmode")
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

    public final static double SERVO_HOME = 0.0;
    public final static double SERVO_MIN_RANGE = 0.0;
    public final static double SERVO_MAX_RANGE = 1.0;
    final double SERVO_SPEED = 0.0;

    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        // app.
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        arm_slider = hardwareMap.get(DcMotor.class, "arm_slider");
        grabber = hardwareMap.get(Servo.class, "grabber");
    }

    @Override
    public void loop() {

        double y2 = gamepad2.right_stick_y;

        arm_slider.setPower(y2);

        if (gamepad1.left_trigger == 1) {
            arm_slider.setPower(1.0);
        }
        else if (gamepad1.right_trigger == 1) {
            arm_slider.setPower(-1.0);
        }
        else {
        }


        /*int a =0;
        if(gamepad1.a){
            a=1;

        }
        if(a==1){
            grabber.setPosition(1.0);
            a=0;
        }
        else{
            grabber.setPosition(0);
        }*/
        if (gamepad2.a) {
            grabber.setPosition(1.0);

        } else{
            grabber.setPosition(0);
        }


        if (gamepad1.left_trigger == 1) {
            for (int i = 0; i < 100; i++) {
                front_right.setPower(1);
                front_left.setPower(1);
                back_right.setPower(1);
                back_left.setPower(1);

            }
            for (int i = 0; i >= 100 && i < 200; i++) {
                front_right.setPower(1);
                front_left.setPower(-1);
                back_right.setPower(1);
                back_left.setPower(-1);
            }
            for (int i = 0; i >= 100 && i < 200; i++) {
                front_right.setPower(1);
                front_left.setPower(1);
                back_right.setPower(1);
                back_left.setPower(1);
            }
        } else {
        }

            /*
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            front_right.setTargetPosition(1800);
            front_left.setTargetPosition(1800);
            back_right.setTargetPosition(1800);
            back_left.setTargetPosition(1800);

            front_right.setTargetPosition(1800);
            front_left.setTargetPosition(-1800);
            back_right.setTargetPosition(1800);
            back_left.setTargetPosition(-1800);

            front_right.setTargetPosition(1800);
            front_left.setTargetPosition(1800);
            back_right.setTargetPosition(1800);
            back_left.setTargetPosition(1800);*/


        if (gamepad1.right_trigger == 1) {
            for (int i = 0; i < 100; i++) {
                front_right.setPower(-1);
                front_left.setPower(1);
                back_right.setPower(-1);
                back_left.setPower(1);

            }
            for (int i = 0; i >= 100 && i < 200; i++) {
                front_right.setPower(1);
                front_left.setPower(1);
                back_right.setPower(1);
                back_left.setPower(1);
            }
            for (int i = 0; i >= 100 && i < 200; i++) {
                front_right.setPower(-1);
                front_left.setPower(1);
                back_right.setPower(-1);
                back_left.setPower(1);
            }
            for (int i = 0; i >= 200; i++) {
            }
        } else {

        }


        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        front_left.setPower(-v1);
        front_right.setPower(v2);
        back_left.setPower(-v3);
        back_right.setPower(v4);

    }
}


