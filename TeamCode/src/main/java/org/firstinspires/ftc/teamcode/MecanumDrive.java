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
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;
    //private Servo gabe_servo_test;
    private Servo grabber;

    public final static double SERVO_HOME = 0.0;
    public final static double SERVO_MIN_RANGE = 0.0;
    public final static double SERVO_MAX_RANGE = 1.0;
    final double SERVO_SPEED = 0.0;

    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        // app.
        front_left   = hardwareMap.get(DcMotor.class, "front_left");
        front_right  = hardwareMap.get(DcMotor.class, "front_right");
        back_left    = hardwareMap.get(DcMotor.class, "back_left");
        back_right   = hardwareMap.get(DcMotor.class, "back_right");
        //gabe_servo_test = hardwareMap.get(Servo.class, "gabe_servo_test");
        grabber = hardwareMap.get(Servo.class, "grabber");
    }

    @Override
    public void loop() {

        /*
        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        double drive  = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_y;
        double twist  = gamepad1.right_stick_y;

        /*
         * If we had a gyro and wanted to do field-oriented control, here
         * is where we would implement it.
         *
         * The idea is fairly simple; we have a robot-oriented Cartesian (x,y)
         * coordinate (strafe, drive), and we just rotate it by the gyro
         * reading minus the offset that we read in the init() method.
         * Some rough pseudocode demonstrating:
         *
         * if Field Oriented Control:
         *     get gyro heading
         *     subtract initial offset from heading
         *     convert heading to radians (if necessary)
         *     new strafe = strafe * cos(heading) - drive * sin(heading)
         *     new drive  = strafe * sin(heading) + drive * cos(heading)


        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.
        double[] speeds = {
                (drive + strafe + twist),
                (-1)*(drive - strafe - twist),
                (-1)*(drive - strafe + twist),
                (-1)*(drive + strafe - twist)
        };

        // Because we are adding vectors and motors only take values between
        // [-1,1] we may need to normalize them.

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for (double speed : speeds) {
            if (max < Math.abs(speed)) max = Math.abs(speed);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        // apply the calculated values to the motors.
        front_left.setPower(speeds[0]);
        front_right.setPower(speeds[1]);
        back_left.setPower(speeds[2]);
        back_right.setPower(speeds[3]);
        */

        /*
        gabe_servo_test.setPosition(Servo.MIN_POSITION);
        if(gamepad1.a) {
            gabe_servo_test.setPosition(Servo.MAX_POSITION);
        }
        */



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
        if(gamepad1.a){
            grabber.setPosition(1.0);

        }
        else {
           grabber.setPosition(0);
        }



        if(gamepad1.left_trigger == 1){
            front_left.setPower(1);
            front_right.setPower(1);
            back_left.setPower(1);
            back_right.setPower(1);
            try {
                wait(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            front_left.setPower(-1);
            front_right.setPower(-1);
            back_left.setPower(1);
            back_right.setPower(1);
            try {
                    wait(1000);
                } catch(InterruptedException e) {
                    e.printStackTrace();
                }
            }
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
