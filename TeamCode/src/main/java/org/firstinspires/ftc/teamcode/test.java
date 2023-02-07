package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


/**
 * This is an example minimal implementation of the mecanum drivetrain
 * for demonstration purposes.  Not tested and not guaranteed to be bug free.
 *
 * @author Brandon Gong
 */
@TeleOp(name="Test", group="Iterative Opmode")
public class test extends LinearOpMode {

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
    public void runOpMode() throws InterruptedException {


        // Name strings must match up with the config on the Robot Controller
        // app.
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        arm_slider = hardwareMap.get(DcMotor.class, "arm_slider");
        grabber = hardwareMap.get(Servo.class, "arm_grabber");

        grabber.setPosition(SERVO_HOME);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(270));

        TrajectorySequence test = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(10)
                .build();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                drive.setPoseEstimate(startPose);

                drive.followTrajectorySequence(test);
            }
        }

    }
}


