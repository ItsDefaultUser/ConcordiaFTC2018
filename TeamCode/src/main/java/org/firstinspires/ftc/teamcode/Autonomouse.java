package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="java Linear Autonomous", group="Linear Opmode")
public class Autonomouse extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    private DcMotor armMain = null;
    //private Servo armClawL = null;
    //private Servo armClawR = null;

    private void ctrlLeft(double pow) {
        leftFront.setPower(-pow);
        leftBack.setPower(-pow);
    }

    private void ctrlRight(double pow) {
        rightFront.setPower(pow);
        rightBack.setPower(pow);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize state
        int state = 0;

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        armMain = hardwareMap.get(DcMotor.class, "arm_main");
        //clawL = hardwareMap.get(Servo.class, "claw_left");
        //clawR = hardwareMap.get(Servo.class, "claw_right");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Send calculated power to wheels
            if (state == 0) {
                ctrlLeft(0.5);
                ctrlRight(0.5);
                if (getRuntime() > 10) {
                    state = 1;
                }
            } else if (state == 1) {
                ctrlLeft(0);
                ctrlRight(0);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motor Status", "Motor Power: " + leftFront.getPower());
            telemetry.update();
        }
    }
}

