/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Autonomous TFlow", group = "Linear Opmode")
public class AutonomousTFlow extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AWlb8T3/////AAABmQyNZ3FgmkP8iv9V7VZHj0pex4kaTGnoU54vUiIzC/RnL27Wq6dZ93BqEGYH3AlTBN7YpQIiaP8FobpfvMb1wvKohnlDTl/fDut1rvccJcRjPC1EJ2NxzXRdznrWtxddFtiZBcdoYj74Uee8KdQILlRukh9L7R7gZ1QeOWR3uN39Z9tOr4FWGH9yS6nDVoiBaLcKSyxbKuLaau11+nNBz4FAdsgOFQBqbB2j8guYybTR+lGV+EgR0fiNVyHrKFHP7hUaZotZT4LPKIfkohYnVLfdFgh8fmSugT/TkPDHO2SvUn+b4QnLbaAKl18Mq6Z8F3O2UkpJL+BHS5mgcXpmQbuZZngTd0GbgxuvgHuG/7D2";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */

    // Define variables and functions
    private TFObjectDetector tfod;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    private DcMotor linearSlide = null;

    private void ctrlMotor(double LPow, double RPow) {
        leftFront.setPower(-LPow);
        leftBack.setPower(-LPow);
        rightFront.setPower(RPow);
        rightBack.setPower(RPow);
    }

    private void ctrlSlide(double pow) {
        linearSlide.setPower(pow);
    }

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        // Initialize variables
        int step = 0;
        int goldMineralPos = 9999;
        int objDetected = 0;
        double sincePrev = 0.0;

        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");

        /** Wait for the game to begin */
        tfod.activate();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // Send calculated power to wheels
            if (step == 0) {
                ctrlSlide(0.1);
                if ((getRuntime() - sincePrev) >= 0.5) {
                    ctrlSlide(0.0);
                    sincePrev = getRuntime();
                    step = 1;
                }
            } else if (step == 1) {
                ctrlMotor(0.5,-0.5);
                if ((getRuntime() - sincePrev) >= 0.05) {
                    ctrlMotor(0.0, 0.0);
                    sincePrev = getRuntime();
                    step = 10;
                }
            } else if (step == 10) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null){
                    objDetected = updatedRecognitions.size();
                    if (updatedRecognitions.size() == 2) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getRight();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getRight();
                            } else {
                                silverMineral2X = (int) recognition.getRight();
                            }
                        }
                        if (goldMineralX == -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            goldMineralPos = -1;
                        } else if (silverMineral2X == -1 && goldMineralX != -1 && silverMineral1X != -1) {
                            if (goldMineralX < silverMineral1X) {
                                goldMineralPos = 0;
                            } else {
                                goldMineralPos = 1;
                            }
                        }
                    }
                }
                if (goldMineralPos != 9999) {
                    sincePrev = getRuntime();
                    step = 11;
                }
            } else if (step == 11) {
                ctrlMotor(0.5, 0.5);
                if ((getRuntime() - sincePrev) >= 0.2) {
                    ctrlMotor(0.0, 0.0);
                    sincePrev = getRuntime();
                    step = 12;
                }
            } else if (step == 12) {
                if (goldMineralPos == -1) {
                    ctrlMotor(-0.5, 0.5);
                } else if (goldMineralPos == 1) {
                    ctrlMotor(0.5, -0.5);
                }
                if ((getRuntime() - sincePrev) >= 0.2) {
                    ctrlMotor(0.0, 0.0);
                    sincePrev = getRuntime();
                    step = 13;
                }
            } else if (step == 13) {
                ctrlMotor(0.5, 0.5);
                if ((getRuntime() - sincePrev) >= 0.5) {
                    ctrlMotor(0.0, 0.0);
                    sincePrev = getRuntime();
                    tfod.shutdown();
                    step = 14;
                }
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Drive Power", "left (%.2f), right (%.2f)", leftFront.getPower(), rightFront.getPower());
            telemetry.addData("Obj detected", "(%.2f)", (double)(objDetected));
            telemetry.addData("Gold mineral location", "(%.2f)", (double)(goldMineralPos));
            telemetry.update();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
