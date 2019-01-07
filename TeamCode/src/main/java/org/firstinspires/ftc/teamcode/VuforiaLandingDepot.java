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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;


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
@Autonomous(name = "VuforiaLandingDepot", group = "pushbot")
//@Disabled
public class VuforiaLandingDepot extends LinearOpMode {
    HardwareAri robot = new HardwareAri();
    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;


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
    private static final String VUFORIA_KEY = "AejTuGz/////AAAAmdZRjZEcgUHshHAYuhaOvWZCO2QXzSdYmf3Xjfz/Axpr6iIN7krl+sSzU/Kba24jaF51aXfZpOKxozk6xNgX/2M5V1iXAZH4C9EsbIsvYImhLq+OXc89yWUzROyEgP8zpgkMbBPxE8IUUY7UNLavSTy55KIYyyl+Q/qHvOFL2iyGVx4VhkbZ50+bk0b4LsVOQhifgbIDoJm0dSTwKC2bDfv3GYEcJtyMuZVBa8if4zwc6Nlz4kOoOaIW7pGU5e4danjpAqIuoUoGHzUF0rYuSfM3RfUKyBcIPcTCRXsjuQKe0Yv14wQav6o1yTr/7HEKMhZTTsVwXjfohOvYLwbHTsmU8/Ww6JliCUccO8LGG6Ua";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    @Override
    public void runOpMode() {

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        robot.init(hardwareMap);
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset.
        telemetry.addData("Path0",  "Starting at %7d :%7d : %7d :%7d",
                robot.frontLeftDrive.getCurrentPosition(),
                robot.frontRightDrive.getCurrentPosition(),
                robot.backRightDrive.getCurrentPosition(),
                robot.backLeftDrive.getCurrentPosition());
        telemetry.update();

        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            /* se localiza el gold mineral si encuentra  objetos en a imagen */
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {

                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    robot.gate.setPosition(.825);
                                    sleep(2000);
                                    robot.lift.setPower(.5);
                                    sleep(4200);
                                    robot.lift.setPower(0);
                                    sleep(1000);
                                    robot.frontLeftDrive.setPower(-1);
                                    robot.frontRightDrive.setPower(1);
                                    robot.backRightDrive.setPower(-1);
                                    robot.backLeftDrive.setPower(1);
                                    sleep(300);
                                    robot.frontRightDrive.setPower(0);
                                    robot.backRightDrive.setPower(0);
                                    robot.frontLeftDrive.setPower(0);
                                    robot.backLeftDrive.setPower(0);
                                    sleep(2000);
                                    robot.frontRightDrive.setPower(-1);
                                    robot.backRightDrive.setPower(-1);
                                    robot.frontLeftDrive.setPower(1);
                                    robot.backLeftDrive.setPower(1);
                                    sleep(300);
                                    robot.frontRightDrive.setPower(-1);
                                    robot.backRightDrive.setPower(-1);
                                    robot.frontLeftDrive.setPower(-1);
                                    robot.backLeftDrive.setPower(-1);
                                    sleep(1000);
                                    robot.frontRightDrive.setPower(0);
                                    robot.backRightDrive.setPower(0);
                                    robot.frontLeftDrive.setPower(0);
                                    robot.backLeftDrive.setPower(0);
                                    sleep(1000);
                                    robot.frontRightDrive.setPower(1);
                                    robot.backRightDrive.setPower(1);
                                    robot.frontLeftDrive.setPower(-1);
                                    robot.backLeftDrive.setPower(-1);
                                    sleep(300);
                                    robot.frontRightDrive.setPower(-1);
                                    robot.backRightDrive.setPower(-1);
                                    robot.frontLeftDrive.setPower(-1);
                                    robot.backLeftDrive.setPower(-1);
                                    sleep(300);
                                    robot.frontRightDrive.setPower(0);
                                    robot.backRightDrive.setPower(0);
                                    robot.frontLeftDrive.setPower(0);
                                    robot.backLeftDrive.setPower(0);
                                    sleep(1000);
                                    robot.recogedor.setPower(.7);
                                    sleep(500);
                                    robot.recogedor.setPower(0);
                                    sleep(100000);

                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    robot.gate.setPosition(.825);
                                    sleep(2000);
                                    robot.lift.setPower(.5);
                                    sleep(4200);
                                    robot.lift.setPower(0);
                                    sleep(1000);
                                    robot.frontLeftDrive.setPower(-1);
                                    robot.frontRightDrive.setPower(1);
                                    robot.backRightDrive.setPower(-1);
                                    robot.backLeftDrive.setPower(1);
                                    sleep(300);
                                    robot.frontRightDrive.setPower(0);
                                    robot.backRightDrive.setPower(0);
                                    robot.frontLeftDrive.setPower(0);
                                    robot.backLeftDrive.setPower(0);
                                    sleep(2000);
                                    robot.frontRightDrive.setPower(1);
                                    robot.backRightDrive.setPower(1);
                                    robot.frontLeftDrive.setPower(-1);
                                    robot.backLeftDrive.setPower(-1);
                                    sleep(300);
                                    robot.frontRightDrive.setPower(-1);
                                    robot.backRightDrive.setPower(-1);
                                    robot.frontLeftDrive.setPower(-1);
                                    robot.backLeftDrive.setPower(-1);
                                    sleep(1000);
                                    robot.frontRightDrive.setPower(0);
                                    robot.backRightDrive.setPower(0);
                                    robot.frontLeftDrive.setPower(0);
                                    robot.backLeftDrive.setPower(0);
                                    sleep(1000);
                                    robot.frontRightDrive.setPower(-1);
                                    robot.backRightDrive.setPower(-1);
                                    robot.frontLeftDrive.setPower(1);
                                    robot.backLeftDrive.setPower(1);
                                    sleep(300);
                                    robot.frontRightDrive.setPower(-1);
                                    robot.backRightDrive.setPower(-1);
                                    robot.frontLeftDrive.setPower(-1);
                                    robot.backLeftDrive.setPower(-1);
                                    sleep(250);
                                    robot.frontRightDrive.setPower(0);
                                    robot.backRightDrive.setPower(0);
                                    robot.frontLeftDrive.setPower(0);
                                    robot.backLeftDrive.setPower(0);
                                    sleep(1000);
                                    robot.recogedor.setPower(.3);
                                    sleep(500);
                                    robot.recogedor.setPower(0);
                                    sleep(100000);

                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    robot.gate.setPosition(.825);
                                    sleep(2000);
                                    robot.lift.setPower(.5);
                                    sleep(4200);
                                    robot.lift.setPower(0);
                                    sleep(1000);
                                    robot.frontLeftDrive.setPower(-1);
                                    robot.frontRightDrive.setPower(1);
                                    robot.backRightDrive.setPower(-1);
                                    robot.backLeftDrive.setPower(1);
                                    sleep(300);
                                    robot.frontRightDrive.setPower(0);
                                    robot.backRightDrive.setPower(0);
                                    robot.frontLeftDrive.setPower(0);
                                    robot.backLeftDrive.setPower(0);
                                    sleep(2000);
                                    robot.frontRightDrive.setPower(-1);
                                    robot.backRightDrive.setPower(-1);
                                    robot.frontLeftDrive.setPower(-1);
                                    robot.backLeftDrive.setPower(-1);
                                    sleep(1500);
                                    robot.frontRightDrive.setPower(0);
                                    robot.backRightDrive.setPower(0);
                                    robot.frontLeftDrive.setPower(0);
                                    robot.backLeftDrive.setPower(0);
                                    sleep(1000);
                                    robot.recogedor.setPower(.3);
                                    sleep(500);
                                    robot.recogedor.setPower(0);
                                    sleep(100000);


                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel
                                        + gravity.yAccel * gravity.yAccel
                                        + gravity.zAccel * gravity.zAccel));
                    }
                });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

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
    public void encoderDrive(double speed,
                             double frontLeftInches, double frontRightInches,
                             double backLeftInches, double backRightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = robot.frontLeftDrive.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = robot.frontRightDrive.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);
            newBackLeftTarget = robot.frontLeftDrive.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = robot.frontRightDrive.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);


            robot.frontLeftDrive.setTargetPosition(newFrontLeftTarget);
            robot.frontRightDrive.setTargetPosition(newFrontRightTarget);
            robot.backLeftDrive.setTargetPosition(newBackLeftTarget);
            robot.backRightDrive.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeftDrive.setPower(Math.abs(speed));
            robot.frontRightDrive.setPower(Math.abs(speed));
            robot.backRightDrive.setPower(Math.abs(speed));
            robot.backLeftDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy() && robot.backRightDrive.isBusy() && robot.backLeftDrive.isBusy()  )) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newFrontLeftTarget,  newFrontRightTarget, newBackLeftTarget, newBackRightTarget );
                telemetry.addData("Path2",  "Running at %7d :%7d : %7d :%7d",
                        robot.frontRightDrive.getCurrentPosition(),
                        robot.frontLeftDrive.getCurrentPosition(),
                        robot.backLeftDrive.getCurrentPosition(),
                        robot.backRightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.backRightDrive.setPower(0);
            robot.backLeftDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


}
