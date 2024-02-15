/* Copyright (c) 2023 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * For an introduction to AprilTags, see the ftc-docs link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@TeleOp(name="display_Teleop", group = "Concept")
//@Disabled
public class display_Teleop extends LinearOpMode
{
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 19   ; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.7;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.7;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.5;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel
    private DcMotor arm1 = null;
    private DcMotor arm2 = null;
    private DcMotor rope = null;
    private Servo armServo = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    double position = 0;
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static int DESIRED_TAG_ID = 2;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    private DistanceSensor sensorDistance;

    @Override public void runOpMode()
    {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        initAprilTag();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).

        // Right may be incorrect right now
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        arm1 = hardwareMap.get(DcMotor.class, "arm1");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");
        rope = hardwareMap.get(DcMotor.class, "rope");
        armServo = hardwareMap.get(Servo.class, "armServo");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
//        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
//        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        arm1.setDirection(DcMotor.Direction.REVERSE);
        arm2.setDirection(DcMotor.Direction.FORWARD);
        rope.setDirection(DcMotor.Direction.FORWARD);


        leftFrontDrive.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        rope.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        armServo.setPosition(0);
        leftClaw.setPosition(0);
        rightClaw.setPosition(0.53);

        if (USE_WEBCAM)
            setManualExposure(1, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            targetFound = false;
            desiredTag  = null;
            double armPower = gamepad2.right_stick_y;

            if (gamepad1.dpad_left){
                DESIRED_TAG_ID = 4;
            }
            if (gamepad1.dpad_up){
                DESIRED_TAG_ID = 5;
            }
            if (gamepad1.dpad_right){
                DESIRED_TAG_ID = 6;
            }

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if ((gamepad1.left_bumper || gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left)&& targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else {

                // drive using manual POV Joystick mode.  Slow things down to make the robot more controllable.
                drive  = -gamepad1.left_stick_y;  // Reduce drive rate to 50%.
                strafe = -gamepad1.left_stick_x;  // Reduce strafe rate to 50%.
                turn   = -gamepad1.right_stick_x;// Reduce turn rate to 33%.
                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            arm1.setPower(armPower * 0.7);
            arm2.setPower(armPower * 0.7);

            // Left joystick y is arm servo
            // Left bumper is open for claw, right is closed
            if (gamepad2.left_stick_y > 0){
                armServo.setPosition(1);
            }
            if (gamepad2.left_stick_y < 0){
                armServo.setPosition(0.4);
            }

            telemetry.addData("Gamepad2 Left Stick position", gamepad2.left_stick_y);

            // Left servo open with left trigger, close with left bumper

            // Right servo open with right trigger, close with right bumper

            if (gamepad2.left_trigger != 0)
            {
                leftClaw.setPosition(0.15);
            }
            if (gamepad2.left_bumper){
                leftClaw.setPosition(0);
            }

            if (gamepad2.right_trigger != 0)
            {
                rightClaw.setPosition(0);
            }
            if (gamepad2.right_bumper){
                rightClaw.setPosition(0.53);
            }

            // Right claw open position == 0, closed position == 0.53
            // Left claw open position == 0.15, closed position == 0
            // Map gamepad2 Down dpad to open and up d pad to close

            if (gamepad2.dpad_down)
            {
                rightClaw.setPosition(0);
                leftClaw.setPosition(0.15);
            }
            if (gamepad2.dpad_up)
            {
                rightClaw.setPosition(0.53);
                leftClaw.setPosition(0);
            }

            if (gamepad2.y){
                armServo.setPosition(0.402);

                arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                arm1.setTargetPosition(-395);
                arm2.setTargetPosition(-409);

                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Add power to the arms in order for them to move!!!!

                arm1.setPower(0.1);
                arm2.setPower(0.1);

                while (opModeIsActive() && arm1.isBusy() && arm2.isBusy()) {
                    telemetry.addLine("Moving arm");
                    telemetry.update();
                    drive  = -gamepad1.left_stick_y * 0.75;  // Reduce drive rate to 50%.
                    strafe = -gamepad1.left_stick_x * 0.75;  // Reduce strafe rate to 50%.
                    turn   = -gamepad1.right_stick_x * 0.75;// Reduce turn rate to 33%.
                    moveRobot(drive,strafe,turn);
//                    if (gamepad2.left_trigger != 0)
//                    {
//                        leftClaw.setPosition(0.15);
//                    }
//                    if (gamepad2.left_bumper){
//                        leftClaw.setPosition(0);
//                    }
//
//                    if (gamepad2.right_trigger != 0)
//                    {
//                        rightClaw.setPosition(0);
//                    }
//                    if (gamepad2.right_bumper){
//                        rightClaw.setPosition(0.53);
//                    }
                }

                arm1.setPower(0);
                arm2.setPower(0);

                sleep(1000);

                arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                arm1.setPower(armPower * 0.7);
                arm2.setPower(armPower * 0.7);

                arm1.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
                arm2.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            }

            if (gamepad1.dpad_down){
                arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (gamepad2.a){
                arm1.setTargetPosition(0);
                arm2.setTargetPosition(0);

                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                arm1.setPower(0.1);
                arm2.setPower(0.1);

                while (opModeIsActive() && arm1.isBusy() && arm2.isBusy()) {
                    telemetry.addLine("Moving arm");
                    telemetry.update();
                    drive  = -gamepad1.left_stick_y * 0.75;  // Reduce drive rate to 50%.
                    strafe = -gamepad1.left_stick_x * 0.75;  // Reduce strafe rate to 50%.
                    turn   = -gamepad1.right_stick_x * 0.75;// Reduce turn rate to 33%.
                    moveRobot(drive,strafe,turn);
                    if (gamepad2.left_trigger != 0)
                    {
                        leftClaw.setPosition(0.15);
                    }
                    if (gamepad2.left_bumper){
                        leftClaw.setPosition(0);
                    }

                    if (gamepad2.right_trigger != 0)
                    {
                        rightClaw.setPosition(0);
                    }
                    if (gamepad2.right_bumper){
                        rightClaw.setPosition(0.53);
                    }
                }

                arm1.setPower(0);
                arm2.setPower(0);

                arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                telemetry.addLine("Done");

                arm1.setPower(armPower * 0.7);
                arm2.setPower(armPower * 0.7);
            }

            if (gamepad2.x){
                armServo.setPosition(0.448);

                arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                arm1.setTargetPosition(-203);
                arm2.setTargetPosition(-193);

                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Add power to the arms in order for them to move!!!!

                arm1.setPower(0.1);
                arm2.setPower(0.1);

                while (opModeIsActive() && arm1.isBusy() && arm2.isBusy()) {
                    telemetry.addLine("Moving arm");
                    telemetry.update();
                    drive  = -gamepad1.left_stick_y * 0.75;  // Reduce drive rate to 50%.
                    strafe = -gamepad1.left_stick_x * 0.75;  // Reduce strafe rate to 50%.
                    turn   = -gamepad1.right_stick_x * 0.75;// Reduce turn rate to 33%.
                    moveRobot(drive,strafe,turn);
                    if (gamepad2.left_trigger != 0)
                    {
                        leftClaw.setPosition(0.15);
                    }
                    if (gamepad2.left_bumper){
                        leftClaw.setPosition(0);
                    }

                    if (gamepad2.right_trigger != 0)
                    {
                        rightClaw.setPosition(0);
                    }
                    if (gamepad2.right_bumper){
                        rightClaw.setPosition(0.53);
                    }
                }

                arm1.setPower(0);
                arm2.setPower(0);

                sleep(1000);

                arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                arm1.setPower(armPower * 0.7);
                arm2.setPower(armPower * 0.7);

                arm1.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
                arm2.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            }

            if(gamepad1.right_trigger != 0){
                rope.setPower(1);
            }
            if (gamepad1.right_trigger == 0){
                rope.setPower(0);
            }
            if (gamepad1.right_bumper){
                rope.setPower(-1);
            }

            // Gamepad2.y causes the arm to move up and go to the correct position to drop pixels

            telemetry.addData("Gamepad 2 Left Stick Y", gamepad2.left_stick_y);

            telemetry.addData("Right Claw position", rightClaw.getPosition());
            telemetry.addData("Left Claw position", leftClaw.getPosition());
            telemetry.addData("Arm Servo position", armServo.getPosition());
            telemetry.addData("Arm 1 position", arm1.getCurrentPosition());
            telemetry.addData("Arm 2 position", arm2.getCurrentPosition());

//            telemetry.addData("deviceName", sensorDistance.getDeviceName() );
//            telemetry.addData("range", String.format("%.01f mm", sensorDistance.getDistance(DistanceUnit.MM)));
//            telemetry.addData("range", String.format("%.01f cm", sensorDistance.getDistance(DistanceUnit.CM)));
//            telemetry.addData("range", String.format("%.01f m", sensorDistance.getDistance(DistanceUnit.METER)));
//            telemetry.addData("range", String.format("%.01f in", sensorDistance.getDistance(DistanceUnit.INCH)));
//
//            // Rev2mDistanceSensor specific methods.
//            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
//            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

            telemetry.update();

            sleep(10);
        }
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }


    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}
