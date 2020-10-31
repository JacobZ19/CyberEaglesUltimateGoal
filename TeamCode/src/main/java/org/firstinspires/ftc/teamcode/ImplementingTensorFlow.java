package org.firstinspires.ftc.teamcode;

import android.hardware.camera2.CameraDevice;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import android.hardware.camera2.CameraDevice;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;



import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;



import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.OdometryGlobalCoordinatePosition;

/**
 * Created by Sarthak on 10/4/2019.
 */

public class ImplementingTensorFlow {
    //Drive motors
    LinearOpMode opMode = null;
    public ElapsedTime     runtime = new ElapsedTime();
    public ImplementingTensorFlow (LinearOpMode opMode){
        this.opMode= opMode;}
    DcMotor rightFrontDrive, rightBackDrive, leftFrontDrive, leftBackDrive;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;
    // Tfod objects
    public static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Quad";
    public static final String LABEL_SECOND_ELEMENT = "Single";
    // Vuforia Key
    public static final String VUFORIA_KEY =
            " Ac4K6BD/////AAABmQmEfMSCD0j8hOxBFYTS/CQj/pGySibYpkLudGb7MN12FPBJ3Om18kKjOQTFwk8o9C3FEY0LIcBYqcPMMB35sfLHF2uwiF/9ElfONAFain0CysTHKvL/mOpSZZxOqevoexo9iNlxftfciARbiruvu5kYGZroBCho5R6WHzHjbGfEAGWjIsckDKRvQQkKw5p5N21GqH5ium/tN/TO0asmGwz0RTb4Djt+P8FSynFSJnmC3gsq97fUipK802hR2A4SvzgJKEmrgoNnOxKGx7E/AkBKGY/mIrTy7/Trhq/mnK6STtm4Vl9GBfnsOj3KAexo7JdfFNCPEuKrMXs7wHfvnEYRS4Lwwf/kwSISAQvIN+8N";

    public VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    public TFObjectDetector tfod;

    final double COUNTS_PER_INCH = 307.699557;
    final int FORWARD = 1;
    final int BACKWARD = 2;
    final int STRAFELEFT = 3;
    final int STRAFERIGHT = 4;


    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "right_front", rbName = "right_back", lfName = "left_front", lbName = "left_back";
    String verticalLeftEncoderName = rfName, verticalRightEncoderName = lfName, horizontalEncoderName = lbName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;



    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members




    //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION




    //      globalPositionUpdate.reverseRightEncoder();
//        globalPositionUpdate.reverseNormalEncoder();

    //        globalPositionUpdate.reverseLeftEncoder();







    public void turn (double robotPower, double desiredRobotOrientation, double allowableAngleError, double timeoutS) {
        runtime.reset();

        double orientationAngle = globalPositionUpdate.returnOrientation();

        if (orientationAngle < 0){
            orientationAngle += 360;
        }

        double angleDifference = desiredRobotOrientation - orientationAngle;

        while (angleDifference > 180)  angleDifference -= 360;
        while (angleDifference <= -180) angleDifference += 360;

        while (opMode.opModeIsActive() && Math.abs(angleDifference) > allowableAngleError && (runtime.seconds() < timeoutS)) {



            orientationAngle = globalPositionUpdate.returnOrientation();

            if (orientationAngle < 0){
                orientationAngle += 360;
            }

            angleDifference = desiredRobotOrientation - orientationAngle;

            while (angleDifference > 180)  angleDifference -= 360;
            while (angleDifference <= -180) angleDifference += 360;

            if (Math.abs(angleDifference) > allowableAngleError) {

                if ( angleDifference > 0) {
                    //turn right
                    leftFrontDrive.setPower(robotPower);
                    rightFrontDrive.setPower(robotPower);
                    leftBackDrive.setPower(robotPower);
                    rightBackDrive.setPower(robotPower);

                    opMode.telemetry.addData("Turning right", "now");
                    opMode.telemetry.addData("X Position (inches)", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
                    opMode.telemetry.addData("Y Position (inches)", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
                    opMode.telemetry.addData("Orientation (Degrees)", orientationAngle);

                    opMode.telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
                    opMode.telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
                    opMode.telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

                    opMode.telemetry.update();

                }
                else if (angleDifference< 0) {
                    leftFrontDrive.setPower(-robotPower);
                    rightFrontDrive.setPower(-robotPower);
                    leftBackDrive.setPower(-robotPower);
                    rightBackDrive.setPower(-robotPower);


                    opMode. telemetry.addData("Turning left", "now");
                    opMode. telemetry.addData("X Position (inches)", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
                    opMode.telemetry.addData("Y Position (inches)", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
                    opMode.telemetry.addData("Orientation (Degrees)", orientationAngle);


                    opMode.telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
                    opMode. telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
                    opMode. telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

                    opMode.telemetry.update();

                    //turn left
                }

            }



        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);


    }





    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError, double timeoutS) {
        goToPosition(targetXPosition, targetYPosition, robotPower, desiredRobotOrientation, allowableDistanceError, timeoutS, FORWARD);
    }
    // To fix Strafe create second method

    public void initDriveHardwareMap(){
        String rfName = "right_front", rbName = "right_back", lfName = "left_front", lbName = "left_back";
        String vlEncoderName = rfName, vrEncoderName = lfName, hEncoderName = lbName;

        rightFrontDrive = opMode.hardwareMap.dcMotor.get(rfName);
        rightBackDrive = opMode.hardwareMap.dcMotor.get(rbName);
        leftFrontDrive = opMode.hardwareMap.dcMotor.get(lfName);
        leftBackDrive = opMode.hardwareMap.dcMotor.get(lbName);





        verticalLeft = opMode.hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = opMode.hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = opMode.hardwareMap.dcMotor.get(hEncoderName);

        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);




        opMode.telemetry.addData("Status", "Init Complete");
        opMode.telemetry.update();




        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
//        globalPositionUpdate.reverseRightEncoder();
//        globalPositionUpdate.reverseNormalEncoder();

        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        verticalLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);

        opMode.telemetry.addData("Status", "Hardware Map Init Complete");
        opMode.telemetry.update();
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError, double timeoutS, int direction) {

        double distanceToXTarget = targetXPosition*COUNTS_PER_INCH - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition*COUNTS_PER_INCH - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        runtime.reset();

        while (opMode.opModeIsActive() && distance > allowableDistanceError*COUNTS_PER_INCH && (runtime.seconds() < timeoutS)) {

            distance = Math.hypot(distanceToXTarget,distanceToYTarget);

            distanceToXTarget = targetXPosition*COUNTS_PER_INCH - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition*COUNTS_PER_INCH - globalPositionUpdate.returnYCoordinate();


            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));
            if (direction == BACKWARD) {
                robotMovementAngle +=180;
            }

            if (direction == STRAFELEFT) {
                robotMovementAngle +=90;
            }

            if (direction == STRAFERIGHT) {
                robotMovementAngle +=270;
            }

            if (robotMovementAngle <0){
                robotMovementAngle += 360;
            }

            double robotMovementXComponent = calculateX(robotMovementAngle, robotPower);
            double robotMovementYComponent = calculateY(robotMovementAngle, robotPower);

            double pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();

            double orientationAngle = globalPositionUpdate.returnOrientation();

            if (orientationAngle < 0){
                orientationAngle += 360;
            }

            double angleDifference = robotMovementAngle - orientationAngle;

            while (angleDifference > 180)  angleDifference -= 360;
            while (angleDifference <= -180) angleDifference += 360;

            if (Math.abs(angleDifference) > 10) {

                if ( angleDifference > 0) {
                    //turn right
                    leftFrontDrive.setPower(robotPower);
                    rightFrontDrive.setPower(robotPower);
                    leftBackDrive.setPower(robotPower);
                    rightBackDrive.setPower(robotPower);

                    opMode.telemetry.addData("Turning right", "now");
                    opMode.telemetry.addData("X Position (inches)", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
                    opMode.telemetry.addData("Y Position (inches)", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
                    opMode.telemetry.addData("Orientation (Degrees)", orientationAngle);
                    opMode.telemetry.addData("Robot Movement Angle", robotMovementAngle);


                    opMode.telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
                    opMode.telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
                    opMode.telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

                    opMode.telemetry.update();

                }
                else if (angleDifference< 0) {
                    leftFrontDrive.setPower(-robotPower);
                    rightFrontDrive.setPower(-robotPower);
                    leftBackDrive.setPower(-robotPower);
                    rightBackDrive.setPower(-robotPower);


                    opMode. telemetry.addData("Turning left", "now");
                    opMode. telemetry.addData("X Position (inches)", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
                    opMode.telemetry.addData("Y Position (inches)", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
                    opMode.telemetry.addData("Orientation (Degrees)", orientationAngle);
                    opMode.telemetry.addData("Robot Movement Angle", robotMovementAngle);


                    opMode.telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
                    opMode. telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
                    opMode. telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

                    opMode.telemetry.update();

                    //turn left
                }

            }

            else {
                //drive to target
                opMode.telemetry.addData("drive", "This is the else loop");
                opMode.telemetry.addData("X Position Inches", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
                opMode.telemetry.addData("Y Position Inches", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
                opMode.telemetry.addData("Distance to X Target", distanceToXTarget / COUNTS_PER_INCH);
                opMode.telemetry.addData("Distance To Y Target", distanceToYTarget / COUNTS_PER_INCH);
                opMode.telemetry.update();
                if (direction == FORWARD) {
                    leftFrontDrive.setPower(robotPower);
                    rightFrontDrive.setPower(-robotPower);
                    leftBackDrive.setPower(robotPower);
                    rightBackDrive.setPower(-robotPower);
                } else if (direction == BACKWARD) {
                    leftFrontDrive.setPower(-robotPower);
                    rightFrontDrive.setPower(robotPower);
                    leftBackDrive.setPower(-robotPower);
                    rightBackDrive.setPower(robotPower);
                } else if (direction == STRAFELEFT) {
                    leftFrontDrive.setPower(-robotPower * 0.75);
                    rightFrontDrive.setPower(-robotPower * 0.75);
                    leftBackDrive.setPower(robotPower * 0.75);
                    rightBackDrive.setPower(robotPower * 0.8);
                } else if (direction == STRAFERIGHT) {
                    leftFrontDrive.setPower(robotPower * 0.75);
                    rightFrontDrive.setPower(robotPower * 0.75);
                    leftBackDrive.setPower(-robotPower * 0.75);
                    rightBackDrive.setPower(-robotPower * 0.8);

                }
            }

        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        opMode.telemetry.addData("Distance to X Target", distanceToXTarget/COUNTS_PER_INCH);
        opMode.telemetry.addData("Distance To Y Target", distanceToYTarget/COUNTS_PER_INCH);
        opMode.telemetry.update();


    }
    // Launch Vuforia and TensorFlow
    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
