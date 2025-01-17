package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import android.util.Size;

import java.lang.*;
import java.util.ArrayList;
import java.util.List;

//import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class chassis{

    // Initialize Instance Variables:
    static private DcMotor fL;
    static private DcMotor fR;
    static private DcMotor bL;
    static private DcMotor bR;
    static private NonEulerianOdometry localizer;
    static private robotIMU imu;
    static private Rev2mDistanceSensor frontDistanceSensor;
    static private ElapsedTime timer;

    static final double rightBias = 1.0;
    static final double leftBias = 1.0;
    static private String angleMode;

    private double waypointToleranceDistX, waypointToleranceDistY, waypointToleranceAng, minGainThresholdX, minGainThresholdY, minGainThresholdTheta, AKpx, BKpx, AKix, BKix, AKdx, BKdx, AKcx, BKcx, AKpy, BKpy, AKiy, BKiy, AKdy, BKdy, AKcy, BKcy, AKpTheta, BKpTheta, AKiTheta, BKiTheta, AKdTheta, BKdTheta, AKcTheta, BKcTheta, waypointAccelLimX, waypointAccelLimY, waypointAccelLimTheta, waypointClampingX, waypointClampingY, waypointClampingTheta, feedForwardThetaBias;
    static final double maxA = 0.3;

    static private VoltageSensor sensor;
    static final double nominalVoltage = 12.15;

    //static private WebcamName camera;
    static private double[] cameraOffsetPose;

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    public chassis(DcMotor FL, DcMotor FR, DcMotor BL, DcMotor BR, BHI260IMU IMU, String thetaMode, double startingX, double startingY, double startingTheta, VoltageSensor voltmeter, DistanceSensor frontDistanceSensor) {
        // Define Timer Objects:
        timer = new ElapsedTime();
        imu = new robotIMU(IMU);
        angleMode = thetaMode;
        localizer = new NonEulerianOdometry(startingX, startingY, startingTheta, BL, BR, FL, imu, angleMode);

        // Define Motor Objects for Chassis:
        fL = FL;
        fR = FR;
        bL = BL;
        bR = BR;

        bL.setDirection(DcMotor.Direction.REVERSE);

        bR.setDirection(DcMotor.Direction.FORWARD);

        fL.setDirection(DcMotor.Direction.REVERSE);

        fR.setDirection(DcMotor.Direction.FORWARD);

        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        bR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        fL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        fR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));


        // extender.setDirection(DcMotor.Direction.FORWARD);

        sensor = voltmeter;

        //camera = cameraName;
        //cameraOffsetPose = cameraNameOffset;
        //cameraOffsetPose[2] *= Math.PI / 180;

        //initAprilTag();

        this.frontDistanceSensor = (Rev2mDistanceSensor) frontDistanceSensor;
    }

    /*private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())

                .setOutputUnits(DistanceUnit.CM, AngleUnit.RADIANS)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(1196.94, 1196.94, 804.849, 411.195)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera
        builder.setCamera(camera);


        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1600, 896));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()*/

    public boolean eqWT(double val1, double val2, double e){
        return Math.abs(val1 - val2) <= e;
    }


    //translation at a constant speed given x and y components of power
    //postiive x is to the right and positive y is forward
    public void translateXY(double powX, double powY, double time) {
        timer.reset();
        while (timer.seconds() <= time) {
            //vector of the front right and bottom left wheels: a
            //vector of the bottom right and front left wheels: b
            double a = (powX+powY)*(Math.pow(2, -0.5));
            double b = (-powX+powY)*(Math.pow(2, -0.5));
            fL.setPower(a * leftBias);
            fR.setPower(b * rightBias);
            bL.setPower(b * leftBias);
            bR.setPower(a * rightBias);
            localizer.updateOdometry();
        }
    }


    public void translateRadDeg(double radius, double theta, double time) {
        //theta is in degrees (north is 90 deg and positive degrees are clockwise), radius is in units of powX and powY
        timer.reset();
        theta = theta; theta *= Math.PI/180;
        double powX = - radius * Math.cos(theta);
        double powY = radius * Math.sin(theta);
        while (timer.seconds() <= time) {
            //vector of the front right and bottom left wheels: a
            //vector of the bottom right and front left wheels: b
            double a = (powX+powY)*(Math.pow(2, -0.5));
            double b = (-powX+powY)*(Math.pow(2, -0.5));
            fL.setPower(a * leftBias);
            fR.setPower(b * rightBias);
            bL.setPower(b * leftBias);
            bR.setPower(a * rightBias);
            localizer.updateOdometry();
        }
    }

    public void throttleTranslateRadDeg(double radius, double theta, double s, double time) {
        //theta is in degrees (north is 90 deg and positive degrees are clockwise), radius is in units of powX and powY
        timer.reset();
        theta = theta; theta *= Math.PI/180;
        double powX = -radius * Math.cos(theta);
        double powY = radius * Math.sin(theta);
        double mp;
        while (timer.seconds() <= time) {
            mp = -1 * Math.abs(maxA * (2.0 * timer.seconds() - time * s ) / 4.0) - Math.abs(maxA * (2.0 * timer.seconds() - 2.0 * time + time * s) / 4.0) + (maxA * time / 2.0);
            //vector of the front right and bottom left wheels: a
            //vector of the bottom right and front left wheels: b
            double a = (powX+powY)*(Math.pow(2, -0.5));
            double b = (-powX+powY)*(Math.pow(2, -0.5));
            fL.setPower(mp * b * leftBias);
            fR.setPower(mp * a * rightBias);
            bL.setPower(mp * a * leftBias);
            bR.setPower(mp * b * rightBias);
            localizer.updateOdometry();
        }
    }

    public void tankTurn(double powLeft, double powRight, double time){
        timer.reset();
        while(timer.seconds() <= time){
            fL.setPower(powLeft * leftBias);
            bL.setPower(powLeft * leftBias);
            fR.setPower(powRight * rightBias);
            bR.setPower(powRight * rightBias);
            localizer.updateOdometry();
        }
    }

    public void localize(double x, double y, double theta){
        localizer = new NonEulerianOdometry(x, y, theta, bL, bR, fL, imu, angleMode);
    }

    public void waypointSettings(double toleranceDistX, double toleranceDistY, double toleranceAng, double minGainThresholdX, double minGainThresholdY, double minGainThresholdTheta, double AKpx, double BKpx, double AKix, double BKix, double AKdx, double BKdx, double AKcx, double BKcx, double AKpy, double BKpy, double AKiy, double BKiy, double AKdy, double BKdy, double AKcy, double BKcy, double AKpTheta, double BKpTheta, double AKiTheta, double BKiTheta, double AKdTheta, double BKdTheta, double AKcTheta, double BKcTheta, double accelLimX, double accelLimY, double accelLimTheta, double clampingX, double clampingY, double clampingTheta, double feedForwardThetaBias){
        waypointToleranceDistX = toleranceDistX;
        waypointToleranceDistY = toleranceDistY;
        waypointToleranceAng = toleranceAng * Math.PI / 180;

        this.AKpx = AKpx;
        this.BKpx = BKpx;
        this.AKix = AKix;
        this.BKix = BKix;
        this.AKdx = AKdx;
        this.BKdx = BKdx;
        this.AKcx = AKcx;
        this.BKcx = BKcx;

        this.AKpy = AKpy;
        this.BKpy = BKpy;
        this.AKiy = AKiy;
        this.BKiy = BKiy;
        this.AKdy = AKdy;
        this.BKdy = BKdy;
        this.AKcy = AKcy;
        this.BKcy = BKcy;

        this.AKpTheta = AKpTheta;
        this.BKpTheta = BKpTheta;
        this.AKiTheta = AKiTheta;
        this.BKiTheta = BKiTheta;
        this.AKdTheta = AKdTheta;
        this.BKdTheta = BKdTheta;
        this.AKcTheta = AKcTheta;
        this.BKcTheta = BKcTheta;

        this.minGainThresholdX = minGainThresholdX;
        this.minGainThresholdY = minGainThresholdY;
        this.minGainThresholdTheta = minGainThresholdTheta;

        waypointAccelLimX = accelLimX;
        waypointAccelLimY = accelLimY;
        waypointAccelLimTheta = accelLimTheta;
        waypointClampingX = clampingX;
        waypointClampingY = clampingY;
        waypointClampingTheta = clampingTheta;

        this.feedForwardThetaBias = feedForwardThetaBias;
    }

    private double getBatteryVoltage(){
        return sensor.getVoltage();
    }


    public double[] toWaypoint(double waypointTargetX, double waypointTargetY, double waypointTargetTheta, int controllerOption, double timeout){

        setWaypointController(controllerOption);

        waypointTargetTheta *= Math.PI / 180.0;
        double Px = 0;
        double Ix = 0;
        double Dx = 0;
        double Py = 0;
        double Iy = 0;
        double Dy = 0;
        double Ptheta = 0;
        double Itheta = 0;
        double Dtheta = 0;
        double previousTime;
        double currentTime = timer.seconds();
        double startTime = timer.seconds();
        double dt;
        double globalCorrectionX = 0;
        double globalCorrectionY = 0;
        double globalCorrectionTheta = 0;
        double previousLocalCorrectionX = 0;
        double previousLocalCorrectionY = 0;
        double previousGlobalCorrectionTheta = 0;
        double previousErrX;
        double previousErrY;
        double previousErrTheta;

        double deltaX = waypointTargetX - getPosition()[0];
        double deltaY = waypointTargetY - getPosition()[1];
        double deltaTheta = waypointTargetTheta - getPosition()[2];

        RobotLog.dd("Global deltaX: ", deltaX + "");
        RobotLog.dd("Global deltaY: ", deltaY + "");

        //For cases where  deltaTheta is not 0, assume the robot is moving in the final angle to the target destination.
        //Practically, this means using a "local" deltaX and deltaY
        double localDeltaX = deltaX * Math.cos(-waypointTargetTheta) - deltaY * Math.sin(-waypointTargetTheta);
        double localDeltaY = deltaX * Math.sin(-waypointTargetTheta) + deltaY * Math.cos(-waypointTargetTheta);
        deltaX = localDeltaX;
        deltaY = localDeltaY;

        RobotLog.dd("Local deltaX: ", deltaX + "");
        RobotLog.dd("Local deltaY: ", deltaY + "");

        RobotLog.dd("MinGainThresholdXValue:", minGainThresholdX + "");

        if(Math.abs(deltaX) < Math.abs(minGainThresholdX)){
            deltaX = minGainThresholdX;
            RobotLog.dd("MinGainThresholdXState:", "true");
        }

        RobotLog.dd("DeltaXValue:", deltaX + "");

        if(Math.abs(deltaY) < Math.abs(minGainThresholdY)){
            deltaY = minGainThresholdY;
        }

        if(Math.abs(deltaTheta) < Math.abs(minGainThresholdTheta * Math.PI / 180)){
            deltaTheta = minGainThresholdTheta * Math.PI / 180;
        }

        double waypointKpx = AKpx * Math.pow(Math.abs(deltaX), BKpx);
        double waypointKix = AKix * Math.pow(Math.abs(deltaX), BKix);
        double waypointKdx = AKdx * Math.pow(Math.abs(deltaX), BKdx);
        double waypointKcx = AKcx * Math.pow(Math.abs(deltaX), BKcx);

        double waypointKpy = AKpy * Math.pow(Math.abs(deltaY), BKpy);
        double waypointKiy = AKiy * Math.pow(Math.abs(deltaY), BKiy);
        double waypointKdy = AKdy * Math.pow(Math.abs(deltaY), BKdy);
        double waypointKcy = AKcy * Math.pow(Math.abs(deltaY), BKcy);

        double waypointKpTheta = AKpTheta * Math.pow(Math.abs(deltaTheta * 180 / Math.PI), BKpTheta);

        RobotLog.dd("InitialKpTheta:", waypointKpTheta + "");
        RobotLog.dd("deltaTheta:", deltaTheta + "");

        double waypointKiTheta = AKiTheta * Math.pow(Math.abs(deltaTheta * 180 / Math.PI), BKiTheta);
        double waypointKdTheta = AKdTheta * Math.pow(Math.abs(deltaTheta * 180 / Math.PI), BKdTheta);
        double waypointKcTheta = AKcTheta * Math.pow(Math.abs(deltaTheta * 180 / Math.PI), BKcTheta);

        //If Turning, Overwrite Translational Feed-Forwards:
        if(deltaTheta != 0){
            waypointKcx = feedForwardThetaBias;
            waypointKcy = feedForwardThetaBias;
        }

        double Kcx = waypointKcx;
        double Kcy = waypointKcy;
        double Kctheta = waypointKcTheta;

        RobotLog.dd("Kpx:", waypointKpx + "");
        RobotLog.dd("Kix:", waypointKix + "");
        RobotLog.dd("Kdx:", waypointKdx + "");
        RobotLog.dd("Kcx:", waypointKcx + "");

        RobotLog.dd("Kpy:", waypointKpy + "");
        RobotLog.dd("Kiy:", waypointKiy + "");
        RobotLog.dd("Kdy:", waypointKdy + "");
        RobotLog.dd("Kcy:", waypointKcy + "");

        RobotLog.dd("KpTheta:", waypointKpTheta + "");
        RobotLog.dd("KiTheta:", waypointKiTheta + "");
        RobotLog.dd("KdTheta:", waypointKdTheta + "");
        RobotLog.dd("KcTheta:", waypointKcTheta + "");

        // Note: Error Here Is Not Exactly the Error, But the Correction Signal (Target - Actual)
        double errX = 0;
        double errY = 0;
        double errTheta = 0;
        double localCorrectionX = 0.0;
        double localCorrectionY = 0.0;

        double localErrX;
        double localErrY;

        double Imax = 0;

        // Read current odometry position
        localizer.updateOdometry();
        double[] currentPos = localizer.getPosition();
        double currentX = currentPos[0];
        double currentY = currentPos[1];
        double currentTheta = currentPos[2];

        RobotLog.dd("chassis", "Starting drive to waypoint");
        // PID control to waypoint
        while((!eqWT(currentX, waypointTargetX, waypointToleranceDistX) || !eqWT(currentY, waypointTargetY, waypointToleranceDistY) || !eqWT(currentTheta, waypointTargetTheta, waypointToleranceAng)) && ((timer.seconds() - startTime) < timeout)){
            // Read current odometry position
            localizer.updateOdometry();
            currentPos = localizer.getPosition();
            currentX = currentPos[0];
            currentY = currentPos[1];
            currentTheta = currentPos[2];
            previousTime = currentTime;
            currentTime = timer.seconds();
            dt = currentTime - previousTime;

            // Update Proportional, Integral, and Derivative Errors
            previousErrX = errX;
            previousErrY = errY;
            previousErrTheta = errTheta;

            // Calculate Global Error:
            errX = waypointTargetX - currentX;
            errY = waypointTargetY - currentY;
            errTheta = waypointTargetTheta - currentTheta;

            // Convert to Local Error:
            localErrX = - errY * Math.sin(- currentTheta) + errX * Math.cos(- currentTheta);
            localErrY = errY * Math.cos( - currentTheta) + errX * Math.sin(- currentTheta);
            errX = localErrX;
            errY = localErrY;

            RobotLog.dd("LocalErrX: ", localErrX + "");
            RobotLog.dd("LocalErrY: ", localErrY + "");


            Px = errX;
            Py = errY;
            Ptheta = errTheta;

            if(Px > 0){
                Kcx = Math.abs(waypointKcx);
            }
            else if(Px == 0){
                Kcx = 0;
            }
            else{
                Kcx = -Math.abs(waypointKcx);
            }

            if(Py > 0){
                Kcy = Math.abs(waypointKcy);
            }
            else if(Py == 0){
                Kcy = 0;
            }
            else{
                Kcy = -Math.abs(waypointKcy);
            }

            if(Ptheta > 0){
                Kctheta = Math.abs(waypointKcTheta);
            }
            else if(Ptheta == 0){
                Kctheta = 0;
            }
            else{
                Kctheta = -Math.abs(waypointKcTheta);
            }

            Ix += dt * (errX + previousErrX) / 2.0;
            Iy += dt * (errY + previousErrY) / 2.0;
            Itheta += dt * (errTheta + previousErrTheta) / 2.0;

            if(Ix * waypointKix > Math.abs(waypointClampingX)){
                Ix = Math.abs(waypointClampingX) / waypointKix;
            }
            else if(Ix * waypointKix < -1 * Math.abs(waypointClampingX)){
                Ix = -1 * Math.abs(waypointClampingX) / waypointKix;
            }

            if(Iy * waypointKiy > Math.abs(waypointClampingY)){
                Iy = Math.abs(waypointClampingY) / waypointKiy;
            }
            else if(Iy * waypointKiy < - Math.abs(waypointClampingY)){
                Iy = -1 * Math.abs(waypointClampingY) / waypointKiy;
            }

            if(Itheta * waypointKiTheta > Math.abs(waypointClampingTheta)){
                Itheta = Math.abs(waypointClampingTheta) / (waypointKiTheta);
            }
            else if(Itheta * waypointKiTheta < - Math.abs(waypointClampingTheta)){
                Itheta = -1 * Math.abs(waypointClampingTheta) / (waypointKiTheta);
            }

            Dx = (errX - previousErrX) / dt;
            Dy = (errY - previousErrY) / dt;
            Dtheta = (errTheta - previousErrTheta) / dt;

            // Calculate correction:

            /*globalCorrectionX = (waypointKpx * Px + waypointKix * Ix + waypointKdx * Dx + Kcx);
            globalCorrectionY = (waypointKpy * Py + waypointKiy * Iy + waypointKdy * Dy + Kcy);
            globalCorrectionTheta = (waypointKpTheta * Ptheta + waypointKiTheta * Itheta + waypointKdTheta * Dtheta + Kctheta);

            localCorrectionY = globalCorrectionY * Math.cos(currentTheta) - globalCorrectionX * Math.sin(currentTheta);
            localCorrectionX = globalCorrectionY * Math.sin(currentTheta) + globalCorrectionX * Math.cos(currentTheta);

            localCorrectionX *= 1;
            localCorrectionY *= 1;*/

            localCorrectionX = (waypointKpx * Px + waypointKix * Ix + waypointKdx * Dx + Kcx);
            localCorrectionY = (waypointKpy * Py + waypointKiy * Iy + waypointKdy * Dy + Kcy);
            globalCorrectionTheta = (waypointKpTheta * Ptheta + waypointKiTheta * Itheta + waypointKdTheta * Dtheta + Kctheta);


            // Rate Limiter: Check if correction is within accelLim of previous correction to avoid slip
            if(!eqWT(localCorrectionX, previousLocalCorrectionX, waypointAccelLimX)){
                if(localCorrectionX > previousLocalCorrectionX){
                    localCorrectionX = previousLocalCorrectionX + waypointAccelLimX;
                }
                else{
                    localCorrectionX = previousLocalCorrectionX - waypointAccelLimX;
                }
            }
            if(!eqWT(localCorrectionY, previousLocalCorrectionY, waypointAccelLimY)){
                if(localCorrectionY > previousLocalCorrectionY){
                    localCorrectionY = previousLocalCorrectionY + waypointAccelLimY;
                }
                else{
                    localCorrectionY = previousLocalCorrectionY - waypointAccelLimY;
                }
            }
            if(!eqWT(globalCorrectionTheta, previousGlobalCorrectionTheta, waypointAccelLimTheta)){
                if(globalCorrectionTheta > previousGlobalCorrectionTheta){
                    globalCorrectionTheta = previousGlobalCorrectionTheta + waypointAccelLimTheta;
                }
                else{
                    globalCorrectionTheta = previousGlobalCorrectionTheta - waypointAccelLimTheta;
                }
            }

            // Controller Toggle:

            /*if(eqWT(currentX, waypointTargetX, waypointToleranceDist)){
                localCorrectionX = 0;
            }
            if(eqWT(currentY, waypointTargetY, waypointToleranceDist)){
                localCorrectionY = 0;
            }
            if(eqWT(currentTheta, waypointTargetTheta, waypointToleranceAng)){
                globalCorrectionTheta = 0;
            }*/

            if(Iy * waypointKiy > Imax){
                Imax = Iy * waypointKiy;
            }

            // Voltage Regulator:
            double voltage = getBatteryVoltage();
            double voltageScale = voltage / nominalVoltage;

            // Actuate Correction
            double denominator = voltageScale * Math.max(Math.abs(localCorrectionX) + Math.abs(localCorrectionY) + Math.abs(globalCorrectionTheta), 1.0);
            double a = (localCorrectionX  + localCorrectionY);
            double b = (-localCorrectionX + localCorrectionY);
            // Randomize Motor Acutation
            double randomNum = (int)(Math.random() * 4) + 1;
            if(randomNum == 1){
                fR.setPower((b * rightBias + globalCorrectionTheta) / denominator);
                fL.setPower((a * leftBias - globalCorrectionTheta) / denominator);
                bR.setPower((a * rightBias + globalCorrectionTheta) / denominator);
                bL.setPower((b * leftBias - globalCorrectionTheta) / denominator);
            }
            else if(randomNum == 2){
                fL.setPower((a * leftBias - globalCorrectionTheta) / denominator);
                bR.setPower((a * rightBias + globalCorrectionTheta) / denominator);
                bL.setPower((b * leftBias - globalCorrectionTheta) / denominator);
                fR.setPower((b * rightBias + globalCorrectionTheta) / denominator);
            }
            else if(randomNum == 3){
                bR.setPower((a * rightBias + globalCorrectionTheta) / denominator);
                bL.setPower((b * leftBias - globalCorrectionTheta) / denominator);
                fR.setPower((b * rightBias + globalCorrectionTheta) / denominator);
                fL.setPower((a * leftBias - globalCorrectionTheta) / denominator);
            }
            else{
                bL.setPower((b * leftBias - globalCorrectionTheta) / denominator);
                fR.setPower((b * rightBias + globalCorrectionTheta) / denominator);
                fL.setPower((a * leftBias - globalCorrectionTheta) / denominator);
                bR.setPower((a * rightBias + globalCorrectionTheta) / denominator);
            }

            RobotLog.dd("X:", getPosition()[0] + "");
            RobotLog.dd("Y:", getPosition()[1] + "");
            RobotLog.dd("Theta:", getPosition()[2] * 180 / Math.PI + "");


            previousLocalCorrectionX = localCorrectionX;
            previousLocalCorrectionY = localCorrectionY;
            previousGlobalCorrectionTheta = globalCorrectionTheta;

            //return new double[]{localCorrectionX, localCorrectionY, globalCorrectionX, globalCorrectionY, a, b};
        }

        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
        return new double[]{Imax, getBatteryVoltage(), 0.0, 0.0};
    }

    /*public void deflectTo(double deflectionXtarget, double deflectionYtarget, double deflectionThetatarget, double XYTol, double AngTol, double finalXtarget, double finalYtarget, double finalThetatarget, double timeout){
        bL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.FLOAT));
        bR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.FLOAT));
        fL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.FLOAT));
        fR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.FLOAT));
        double prevTolDist = waypointToleranceDist;
        double prevTolAng = waypointToleranceAng;
        double prevKp = waypointKp;
        double prevKi = waypointKi;
        double prevKd = waypointKd;
        double prevKcx = waypointKcx;
        double prevKcy = waypointKcy;
        double prevKctheta = waypointKctheta;
        double prevThetaWeight = waypointThetaWeight;
        double prevAccelLimXY = waypointAccelLimXY;
        double prevAccelLimTheta = waypointAccelLimTheta;
        double prevClamping = waypointClamping;
        waypointSettings(XYTol, AngTol,.027,0.0027, .0027, .00375, .00375, .00375, 15, .1, .1, .1);
        toWaypoint(deflectionXtarget, deflectionYtarget, deflectionThetatarget, timeout);
        waypointSettings(prevTolDist, prevTolAng, prevKp, prevKi, prevKd, prevKcx, prevKcy, prevKctheta, prevThetaWeight, prevAccelLimXY, prevAccelLimTheta, prevClamping);
        toWaypoint(finalXtarget, finalYtarget, finalThetatarget, timeout);
        bL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        bR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        fL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        fR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
    }*/

    public void gyroTurn(double powLeft, double powRight, double targetAngle){
        targetAngle *= Math.PI / 180.0;
        double[] pos = localizer.getPosition();
        if(targetAngle > pos[2]){
            while(pos[2] < targetAngle){
                fL.setPower(powLeft * leftBias);
                bL.setPower(powLeft * leftBias);
                fR.setPower(powRight * rightBias);
                bR.setPower(powRight * rightBias);
                localizer.updateOdometry();
                pos = localizer.getPosition();
            }
        }
        else if(targetAngle < pos[2]){
            while(pos[2] > targetAngle){
                fL.setPower(powLeft * leftBias);
                bL.setPower(powLeft * leftBias);
                fR.setPower(powRight * rightBias);
                bR.setPower(powRight * rightBias);
                localizer.updateOdometry();
                pos = localizer.getPosition();
            }
        }
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }

    public DcMotor getMotor(String MotorName){
        if(MotorName.equals("fL")){
            return fL;
        }
        else if(MotorName.equals("fR")){
            return fR;
        }
        else if(MotorName.equals("bL")){
            return bL;
        }
        else{
            return bR;
        }
    }

    public void updateOdometry(){

        localizer.updateOdometry();
    }

    public double[] getPosition(){
        localizer.updateOdometry();
        return localizer.getPosition();
    }

    public double getAngle(){
        return imu.updateAngle()[0];
    }

    private int factorial(int n){
        int product = 1;
        if(n == 0){
            return 1;
        }
        for(int i = 0; i < n; i++){
            product *= (i + 1);
        }
        return product;
    }


    private double Brez(ArrayList<Double> X, double t){
        double cord = 0;
        int sz = X.size();
        int n = sz - 1;
        for(int i = 0; i <= n; i++){
            cord = cord + (factorial(n) / (factorial(i) * factorial(n - i))) * Math.pow(1 - t, n - i) * X.get(i) * Math.pow(t, i);
        }
        return cord;
    }

    // NOTE: High Chance for Computational Failure With More Than 10 Bezier Points; Guarenteed Failure With 14 or More Bezier Points;
    public double[] toWaypointBezier(ArrayList<double[]> targetPoints,  int controllerOption, double runtime, double timeout){

        setBezierWaypointController(controllerOption);

        for(int i = 0; i < targetPoints.size(); i++){
            double[] point = targetPoints.get(i);
            point[2] *= Math.PI / 180.0;
            targetPoints.set(i, point);
        }

        ArrayList<Double> xPoints = new ArrayList<Double>();
        ArrayList<Double> yPoints = new ArrayList<Double>();
        ArrayList<Double> thetaPoints = new ArrayList<Double>();
        for(double[] point : targetPoints){
            xPoints.add(point[0]);
            yPoints.add(point[1]);
            thetaPoints.add(point[2]);
        }

        //Riemann Sum w/ Trapazoidal Rule and Left-Endpoints to Approximate X, Y, and Theta Path Lengths
        double deltaX = 0;
        double deltaY = 0;
        double deltaTheta = 0;
        double Global_dX = 0;
        double Global_dY = 0;
        double Global_dTheta = 0;
        double Local_dX = 0;
        double Local_dY = 0;
        ArrayList<Double> deltaXofT = new ArrayList<Double>();
        ArrayList<Double> deltaYofT = new ArrayList<Double>();
        ArrayList<Double> deltaThetaofT = new ArrayList<Double>();

        for(double t = 0; t < runtime; t += 0.01){

            //Global_dX = (Brez(xPoints, t + 0.01) -  Brez(xPoints, t));
            //Global_dY = (Brez(yPoints, t + 0.01) -  Brez(yPoints, t));

            Local_dX = (Brez(xPoints, t + 0.01) -  Brez(xPoints, t));
            Local_dY = (Brez(yPoints, t + 0.01) -  Brez(yPoints, t));
            Global_dTheta = (Brez(thetaPoints, t + 0.01) - Brez(thetaPoints, t));

            /*//For cases where  deltaTheta is not 0, assume the robot is moving in the final angle to the target destination.
            //Practically, this means using a "local" deltaX and deltaY
            Local_dX = Global_dX * Math.cos(-Global_dTheta) - Global_dY * Math.sin(-Global_dTheta);
            Local_dY = Global_dX * Math.sin(-Global_dTheta) + Global_dY * Math.cos(-Global_dTheta);*/

            deltaX += Local_dX;
            deltaY += Local_dY;
            deltaTheta += Global_dTheta;

            deltaXofT.add(deltaX);
            deltaYofT.add(deltaY);
            deltaThetaofT.add(deltaTheta);
        }


        // Identify extrema for integrals:
        double minDeltaX = 0;
        double maxDeltaX = 0;

        double minDeltaY = 0;
        double maxDeltaY = 0;

        double minDeltaTheta = 0;
        double maxDeltaTheta = 0;


        for(int i = 0; i < deltaXofT.size(); i++){

            if(deltaXofT.get(i) < minDeltaX) {
                minDeltaX = deltaXofT.get(i);
            }
            if(deltaXofT.get(i) > maxDeltaX){
                maxDeltaX = deltaXofT.get(i);
            }

            if(deltaYofT.get(i) < minDeltaY){
                minDeltaY = deltaYofT.get(i);
            }
            if(deltaYofT.get(i) > maxDeltaY){
                maxDeltaY = deltaYofT.get(i);
            }

            if(deltaThetaofT.get(i) < minDeltaTheta){
                minDeltaTheta = deltaThetaofT.get(i);
            }
            if(deltaThetaofT.get(i) > maxDeltaTheta){
                maxDeltaTheta = deltaThetaofT.get(i);
            }
        }

        RobotLog.dd("minX: ", minDeltaX + "");
        RobotLog.dd("maxX: ", maxDeltaX + "");

        // Use the maximum extrema of the integral of x, y, and theta (local frame of reference) to compute gains:
        if(Math.abs(minDeltaX) >= Math.abs(maxDeltaX)){
            deltaX =  Math.abs(minDeltaX);
        }
        else{
            deltaX = Math.abs(maxDeltaX);
        }
        if(Math.abs(minDeltaY) >= Math.abs(maxDeltaY)){
            deltaY = Math.abs(minDeltaY);
        }
        else{
            deltaY = Math.abs(maxDeltaY);
        }
        if(Math.abs(minDeltaTheta) >= Math.abs(maxDeltaTheta)){
            deltaTheta = Math.abs(minDeltaTheta);
        }
        else{
            deltaTheta = Math.abs(maxDeltaTheta);
        }

        RobotLog.dd("Brez. Integral DeltaX: ", deltaX + "");
        RobotLog.dd("Brez. Integral DeltaY: ", deltaY + "");
        RobotLog.dd("Brez. Integral DeltaTheta: ", deltaTheta + "");
        /*for(double targetX : xPoints){
            if(Math.abs(targetX - getPosition()[0]) > deltaX){
                deltaX = Math.abs(targetX - getPosition()[0]);
            }
        }

        double deltaY = 0;
        for(double targetY : yPoints){
            if(Math.abs(targetY - getPosition()[1]) > deltaY){
                deltaY = Math.abs(targetY - getPosition()[1]);
            }
        }

        double deltaTheta = 0;
        for(double targetTheta : thetaPoints){
            if(Math.abs(targetTheta - getPosition()[2]) > deltaTheta){
                deltaTheta = Math.abs(targetTheta - getPosition()[2]);


            }
        }*/

        if(Math.abs(deltaX) < Math.abs(minGainThresholdX)){
            deltaX = minGainThresholdX;
        }

        if(Math.abs(deltaY) < Math.abs(minGainThresholdY)){
            deltaY = minGainThresholdY;
        }

        if(Math.abs(deltaTheta) < Math.abs(minGainThresholdTheta)){
            deltaTheta = minGainThresholdTheta;
        }

        double Px = 0;
        double Ix = 0;
        double Dx = 0;
        double Py = 0;
        double Iy = 0;
        double Dy = 0;
        double Ptheta = 0;
        double Itheta = 0;
        double Dtheta = 0;
        double previousTime;
        double currentTime = timer.seconds();
        double startTime = timer.seconds();
        double dt;
        double globalCorrectionTheta = 0;
        double previousLocalCorrectionX = 0;
        double previousLocalCorrectionY = 0;
        double previousGlobalCorrectionTheta = 0;
        double previousErrX;
        double previousErrY;
        double previousErrTheta;

        RobotLog.dd("AKpx", AKpx + " ");
        RobotLog.dd("AKix", AKix + " ");
        RobotLog.dd("AKdx", AKdx + " ");
        RobotLog.dd("AKpy", AKpy + " ");
        RobotLog.dd("AKiy", AKiy + " ");
        RobotLog.dd("AKdy", AKdy + " ");
        RobotLog.dd("AKpTheta", AKpTheta + " ");
        RobotLog.dd("AKiTheta", AKiTheta + " ");
        RobotLog.dd("AKdTheta", AKdTheta + " ");



        double waypointKpx = AKpx * Math.pow(Math.abs(deltaX), BKpx);
        double waypointKix = AKix * Math.pow(Math.abs(deltaX), BKix);
        double waypointKdx = AKdx * Math.pow(Math.abs(deltaX), BKdx);
        double waypointKcx = AKcx * Math.pow(Math.abs(deltaX), BKcx);

        double waypointKpy = AKpy * Math.pow(Math.abs(deltaY), BKpy);
        double waypointKiy = AKiy * Math.pow(Math.abs(deltaY), BKiy);
        double waypointKdy = AKdy * Math.pow(Math.abs(deltaY), BKdy);
        double waypointKcy = AKcy * Math.pow(Math.abs(deltaY), BKcy);

        double waypointKpTheta = AKpTheta * Math.pow(Math.abs(deltaTheta * 180 / Math.PI), BKpTheta);

        RobotLog.dd("InitialKpTheta:", waypointKpTheta + "");
        RobotLog.dd("deltaTheta:", deltaTheta + "");

        double waypointKiTheta = AKiTheta * Math.pow(Math.abs(deltaTheta * 180 / Math.PI), BKiTheta);
        double waypointKdTheta = AKdTheta * Math.pow(Math.abs(deltaTheta * 180 / Math.PI), BKdTheta);
        double waypointKcTheta = AKcTheta * Math.pow(Math.abs(deltaTheta * 180 / Math.PI), BKcTheta);

        RobotLog.dd("waypointKpx", waypointKpx + " ");
        RobotLog.dd("waypointKix", waypointKix + " ");
        RobotLog.dd("waypointKdx", waypointKdx + " ");
        RobotLog.dd("waypointKpy", waypointKpy + " ");
        RobotLog.dd("waypointKiy", waypointKiy + " ");
        RobotLog.dd("waypointKdy", waypointKdy + " ");
        RobotLog.dd("waypointKpTheta", waypointKpTheta + " ");
        RobotLog.dd("waypointKiTheta", waypointKiTheta + " ");
        RobotLog.dd("waypointKdTheta", waypointKdTheta + " ");

        double Kcx = waypointKcx;
        double Kcy = waypointKcy;
        double Kctheta = waypointKcTheta;

        // Note: Error Here Is Not Exactly the Error, But the Correction Signal (Target - Actual)
        double errX = 0;
        double errY = 0;
        double errTheta = 0;
        double localCorrectionX = 0.0;
        double localCorrectionY = 0.0;

        double localErrX;
        double localErrY;

        double Imax = 0;

        double t = 0;


        // Read current odometry position
        localizer.updateOdometry();
        double[] currentPos = localizer.getPosition();
        double currentX = currentPos[0];
        double currentY = currentPos[1];
        double currentTheta = currentPos[2];

        // PID control to waypoint
        while((!eqWT(currentX, targetPoints.get(targetPoints.size() - 1)[0], waypointToleranceDistX) || !eqWT(currentY, targetPoints.get(targetPoints.size() - 1)[1], waypointToleranceDistY) || !eqWT(currentTheta, targetPoints.get(targetPoints.size() - 1)[2], waypointToleranceAng) || (t != 1)) && ((timer.seconds() - startTime) < timeout)){
            // Read current odometry position
            localizer.updateOdometry();
            currentPos = localizer.getPosition();
            currentX = currentPos[0];
            currentY = currentPos[1];
            currentTheta = currentPos[2];
            previousTime = currentTime;
            currentTime = timer.seconds();
            dt = currentTime - previousTime;

            // Update Proportional, Integral, and Derivative Errors
            previousErrX = errX;
            previousErrY = errY;
            previousErrTheta = errTheta;

            // Get Bezier Target
            t = (timer.seconds() - startTime) / runtime;
            if(t > 1){
                t = 1;
            }
            double waypointTargetX = Brez(xPoints, t);
            double waypointTargetY = Brez(yPoints, t);
            double waypointTargetTheta = Brez(thetaPoints, t);

            // Calculate Global Error:
            errX = waypointTargetX - currentX;
            errY = waypointTargetY - currentY;
            errTheta = waypointTargetTheta - currentTheta;

            // Convert to Local Error:
            localErrX = - errY * Math.sin(- currentTheta) + errX * Math.cos(- currentTheta);
            localErrY = errY * Math.cos( - currentTheta) + errX * Math.sin(- currentTheta);
            errX = localErrX;
            errY = localErrY;

            Px = errX;
            Py = errY;
            Ptheta = errTheta;



            if(Px > 0){
                Kcx = Math.abs(waypointKcx);
            }
            else if(Px == 0){
                Kcx = 0;
            }
            else{
                Kcx = -Math.abs(waypointKcx);
            }

            if(Py > 0){
                Kcy = Math.abs(waypointKcy);
            }
            else if(Py == 0){
                Kcy = 0;
            }
            else{
                Kcy = -Math.abs(waypointKcy);
            }

            if(Ptheta > 0){
                Kctheta = Math.abs(waypointKcTheta);
            }
            else if(Ptheta == 0){
                Kctheta = 0;
            }
            else{
                Kctheta = -Math.abs(waypointKcTheta);
            }

            Ix += dt * (errX + previousErrX) / 2.0;
            Iy += dt * (errY + previousErrY) / 2.0;
            Itheta += dt * (errTheta + previousErrTheta) / 2.0;

            if(Ix * waypointKix > Math.abs(waypointClampingX)){
                Ix = Math.abs(waypointClampingX) / waypointKix;
            }
            else if(Ix * waypointKix < -1 * Math.abs(waypointClampingX)){
                Ix = -1 * Math.abs(waypointClampingX) / waypointKix;
            }

            if(Iy * waypointKiy > Math.abs(waypointClampingY)){
                Iy = Math.abs(waypointClampingY) / waypointKiy;
            }
            else if(Iy * waypointKiy < - Math.abs(waypointClampingY)){
                Iy = -1 * Math.abs(waypointClampingY) / waypointKiy;
            }

            if(Itheta * waypointKiTheta > Math.abs(waypointClampingTheta)){
                Itheta = Math.abs(waypointClampingTheta) / (waypointKiTheta);
            }
            else if(Itheta * waypointKiTheta < - Math.abs(waypointClampingTheta)){
                Itheta = -1 * Math.abs(waypointClampingTheta) / (waypointKiTheta);
            }

            Dx = (errX - previousErrX) / dt;
            Dy = (errY - previousErrY) / dt;
            Dtheta = (errTheta - previousErrTheta) / dt;

            // Calculate correction:

            localCorrectionX = (waypointKpx * Px + waypointKix * Ix + waypointKdx * Dx + Kcx);
            localCorrectionY = (waypointKpy * Py + waypointKiy * Iy + waypointKdy * Dy + Kcy);
            globalCorrectionTheta = (waypointKpTheta * Ptheta + waypointKiTheta * Itheta + waypointKdTheta * Dtheta + Kctheta);

            // Rate Limiter: Check if correction is within accelLim of previous correction to avoid slip
            if(!eqWT(localCorrectionX, previousLocalCorrectionX, waypointAccelLimX)){
                if(localCorrectionX > previousLocalCorrectionX){
                    localCorrectionX = previousLocalCorrectionX + waypointAccelLimX;
                }
                else{
                    localCorrectionX = previousLocalCorrectionX - waypointAccelLimX;
                }
            }
            if(!eqWT(localCorrectionY, previousLocalCorrectionY, waypointAccelLimY)){
                if(localCorrectionY > previousLocalCorrectionY){
                    localCorrectionY = previousLocalCorrectionY + waypointAccelLimY;
                }
                else{
                    localCorrectionY = previousLocalCorrectionY - waypointAccelLimY;
                }
            }
            if(!eqWT(globalCorrectionTheta, previousGlobalCorrectionTheta, waypointAccelLimTheta)){
                if(globalCorrectionTheta > previousGlobalCorrectionTheta){
                    globalCorrectionTheta = previousGlobalCorrectionTheta + waypointAccelLimTheta;
                }
                else{
                    globalCorrectionTheta = previousGlobalCorrectionTheta - waypointAccelLimTheta;
                }
            }

            // Controller Toggle:

            /*if(eqWT(currentX, waypointTargetX, waypointToleranceDist)){
                localCorrectionX = 0;
            }
            if(eqWT(currentY, waypointTargetY, waypointToleranceDist)){
                localCorrectionY = 0;
            }
            if(eqWT(currentTheta, waypointTargetTheta, waypointToleranceAng)){
                globalCorrectionTheta = 0;
            }*/

            if(Iy * waypointKiy > Imax){
                Imax = Iy * waypointKiy;
            }

            // Voltage Regulator:
            double voltage = getBatteryVoltage();
            double voltageScale = voltage / nominalVoltage;

            // Actuate Correction
            double denominator = voltageScale * Math.max(Math.abs(localCorrectionX) + Math.abs(localCorrectionY) + Math.abs(globalCorrectionTheta), 1.0);
            double a = (localCorrectionX  + localCorrectionY);
            double b = (-localCorrectionX + localCorrectionY);
            // Randomize Motor Acutation
            double randomNum = (int)(Math.random() * 4) + 1;
            if(randomNum == 1){
                fR.setPower((b * rightBias + globalCorrectionTheta) / denominator);
                fL.setPower((a * leftBias - globalCorrectionTheta) / denominator);
                bR.setPower((a * rightBias + globalCorrectionTheta) / denominator);
                bL.setPower((b * leftBias - globalCorrectionTheta) / denominator);
            }
            else if(randomNum == 2){
                fL.setPower((a * leftBias - globalCorrectionTheta) / denominator);
                bR.setPower((a * rightBias + globalCorrectionTheta) / denominator);
                bL.setPower((b * leftBias - globalCorrectionTheta) / denominator);
                fR.setPower((b * rightBias + globalCorrectionTheta) / denominator);
            }
            else if(randomNum == 3){
                bR.setPower((a * rightBias + globalCorrectionTheta) / denominator);
                bL.setPower((b * leftBias - globalCorrectionTheta) / denominator);
                fR.setPower((b * rightBias + globalCorrectionTheta) / denominator);
                fL.setPower((a * leftBias - globalCorrectionTheta) / denominator);
            }
            else{
                bL.setPower((b * leftBias - globalCorrectionTheta) / denominator);
                fR.setPower((b * rightBias + globalCorrectionTheta) / denominator);
                fL.setPower((a * leftBias - globalCorrectionTheta) / denominator);
                bR.setPower((a * rightBias + globalCorrectionTheta) / denominator);
            }

            previousLocalCorrectionX = localCorrectionX;
            previousLocalCorrectionY = localCorrectionY;
            previousGlobalCorrectionTheta = globalCorrectionTheta;

            //return new double[]{localCorrectionX, localCorrectionY, globalCorrectionX, globalCorrectionY, a, b};
        }
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
        return new double[]{Imax, getBatteryVoltage(), 0.0, 0.0};
    }

    /*public void deflectTo(double deflectionXtarget, double deflectionYtarget, double deflectionThetatarget, double XYTol, double AngTol, double finalXtarget, double finalYtarget, double finalThetatarget, double timeout){
        bL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.FLOAT));
        bR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.FLOAT));
        fL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.FLOAT));
        fR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.FLOAT));
        double prevTolDist = waypointToleranceDist;
        double prevTolAng = waypointToleranceAng;
        double prevKp = waypointKp;
        double prevKi = waypointKi;
        double prevKd = waypointKd;
        double prevKcx = waypointKcx;
        double prevKcy = waypointKcy;
        double prevKctheta = waypointKctheta;
        double prevThetaWeight = waypointThetaWeight;
        double prevAccelLimXY = waypointAccelLimXY;
        double prevAccelLimTheta = waypointAccelLimTheta;
        double prevClamping = waypointClamping;
        waypointSettings(XYTol, AngTol,.027,0.0027, .0027, .00375, .00375, .00375, 15, .1, .1, .1);
        toWaypoint(deflectionXtarget, deflectionYtarget, deflectionThetatarget, timeout);
        waypointSettings(prevTolDist, prevTolAng, prevKp, prevKi, prevKd, prevKcx, prevKcy, prevKctheta, prevThetaWeight, prevAccelLimXY, prevAccelLimTheta, prevClamping);
        toWaypoint(finalXtarget, finalYtarget, finalThetatarget, timeout);
        bL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        bR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        fL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        fR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
    }*/

    /*public void visualLocalization(double[] aprilTagPosition, int aprilTagID, int filterSize){

        aprilTagPosition[2] *= Math.PI / 180;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        double xDist = 0;
        double yDist = 0;
        double thetaDiff = 0;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == aprilTagID) {
                double xSum = 0;
                double ySum = 0;
                double thetaSum = 0;
                for(int i = 0; i < filterSize; i++){
                    xSum += detection.ftcPose.x;
                    ySum += detection.ftcPose.y;
                    thetaSum += detection.ftcPose.yaw;
                }
                xDist = xSum / filterSize;
                yDist = ySum / filterSize;
                thetaDiff = thetaSum / filterSize;
            }
        }   // end for() loop

        double thetaPos = aprilTagPosition[2] - thetaDiff + Math.PI;

        double xPos = aprilTagPosition[0] + (Math.sqrt(Math.pow(xDist, 2) + Math.pow(yDist, 2)) * Math.cos(thetaPos - (Math.PI / 2)));
        double yPos = aprilTagPosition[1] + (Math.sqrt(Math.pow(xDist, 2) + Math.pow(yDist, 2)) * Math.sin(thetaPos - (Math.PI / 2)));
        thetaPos -= cameraOffsetPose[2];
        double beta = 0;
        if(cameraOffsetPose[0] > 0){
            beta = Math.atan(cameraOffsetPose[1] / cameraOffsetPose[0]);
        }
        else if(cameraOffsetPose[0] < 0){
            beta = Math.atan(cameraOffsetPose[1]) + Math.PI;
        }
        else if(cameraOffsetPose[0] == 0 && cameraOffsetPose[1] > 0){
            beta = Math.PI / 2;
        }
        else{
            beta = - Math.PI / 2;
        }
        xPos += (Math.sqrt(Math.pow(cameraOffsetPose[0], 2) + Math.pow(cameraOffsetPose[1], 2)) * Math.cos(thetaPos + beta - Math.PI));
        yPos += (Math.sqrt(Math.pow(cameraOffsetPose[0], 2) + Math.pow(cameraOffsetPose[1], 2)) * Math.sin(thetaPos + beta - Math.PI));

        localize(xPos, yPos, thetaPos * 180 / Math.PI);

    }*/

    public void ultrasonicLocalization(double[] objectPos){
        double Xobj = objectPos[0];
        double Yobj = objectPos[1];

        double ThetaR = localizer.getPosition()[2];
        double Xr = localizer.getPosition()[0];
        double Yr = localizer.getPosition()[1];

        double Xupdated;
        double Yupdated;

        double xPrime = Xr * Math.cos(ThetaR) + Yr * Math.sin(ThetaR);
        double yPrime = Yobj * Math.cos(ThetaR) - Xobj * Math.sin(ThetaR) - frontDistanceSensor.getDistance(DistanceUnit.CM);

        Xupdated = xPrime * Math.cos(ThetaR) - yPrime * Math.sin(ThetaR);
        Yupdated = xPrime * Math.sin(ThetaR) + yPrime * Math.cos(ThetaR);

        localize(Xupdated, Yupdated, ThetaR);
    }

    public void setWaypointController(int controllerOption){

        if(controllerOption == 1) { //controllerOption 1 = Higher accelLims
            waypointSettings(1, 1, 1.5,
                    30, 30, 5,
                    0.0872169527, -0.4891123746,
                    4.060598e-5, 0,
                    0.0002125, 0,
                    0, 0,
                    .117312536, -.5899072879,
                    4.060598e-23, 7.598320549,
                    0.002, 0,
                    0.0, 0,
                    2.147257771, -0.3554874788,
                    11.49861303, -1.283011678,
                    .00, 0,
                    0.0, 0,
                    .024, .03, 0.0375,
                    1, 1, 1,
                    .1);
        }
        else { // controllerOption 2 = Lower accelLims
            waypointSettings(1, 1, 1.5,
                    30, 30, 5,
                    0.0872169527, -0.4891123746,
                    4.060598e-5, 0,
                    0.0002125, 0,
                    0, 0,
                    .117312536, -.5899072879,
                    4.060598e-23, 7.598320549,
                    0.002, 0,
                    0.0, 0,
                    2.147257771, -0.3554874788,
                    11.49861303, -1.283011678,
                    .00, 0,
                    0.0, 0,
                    .024 * 0.5, .03 * 0.5, 0.0375 * 0.5,
                    1, 1, 1,
                    .1);
        }
    }

    public void setBezierWaypointController(int controllerOption){

        if(controllerOption == 1) { // ControllerOption 1 = Higher AccelLims
            waypointSettings(1, 1, 1.5,
                    30, 30, 5,
                    0.01575, 0,
                    .030691, 0,
                    .0025, 0,
                    0, 0,
                    .012, 0,
                    .007049, 0,
                    .0025, 0,
                    0.0, 0,
                    .35, 0,
                    .035, 0,
                    .004375, 0,
                    0.0, 0,
                    .024 * 3, .15, 0.028125,
                    .15, .3, .3,
                    .1);
        }
        else { // ControllerOption 2 = Lower AccelLims
            waypointSettings(1, 1, 1.5,
                    30, 30, 5,
                    0.01575, 0,
                    .030691, 0,
                    .0025, 0,
                    0, 0,
                    .012, 0,
                    .007049, 0,
                    .0025, 0,
                    0.0, 0,
                    .35, 0,
                    .035, 0,
                    .004375, 0,
                    0.0, 0,
                    .024 * 3 * 0.25, .15 * 0.25, 0.028125 * 0.25,
                    .15, .3, .3,
                    .1);
        }
    }

    public void stopChaassis(){
        //Stop all motors
        fL.setPower(0); fR.setPower(0); bL.setPower(0); bR.setPower(0);
    }

}
