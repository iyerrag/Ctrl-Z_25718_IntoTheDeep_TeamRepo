package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;

@Autonomous
public class AutoStartRight extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        DcMotor fL = hardwareMap.get(DcMotor.class, "FrontLeft");
        DcMotor fR = hardwareMap.get(DcMotor.class, "FrontRight");
        DcMotor bL = hardwareMap.get(DcMotor.class, "BackLeft");
        DcMotor bR = hardwareMap.get(DcMotor.class, "BackRight");
        BHI260IMU IMU = hardwareMap.get(BHI260IMU.class, "imu");
        VoltageSensor voltmeter = hardwareMap.voltageSensor.iterator().next();
        //WebcamName myCamera = hardwareMap.get(WebcamName.class, "Webcam 1");

        chassis robot = new chassis(fL, fR, bL, bR, IMU, "IMU", 203, 12, 0, voltmeter, hardwareMap.get(DistanceSensor.class, "frontDistanceSensor"));
        //chassis robot = new chassis(fL, fR, bL, bR, IMU, "IMU", 240, 12, 0, voltmeter, myCamera, new double[]{14.605, 32.385, 0});
        actuators gripper = new actuators(hardwareMap.get(Servo.class, "wrist"), hardwareMap.get(Servo.class, "rotationServo"), hardwareMap.get(Servo.class,
                "beak"), hardwareMap.get(DcMotor.class, "lifterLeft"), hardwareMap.get(DcMotor.class, "lifterRight"), hardwareMap.get(DcMotor.class, "elbow"), hardwareMap.get(DistanceSensor.class,
                "lifterHeightSensor"), hardwareMap.get(TouchSensor.class,"lifterTouchSensor"));

        /*robot.waypointSettings(1, 1, 1.5,
                                20, 20, 5,
                                /*.1425529672, -0.5733673083,
                                 .01775,0,
                                0.0,0,
                                0.000,0,
                                0,0,
                                .117312536, -.5899072879,
                                1.093174e-21,7.057233538,
                                .002,0,
                                0.0,0,
                                2.147257771, -0.3554874788,
                                11.49861303, -1.283011678,
                                .00,0,
                                0.0,0,
                                .024, .03, 0.0375,
                                1, 1, 1 );*/

        waitForStart();

        if (opModeIsActive()) {

            /*ArrayList<double[]> firstHighBucketDropMov = new ArrayList<double[]>();
            firstHighBucketDropMov.add(new double[]{180, 120, -45});
            firstHighBucketDropMov.add(new double[]{120, 60, -45});
            firstHighBucketDropMov.add(new double[]{60, 60, -45});
            firstHighBucketDropMov.add(new double[]{60, 30, -45});

            robot.toWaypointBezier(firstHighBucketDropMov, 2.25, 5);*/
            /*robot.toWaypoint(30, 30, -45, 3.5);
            telemetry.addData("X:", robot.getPosition()[0]);
            telemetry.addData("Y:", robot.getPosition()[1]);
            telemetry.addData("Theta:", robot.getPosition()[2] * 180 / Math.PI);
            telemetry.update();

            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);
            bR.setPower(0);
            Thread.sleep(10000);*/



            //gripper.initializePosition();
            robot.toWaypoint(180, 45, 0, 1, 1);
            gripper.moveToHangInsertPosition();
            robot.toWaypoint(175, 95, 0, 1, 2);
            robot.stopChaassis();
            gripper.hang();
            gripper.moveToSpecimenExtractPos();

            ArrayList<double[]> firstCollectionAndExtraction = new ArrayList<double[]>();
            firstCollectionAndExtraction.add(new double[]{240, -300, 0});
            firstCollectionAndExtraction.add(new double[]{320,450,0});
            //firstCollectionAndExtraction.add(new double[]{300, 120, 0});
            firstCollectionAndExtraction.add(new double[]{300, 45, 0});
            robot.toWaypointBezier(firstCollectionAndExtraction, 1, 3.5, 3.75);
            robot.toWaypoint(300, 25, 0, 1, 1.5);
            gripper.extract();

            robot.toWaypoint(180, 45, 0, 1, 2);
            gripper.moveToHangInsertPosition();
            robot.toWaypoint(180, 100, 0, 1, 1.5);
            robot.stopChaassis();
            gripper.hang();
            gripper.moveToSpecimenExtractPos();


            ArrayList<double[]> secondExtraction = new ArrayList<double[]>();
            secondExtraction.add(new double[]{180, 45, 0});
            secondExtraction.add(new double[]{330, 120, 0});
            secondExtraction.add(new double[]{300, 45, 0});
            robot.toWaypointBezier(secondExtraction, 1, 2, 2.25);
            robot.toWaypoint(300, 25, 0, 1,1);
            gripper.extract();

            robot.toWaypoint(180, 45, 0, 1, 2);
            gripper.moveToHangInsertPosition();
            robot.toWaypoint(190, 100, 0, 1, 1.5);
            gripper.hang();

            robot.toWaypoint(300, 25, 0, 1, 2.5);
            gripper.resetMotors();



            /*
            robot.toWaypoint(210, 50, 0, 0.5);
            gripper.moveToTransportPosition();

            robot.toWaypoint(275, 45, 0, 1.5);
            robot.toWaypoint(275, 150, 0, 2);

            ArrayList<double[]> sampleExtractMov1 = new ArrayList<double[]>();
            sampleExtractMov1.add(new double[]{340, 130, 0});
            sampleExtractMov1.add(new double[]{285, 30, 0});

            ArrayList<double[]> sampleExtractMov2 = new ArrayList<double[]>();
            sampleExtractMov2.add(new double[]{200, 300, 0});
            sampleExtractMov2.add(new double[]{400, 130, 0});
            sampleExtractMov2.add(new double[]{285, 30, 0});

            ArrayList<double[]> sampleExtractMov3 = new ArrayList<double[]>();
            sampleExtractMov3.add(new double[]{230, 300, 0});
            sampleExtractMov3.add(new double[]{390, 30, 0});

            robot.toWaypointBezier(sampleExtractMov1, 1.25, 2.0);
            robot.toWaypointBezier(sampleExtractMov2, 3.5, 3.75);
            robot.toWaypointBezier(sampleExtractMov3, 3.5, 3.75);

            /*robot.toWaypoint(303.5, 57, 0, 3);
            gripper.moveToPickupPositionAuto();
            Thread.sleep(500);
            gripper.changeClawState();
            gripper.moveToObservationDropOffPos();
            robot.toWaypoint(330, 43, 0, 1);
            gripper.changeClawState();

            robot.toWaypoint(330, 57, 0, 1.5);
            gripper.moveToPickupPositionAuto();
            Thread.sleep(1000);
            gripper.changeClawState();
            gripper.moveToObservationDropOffPos();
            robot.toWaypoint(360, 0, 0, 1);
            gripper.changeClawState();

            /*robot.toWaypoint(180, 45, 0, 1);
            gripper.moveToHangInsertPosition();
            robot.toWaypoint(180, 80, 0, 1.2);

            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);
            bR.setPower(0);
            gripper.hang();
            Thread.sleep(500);
            gripper.changeClawState();
            robot.toWaypoint(180, 45, 0, .75);

            gripper.moveToTransportPosition();
            robot.toWaypoint(303, 55, 0, 3);
            gripper.moveToPickupPosition();
            Thread.sleep(500);
            gripper.changeClawState();
            gripper.moveToObservationDropOffPos();
            robot.toWaypoint(330, 43, 0, 1);
            gripper.changeClawState();

            robot.toWaypoint(330, 55, 0, 1.5);
            gripper.moveToPickupPosition();
            Thread.sleep(1000);
            gripper.changeClawState();
            gripper.moveToObservationDropOffPos();
            robot.toWaypoint(330, 43, 0, 1);
            gripper.changeClawState();

            ArrayList<double[]> secondExtraction = new ArrayList<double[]>();
            secondExtraction.add(new double[]{315, 134, 0});
            secondExtraction.add(new double[]{300, 20, 0});
            robot.toWaypointBezier(secondExtraction, 2, 3);
            gripper.changeClawState();
            gripper.liftTo(1000);

            robot.toWaypoint(200, 45, 0, 3);
            gripper.moveToHangInsertPosition();
            robot.toWaypoint(200, 80, 0, 1);

            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);
            bR.setPower(0);
            gripper.hang();
            Thread.sleep(1250);
            gripper.changeClawState();

            robot.toWaypoint(180, 45, 0, 1);
            gripper.moveToTransportPosition();
            robot.toWaypoint(300, 30, 0, 2);*/


           /* gripper.moveToTransportPosition();
            gripper.liftTo(3700);
            robot.waypointSettings(1.5, 1.5, 1,
                    .0145, .008125, 0, 0,
                    .012, .002025, 0, 0,
                    .125, .1425, 0, 0,
                    .024, .03, 10,
                    .075, .03, .05);
            ArrayList<double[]> speciminHangMov = new ArrayList<double[]>();
            speciminHangMov.add(new double[]{180, 60, 0});
            speciminHangMov.add(new double[]{180, 94, 0});
            robot.toWaypointBezier(speciminHangMov, 2.5, 2.75);
            //robot.toWaypoint(180, 60, 0, 10);
            //robot.toWaypoint(240, 60,0, 2);
            gripper.liftTo(2500);
            Thread.sleep(1000);
            ArrayList<double[]> awayFromSpeciminHangMov = new ArrayList<double[]>();
            awayFromSpeciminHangMov.add(new double[]{210, 60, 0});
            awayFromSpeciminHangMov.add(new double[]{280, 60, 0});
            gripper.liftTo(0);
            robot.toWaypointBezier(awayFromSpeciminHangMov, 1, 2.25);
            robot.toWaypoint(270,150,0,2);
            ArrayList<double[]> collectionMov1 = new ArrayList<double[]>();
            collectionMov1.add(new double[]{310, 150, 0});
            collectionMov1.add(new double[]{300, 30, 0});
            robot.toWaypointBezier(collectionMov1,1.0, 2.25);*/

            telemetry.addData("X:", robot.getPosition()[0]);
            telemetry.addData("Y:", robot.getPosition()[1]);
            telemetry.addData("Theta:", robot.getPosition()[2] * 180 / Math.PI);
            telemetry.update();

            Thread.sleep(10000);
        }
    }
}
