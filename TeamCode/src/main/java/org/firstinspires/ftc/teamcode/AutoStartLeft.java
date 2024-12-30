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
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;

@Autonomous
public class AutoStartLeft extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        DcMotor fL = hardwareMap.get(DcMotor.class, "FrontLeft");
        DcMotor fR = hardwareMap.get(DcMotor.class, "FrontRight");
        DcMotor bL = hardwareMap.get(DcMotor.class, "BackLeft");
        DcMotor bR = hardwareMap.get(DcMotor.class, "BackRight");
        BHI260IMU IMU = hardwareMap.get(BHI260IMU.class, "imu");
        VoltageSensor voltmeter = hardwareMap.voltageSensor.iterator().next();
        WebcamName myCamera = hardwareMap.get(WebcamName.class, "Webcam 1");

        chassis robot = new chassis(fL, fR, bL, bR, IMU, "IMU", 180, 12, 0, voltmeter, hardwareMap.get(DistanceSensor.class, "frontDistanceSensor"));
        //chassis robot = new chassis(fL, fR, bL, bR, IMU, "IMU", 240, 12, 0, voltmeter, myCamera, new double[]{14.605, 32.385, 0});
        actuators gripper = new actuators(hardwareMap.get(Servo.class, "wrist"), hardwareMap.get(Servo.class, "rotationServo"), hardwareMap.get(Servo.class,
                "beak"), hardwareMap.get(DcMotor.class, "lifterLeft"), hardwareMap.get(DcMotor.class, "lifterRight"), hardwareMap.get(DcMotor.class, "elbow"), hardwareMap.get(DistanceSensor.class,
                "lifterHeightSensor"));
        waitForStart();
        gripper.beakRotateTo(0.65);
        robot.waypointSettings(1, 1, 1,
                .01575, .020125, .0025, 0,
                .012, .006025, 0.0025, 0,
                .35, .035, .0035, 0,
                .024, .03, 10,
                .15, .15, .4);

        robot.toWaypoint(180, 45, 0, 1);
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

        gripper.moveToSpecimenExtractPos();
        ArrayList<double[]> secondExtraction = new ArrayList<double[]>();
        secondExtraction.add(new double[]{315, 90, 0});
        secondExtraction.add(new double[]{298, 20, 0});
        robot.toWaypointBezier(secondExtraction, 2.25, 3);
        gripper.changeClawState();
        gripper.liftTo(1250);

        robot.toWaypoint(210, 45, 0, 1.75);
        gripper.moveToHangInsertPosition();
        robot.toWaypoint(190, 80, 0, 1);

        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
        gripper.hang();
        Thread.sleep(500);
        gripper.changeClawState();

        robot.toWaypoint(170, 65, 0, 0.5);
        gripper.liftTo(0);
        robot.toWaypoint(57, 57, 0, 3);
        //gripper.moveToPickupPositionAuto();
        Thread.sleep(500);
        gripper.changeClawState();

        gripper.moveToTransportPosition();
        robot.toWaypoint(50, 50, -45, 2);
        gripper.moveToHighBucketPosition();
        Thread.sleep(2500);
        robot.toWaypoint(25, 25, -45, 0.75);
        gripper.changeClawState();
        robot.toWaypoint(50, 50, -45, 1);
        gripper.moveToTransportPosition();

        robot.toWaypoint(30, 57, 0, 1.5);
        //gripper.moveToPickupPositionAuto();
        Thread.sleep(1000);
        gripper.changeClawState();

        gripper.moveToTransportPosition();
        robot.toWaypoint(50, 50, -45, 2);
        gripper.moveToHighBucketPosition();
        Thread.sleep(2500);
        robot.toWaypoint(25, 25, -45, 0.75);
        gripper.changeClawState();
        robot.toWaypoint(50, 50, -45, 1);
        gripper.moveToTransportPosition();

        robot.toWaypoint(300, 30, 0,4);

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



    }
}
