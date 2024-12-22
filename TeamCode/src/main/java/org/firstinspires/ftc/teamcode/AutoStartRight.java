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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

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
        WebcamName myCamera = hardwareMap.get(WebcamName.class, "Webcam 1");

        chassis robot = new chassis(fL, fR, bL, bR, IMU, "IMU", 180, 12, 0, voltmeter, myCamera, new double[]{14.605, 32.385, 0});
        //chassis robot = new chassis(fL, fR, bL, bR, IMU, "IMU", 240, 12, 0, voltmeter, myCamera, new double[]{14.605, 32.385, 0});
        claw gripper = new claw(hardwareMap.get(Servo.class, "wrist"), hardwareMap.get(Servo.class, "leftTalon"), hardwareMap.get(Servo.class, "rightTalon"), hardwareMap.get(Servo.class,
                "beak"), hardwareMap.get(DcMotor.class, "lifterLeft"), hardwareMap.get(DcMotor.class, "lifterRight"), hardwareMap.get(DcMotor.class, "elbow"));
        waitForStart();
        robot.waypointSettings(1, 1, 1,
                .01575, .020125, .0025, 0,
                .012, .006025, 0.0025, 0,
                .35, .035, .0035, 0,
                .024, .03, 10,
                .15, .15, .4);
        robot.toWaypoint(180, 72,0, 2);
        robot.toWaypoint(300, 42,0, 3);
        robot.toWaypoint(180, 72,0, 3);
        robot.toWaypoint(300, 42,0, 3);
        robot.toWaypoint(180, 72,0, 3);

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
