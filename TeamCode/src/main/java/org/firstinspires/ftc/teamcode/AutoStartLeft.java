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

        chassis robot = new chassis(fL, fR, bL, bR, IMU, "IMU", 180, 12, 0, voltmeter, hardwareMap.get(DistanceSensor.class, "frontDistanceSensor"));
        actuators gripper = new actuators(hardwareMap.get(Servo.class, "wrist"), hardwareMap.get(Servo.class, "rotationServo"), hardwareMap.get(Servo.class,
                "beak"), hardwareMap.get(DcMotor.class, "lifterLeft"), hardwareMap.get(DcMotor.class, "lifterRight"), hardwareMap.get(DcMotor.class, "elbow"), hardwareMap.get(DistanceSensor.class,
                "lifterHeightSensor"));
        robot.waypointSettings(1, 1, 1,
                .01575, .020125, .0025, 0,
                .012, .006025, 0.0025, 0,
                .35, .035, .0035, 0,
                .012, .015, 5,
                .15, .15, .4);

        waitForStart();
        gripper.initializePosition();

        gripper.liftTo(75);
        gripper.elbowRotateTo(180, 1);
        gripper.wristRotateTo(150);
        gripper.rotateTo(-180);
        ArrayList<double[]> firstHighBucketDropMov = new ArrayList<double[]>();
        firstHighBucketDropMov.add(new double[]{120, 60, 0});
        firstHighBucketDropMov.add(new double[]{30, 30, -45});
        robot.toWaypointBezier(firstHighBucketDropMov, 3, 3.25);
        while(!gripper.eqWT(gripper.getElbowAngle(), 180, 1)){}
        while(!gripper.eqWT(gripper.getWristAngle(), 150, 1)){}
        gripper.declareHighBucketStatusTrue();
        Thread.sleep(300);
        gripper.openBeak();
        Thread.sleep(200);

        gripper.liftTo(0);
        gripper.elbowRotateTo(-5, 1);
        gripper.wristRotateTo(95);
        gripper.rotateTo(-180);
        robot.toWaypoint(56, 60, 0, 2.25);
        gripper.resetLifters();
        gripper.declareHighBucketStatusFalse();

        gripper.closeBeak();
        Thread.sleep(200);
        gripper.liftTo(75);
        Thread.sleep(1750);
        gripper.elbowRotateTo(180, 1);
        gripper.wristRotateTo(130);
        gripper.rotateTo(-180);
        robot.toWaypoint(37, 37, -45, 2.25);
        gripper.declareHighBucketStatusTrue();
        Thread.sleep(300);
        gripper.openBeak();
        Thread.sleep(200);

        gripper.liftTo(0);
        gripper.elbowRotateTo(-5, 1);
        gripper.wristRotateTo(95);
        gripper.rotateTo(-180);
        robot.toWaypoint(28, 60, 0, 2.25);
        gripper.resetLifters();
        gripper.declareHighBucketStatusFalse();

        robot.waypointSettings(1, 1, 1,
                .01575, .020125, .0025, 0,
                .012, .006025, 0.0025, 0,
                .35, .035, .0035, 0,
                .024, .03, 10,
                .15, .15, .4);

    }
}
