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
        actuators gripper = new actuators(hardwareMap.get(Servo.class, "lwServo"), hardwareMap.get(Servo.class, "rwServo"), hardwareMap.get(Servo.class, "wrollServo"), hardwareMap.get(Servo.class,
                "beak"), hardwareMap.get(DcMotor.class, "lifterLeft"), hardwareMap.get(DcMotor.class, "lifterRight"), hardwareMap.get(DcMotor.class, "elbow"), hardwareMap.get(DistanceSensor.class,
                "lifterHeightSensor"), hardwareMap.get(TouchSensor.class,"lifterTouchSensor"), hardwareMap.get(TouchSensor.class,"elbowTouchSensor"));

        waitForStart();

        if (opModeIsActive()) {

           //gripper.initializePosition();
            robot.toWaypoint(180, 45, 0, 1, 1);
            gripper.moveToHangInsertPosition();
            robot.toWaypoint(175, 95, 0, 1, 2);
            robot.stopChaassis();
            gripper.hangRelease();
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
            gripper.hangRelease();
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
            gripper.hangRelease();

            robot.toWaypoint(300, 25, 0, 1, 2.5);
            gripper.resetMotors();

            telemetry.addData("X:", robot.getPosition()[0]);
            telemetry.addData("Y:", robot.getPosition()[1]);
            telemetry.addData("Theta:", robot.getPosition()[2] * 180 / Math.PI);
            telemetry.update();

            Thread.sleep(10000);
        }
    }
}
