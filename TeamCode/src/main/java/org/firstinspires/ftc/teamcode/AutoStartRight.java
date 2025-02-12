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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

            // Move to Hang Insert Position
            gripper.closeBeak();
            gripper.liftTo(0);
            gripper.elbowRotateTo(112, 1);//32
            gripper.wristRotateTo_Pitch(85); //243.33
            gripper.wristRotateTo_Roll(0);

            // Go to Hang 1st specimen
            robot.translateRadDeg(.8,65,.4);
            Thread.sleep(600);

            // Hang 1st specimen
            gripper.liftTo(25);
            Thread.sleep(850);
            gripper.openBeak();

            // Move to Specimen Extract Position
            gripper.liftTo(0);
            gripper.elbowRotateTo(190, 1);
            gripper.wristRotateTo_Pitch(160);
            gripper.wristRotateTo_Roll(0);
            gripper.openBeakWide();

            // First Collection
            robot.translateRadDeg(1,195,1.5);
            robot.translateRadDeg(1,90,1);
            robot.translateRadDeg(1,180,.5);
            robot.translateRadDeg(1,-90,1.1);

            // Second Collection
            robot.translateRadDeg(1,90,1.2);
            robot.translateRadDeg(1,180,.65);
            robot.translateRadDeg(1,-90,.7);

            robot.toWaypoint(330, 25, 0, 1,1.25);

            //Extract 2nd specimen and move to Hang Insert Position
            gripper.closeBeak();
            gripper.liftTo(13);
            gripper.elbowRotateTo(112, 1);//32
            gripper.wristRotateTo_Pitch(85); //243.33
            gripper.wristRotateTo_Roll(0);
            gripper.liftTo(0);


           // Go to Hang 2nd specimen
            ArrayList<double[]> secondHang = new ArrayList<double[]>();
            secondHang.add(new double[]{150, 60, 0});
            secondHang.add(new double[]{150, 75, 0});
            robot.toWaypointBezier(secondHang, 1, 1.5, 1.75);

            robot.toWaypoint(170, 95, 0, 1, 1.5);
            robot.stopChaassis();

            // Hang 2nd specimen
            gripper.liftTo(25);
            Thread.sleep(850);
            gripper.openBeak();

            // Move to Specimen Extract Position
            gripper.liftTo(0);
            gripper.elbowRotateTo(190, 1);
            gripper.wristRotateTo_Pitch(160);
            gripper.wristRotateTo_Roll(0);
            gripper.openBeakWide();

            // Extract 3rd Specimen
            robot.translateRadDeg(1,190,1.85);
            robot.toWaypoint(300, 25, 0, 1,1);
            gripper.closeBeak();
            gripper.liftTo(13);
            gripper.elbowRotateTo(112, 1);//32
            gripper.wristRotateTo_Pitch(85); //243.33
            gripper.wristRotateTo_Roll(0);
            gripper.liftTo(0);

            //Go to Hang 3rd specimen
            ArrayList<double[]> thirdHang = new ArrayList<double[]>();
            thirdHang.add(new double[]{150, 60, 0});
            thirdHang.add(new double[]{180, 75, 0});

            robot.toWaypointBezier(thirdHang, 1, 1.5, 1.7);
            robot.toWaypoint(180, 95, 0, 1, 1.5);
            robot.stopChaassis();

            // Hang 3rd specimen
            gripper.liftTo(25);
            Thread.sleep(850);
            gripper.openBeak();

            // Move to Specimen Extract Position
            gripper.liftTo(0);
            gripper.elbowRotateTo(190, 1);
            gripper.wristRotateTo_Pitch(160);
            gripper.wristRotateTo_Roll(0);
            gripper.openBeakWide();

            // Extract 4th Specimen
            robot.translateRadDeg(1,190,1.85);
            robot.toWaypoint(300, 25, 0, 1,1);
            gripper.closeBeak();
            gripper.liftTo(13);
            gripper.elbowRotateTo(112, 1);//32
            gripper.wristRotateTo_Pitch(85); //243.33
            gripper.wristRotateTo_Roll(0);
            gripper.liftTo(0);

            //Go to Hang 4th specimen
            ArrayList<double[]> fourthHang = new ArrayList<double[]>();
            fourthHang.add(new double[]{150, 60, 0});
            secondHang.add(new double[]{190, 75, 0});
            robot.toWaypointBezier(fourthHang, 1, 1.5, 1.6);
            robot.toWaypoint(190, 95, 0, 1, 1.5);
            robot.stopChaassis();

            // Hang 4th specimen
            gripper.liftTo(25);
            Thread.sleep(850);
            gripper.openBeak();

            // Park
            robot.translateRadDeg(1,195,1.6);
            gripper.resetMotors();

            telemetry.addData("X:", robot.getPosition()[0]);
            telemetry.addData("Y:", robot.getPosition()[1]);
            telemetry.addData("Theta:", robot.getPosition()[2] * 180 / Math.PI);
            telemetry.update();

        }
    }
}
