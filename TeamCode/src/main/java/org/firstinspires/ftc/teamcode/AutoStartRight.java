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
                "lifterHeightSensor"), hardwareMap.get(DistanceSensor.class, "frontDistanceSensor"), hardwareMap.get(TouchSensor.class,"lifterTouchSensor"), hardwareMap.get(TouchSensor.class,"elbowTouchSensor"));


        gripper.resetElbow(); // Safety
        gripper.resetLifters(); // Safety
        gripper.closeBeak();
        waitForStart();

        if (opModeIsActive()) {

            // Move to Hang Insert Position
            //gripper.closeBeak();
            //gripper.liftTo(0);
            gripper.elbowRotateTo(112, 1);//32
            gripper.wristRotateTo_Pitch(85); //243.33
            gripper.wristRotateTo_Roll(0);

            // Go to Hang 1st specimen and hang specimen
            robot.translateRadDeg(1,50, 0.85, false);
            gripper.liftTo(25);
            robot.translateRadDeg(0.3,50, 0.2, true);
            Thread.sleep(700);
            gripper.openBeak();

            // Move to Specimen Extract Position
            gripper.liftTo(0);
            gripper.elbowRotateTo(190, 1);
            gripper.wristRotateTo_Pitch(160);
            gripper.wristRotateTo_Roll(0);
            gripper.openBeakWide();

            // First Collection
            ArrayList<Double> firstCollectionMov1 = new ArrayList<Double>();
            firstCollectionMov1.add(250.0);
            firstCollectionMov1.add(80.0);
            robot.translateRadDeg_Smooth(1, firstCollectionMov1, 2.4, false);

            ArrayList<Double> firstCollectionMov2 = new ArrayList<Double>();
            firstCollectionMov2.add(90.0);
            firstCollectionMov2.add(155.0);
            robot.translateRadDeg_Smooth(1, firstCollectionMov2, .8, false);

            ArrayList<Double> firstCollectionMov3 = new ArrayList<Double>();
            firstCollectionMov3.add(270.0);
            robot.translateRadDeg_Smooth(1, firstCollectionMov3, 1.5, true);

            // Second Collection
            ArrayList<Double> secondCollectionMov1 = new ArrayList<Double>();
            secondCollectionMov1.add(90.0);
            secondCollectionMov1.add(125.0);
            robot.translateRadDeg_Smooth(1, secondCollectionMov1, 1.5, false);

            ArrayList<Double> secondCollectionMov2 = new ArrayList<Double>();
            secondCollectionMov2.add(270.0);
            robot.translateRadDeg_Smooth(1, secondCollectionMov2, .9, false);

            robot.toWaypoint(330, 25, 0, 1,1.75);

            //Extract 2nd specimen and move to Hang Insert Position
            gripper.closeBeak();
            gripper.liftTo(13);
            gripper.elbowRotateTo(112, 1);//32
            gripper.wristRotateTo_Pitch(85); //243.33
            gripper.wristRotateTo_Roll(0);
            gripper.liftTo(0);


            // Go to Hang 2nd specimen and hang specimen
            ArrayList<double[]> secondHang = new ArrayList<double[]>();
            secondHang.add(new double[]{150, 60, 0});
            secondHang.add(new double[]{165, 75, 0});
            robot.toWaypointBezier(secondHang, 1, 1.5, 1.6);
            robot.toWaypoint(165, 95, 0, 1, 1.5);

            gripper.liftTo(25);
            Thread.sleep(825);
            gripper.openBeak();

            // Move to Specimen Extract Position
            gripper.liftTo(0);
            gripper.elbowRotateTo(190, 1);
            gripper.wristRotateTo_Pitch(160);
            gripper.wristRotateTo_Roll(0);
            gripper.openBeakWide();

            // Extract 3rd Specimen
            ArrayList<Double> thirdSpecimenExtractMov1 = new ArrayList<Double>();
            thirdSpecimenExtractMov1.add(205.0);
            robot.translateRadDeg_Smooth(1, thirdSpecimenExtractMov1, .35, false);

            ArrayList<Double> thirdSpecimenExtractMov2 = new ArrayList<Double>();
            thirdSpecimenExtractMov2.add(170.0);
            robot.translateRadDeg_Smooth(1, thirdSpecimenExtractMov2, 1.25, false);


            robot.toWaypoint(300, 25, 0, 1,1.5);
            gripper.closeBeak();
            gripper.liftTo(13);
            gripper.elbowRotateTo(112, 1);//32
            gripper.wristRotateTo_Pitch(85); //243.33
            gripper.wristRotateTo_Roll(0);
            gripper.liftTo(0);

            //Go to Hang 3rd specimen and hang
            ArrayList<double[]> thirdHang = new ArrayList<double[]>();
            thirdHang.add(new double[]{150, 60, 0});
            thirdHang.add(new double[]{175, 75, 0});
            robot.toWaypointBezier(thirdHang, 1, 1.5, 1.6);
            robot.toWaypoint(175, 95, 0, 1, 1.6);

            gripper.liftTo(25);
            Thread.sleep(825);
            gripper.openBeak();

            // Move to Specimen Extract Position
            gripper.liftTo(0);
            gripper.elbowRotateTo(190, 1);
            gripper.wristRotateTo_Pitch(160);
            gripper.wristRotateTo_Roll(0);
            gripper.openBeakWide();


            // Extract 4th Specimen
            ArrayList<Double> fourthSpecimenExtractMov1 = new ArrayList<Double>();
            fourthSpecimenExtractMov1.add(210.0);
            robot.translateRadDeg_Smooth(1, fourthSpecimenExtractMov1, .35, false);

            ArrayList<Double> fourthSpecimenExtractMov2 = new ArrayList<Double>();
            fourthSpecimenExtractMov2.add(170.0);
            robot.translateRadDeg_Smooth(1, fourthSpecimenExtractMov2, 1.25, false);


            robot.toWaypoint(300, 25, 0, 1,1.5);
            gripper.closeBeak();
            gripper.liftTo(13);
            gripper.elbowRotateTo(112, 1);//32
            gripper.wristRotateTo_Pitch(85); //243.33
            gripper.wristRotateTo_Roll(0);
            gripper.liftTo(0);

            //Go to Hang 4th specimen and hang
            ArrayList<double[]> fourthHang = new ArrayList<double[]>();
            fourthHang.add(new double[]{150, 60, 0});
            fourthHang.add(new double[]{180, 75, 0});
            robot.toWaypointBezier(fourthHang, 1, 1.5, 1.75);
            robot.toWaypoint(185, 95, 0, 1, 1.5);

            gripper.liftTo(25);
            Thread.sleep(825);
            gripper.openBeak();

            // Park
            ArrayList<Double> parkMov1 = new ArrayList<Double>();
            parkMov1.add(200.0);

            gripper.liftTo(0);
            gripper.elbowRotateTo(193, 1);
            robot.translateRadDeg_Smooth(1, parkMov1, 2, false);


            telemetry.addData("X:", robot.getPosition()[0]);
            telemetry.addData("Y:", robot.getPosition()[1]);
            telemetry.addData("Theta:", robot.getPosition()[2] * 180 / Math.PI);
            telemetry.update();

        }
    }
}
