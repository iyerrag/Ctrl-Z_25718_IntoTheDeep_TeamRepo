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
            gripper.elbowRotateTo(112, 1);//32
            gripper.wristRotateTo_Pitch(85); //243.33
            gripper.wristRotateTo_Roll(0);

            // Go to Hang 1st specimen and hang specimen
            robot.translateRadDeg(1,50, 0.7, false);
            robot.toWaypoint(160, 100, 0, 1, .6);

            gripper.liftTo(25);
            Thread.sleep(700);
            gripper.openBeak();

            // Move to Specimen Extract Position
            gripper.liftTo(0);
            gripper.elbowRotateTo(190, 1);
            gripper.wristRotateTo_Pitch(160);
            gripper.wristRotateTo_Roll(0);
            gripper.openBeakWide();

            // First Collection
            ArrayList<double[]> initializeCollectionPos = new ArrayList<double[]>();
            initializeCollectionPos.add(new double[]{180, 60, 0});
            initializeCollectionPos.add(new double[]{307, 60, 0});
            initializeCollectionPos.add(new double[]{280, 170, 0});
            robot.toWaypointBezier(initializeCollectionPos, 1, 2.25, 2.5);

            ArrayList<double[]> firstSampleCollection = new ArrayList<double[]>();
            firstSampleCollection.add(new double[]{315,170,0});
            firstSampleCollection.add(new double[]{300, 55, 0});
            robot.toWaypointBezier(firstSampleCollection, 1, 1, 2);

            // Second Collection

            ArrayList<double[]> secondSampleCollection = new ArrayList<double[]>();
            secondSampleCollection.add(new double[]{275, 170, 0});
            secondSampleCollection.add(new double[]{360, 190, 0});
            secondSampleCollection.add(new double[]{330, 80, 0});
            secondSampleCollection.add(new double[]{300, 25, 0});
            robot.toWaypointBezier(secondSampleCollection, 1, 3.25, 3.75);

            gripper.closeBeak();
            Thread.sleep(300);
            gripper.liftTo(11.25);
            gripper.elbowRotateTo(112, 1);//32
            gripper.wristRotateTo_Pitch(85); //243.33
            gripper.wristRotateTo_Roll(0);
            gripper.liftTo(0);

            // Go to Hang 2nd specimen
            ArrayList<double[]> secondHang = new ArrayList<double[]>();
            secondHang.add(new double[]{155, 60, 0});
            secondHang.add(new double[]{157, 100, 0});
            robot.toWaypointBezier(secondHang, 1, 2.25, 2.5);

            gripper.liftTo(27);
            Thread.sleep(820);
            gripper.openBeak();

            // Move to Specimen Extract Position
            gripper.liftTo(0);
            gripper.elbowRotateTo(190, 1);
            gripper.wristRotateTo_Pitch(160);
            gripper.wristRotateTo_Roll(0);
            gripper.openBeakWide();

            // Extract 3rd Specimen
            ArrayList<double[]> thirdSpecimenExtractMov = new ArrayList<double[]>();
            thirdSpecimenExtractMov.add(new double[]{210, 60, 0});
            thirdSpecimenExtractMov.add(new double[]{240, 90, 0});
            thirdSpecimenExtractMov.add(new double[]{350, 120, 0});
            thirdSpecimenExtractMov.add(new double[]{290, 60, 0});
            thirdSpecimenExtractMov.add(new double[]{300, 25, 0});
            robot.toWaypointBezier(thirdSpecimenExtractMov, 1,2.5, 3);

            gripper.closeBeak();
            Thread.sleep(300);
            gripper.liftTo(11.25);
            gripper.elbowRotateTo(112, 1);//32
            gripper.wristRotateTo_Pitch(85); //243.33
            gripper.wristRotateTo_Roll(0);
            gripper.liftTo(0);

            //Go to Hang 3rd specimen and hang
            ArrayList<double[]> thirdHang = new ArrayList<double[]>();
            thirdHang.add(new double[]{157, 60, 0});
            thirdHang.add(new double[]{167, 100, 0});
            robot.toWaypointBezier(thirdHang, 1, 2.5, 2.75);

            gripper.liftTo(27);
            Thread.sleep(820);
            gripper.openBeak();

            // Move to Specimen Extract Position
            gripper.liftTo(0);
            gripper.elbowRotateTo(190, 1);
            gripper.wristRotateTo_Pitch(160);
            gripper.wristRotateTo_Roll(0);
            gripper.openBeakWide();


            // Extract 4th Specimen
            ArrayList<double[]> fourthSpecimenExtractMov = new ArrayList<double[]>();
            fourthSpecimenExtractMov.add(new double[]{210, 60, 0});
            fourthSpecimenExtractMov.add(new double[]{240, 90, 0});
            fourthSpecimenExtractMov.add(new double[]{350, 120, 0});
            fourthSpecimenExtractMov.add(new double[]{290, 60, 0});
            fourthSpecimenExtractMov.add(new double[]{300, 25, 0});
            robot.toWaypointBezier(fourthSpecimenExtractMov, 1,2.5, 3);

            gripper.closeBeak();
            Thread.sleep(300);
            gripper.liftTo(11.25);
            gripper.elbowRotateTo(112, 1);//32
            gripper.wristRotateTo_Pitch(85); //243.33
            gripper.wristRotateTo_Roll(0);
            gripper.liftTo(0);

            //Go to Hang 4th specimen and hang
            ArrayList<double[]> fourthHang = new ArrayList<double[]>();
            fourthHang.add(new double[]{170, 60, 0});
            fourthHang.add(new double[]{175, 100, 0});
            robot.toWaypointBezier(fourthHang, 1, 2.5, 2.75);

            gripper.liftTo(27);
            Thread.sleep(900);
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
