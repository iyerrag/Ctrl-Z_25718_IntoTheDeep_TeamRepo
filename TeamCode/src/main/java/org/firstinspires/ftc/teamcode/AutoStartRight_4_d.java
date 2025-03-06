package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.ArrayList;

@Autonomous
public class AutoStartRight_4_d extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        DcMotor fL = hardwareMap.get(DcMotor.class, "FrontLeft");
        DcMotor fR = hardwareMap.get(DcMotor.class, "FrontRight");
        DcMotor bL = hardwareMap.get(DcMotor.class, "BackLeft");
        DcMotor bR = hardwareMap.get(DcMotor.class, "BackRight");
        BHI260IMU IMU = hardwareMap.get(BHI260IMU.class, "imu");
        VoltageSensor voltmeter = hardwareMap.voltageSensor.iterator().next();
        //WebcamName myCamera = hardwareMap.get(WebcamName.class, "Webcam 1");

        chassis robot = new chassis(fL, fR, bL, bR, IMU, "IMU", 270, 12, 0, voltmeter);
        //chassis robot = new chassis(fL, fR, bL, bR, IMU, "IMU", 240, 12, 0, voltmeter, myCamera, new double[]{14.605, 32.385, 0});
        actuators gripper = new actuators(hardwareMap.get(Servo.class, "lwServo"), hardwareMap.get(Servo.class, "rwServo"), hardwareMap.get(Servo.class, "wrollServo"), hardwareMap.get(Servo.class,
                "beak"), hardwareMap.get(Servo.class, "sweeperServo"), hardwareMap.get(DcMotor.class, "lifterLeft"), hardwareMap.get(DcMotor.class, "lifterRight"), hardwareMap.get(DcMotor.class, "elbow"), hardwareMap.get(TouchSensor.class,"lifterTouchSensor"), hardwareMap.get(TouchSensor.class,"elbowTouchSensor"));


        gripper.resetElbow(); // Safety
        gripper.resetLifters(); // Safety
        gripper.closeBeak();
        gripper.sweeperSetup();

        waitForStart();

        if (opModeIsActive()) {

            // Defend first
            ArrayList<double[]> defencePos = new ArrayList<double[]>();
            defencePos.add(new double[]{270, 60, 0});
            defencePos.add(new double[]{270, 120, 0});
            defencePos.add(new double[]{270, 150, 0});
            defencePos.add(new double[]{333, 177, 0});
            robot.toWaypointBezier(defencePos, 1, 1.5, 1.7);

            gripper.sweeperOpen();
            Thread.sleep(250);


            robot.translateRadDeg(1,0, 0.47,false);


            // First Collection
            ArrayList<double[]> firstCollection = new ArrayList<double[]>();
            firstCollection.add(new double[]{300, 180, 0});
            firstCollection.add(new double[]{300,150, 0});
            firstCollection.add(new double[]{300, 40, 0});
            robot.toWaypointBezier(firstCollection, 1, 1.35, 1.35);
            gripper.sweeperClose();

            // Second Collection

            ArrayList<double[]> secondSamplePos = new ArrayList<double[]>();
            secondSamplePos.add(new double[]{270, 60, 0});
            secondSamplePos.add(new double[]{300, 120, 0});
            secondSamplePos.add(new double[]{330, 150, 0});
            robot.toWaypointBezier(secondSamplePos, 1, 1.6, 1.75);

            // Move to Specimen Extract Position
            gripper.liftTo(0);
            gripper.elbowRotateTo(190, 1);
            gripper.wristRotateTo_Pitch(160);
            gripper.wristRotateTo_Roll(0);
            gripper.openBeakWide();

            ArrayList<double[]> secondSampleCollection = new ArrayList<double[]>();
            secondSampleCollection.add(new double[]{330,120,0});
            secondSampleCollection.add(new double[]{300, 25, 0});
            robot.toWaypointBezier(secondSampleCollection, 1, 1.45, 1.55);

            gripper.closeBeak();
            Thread.sleep(250);
            gripper.liftTo(11.25);
            gripper.elbowRotateTo(112, 1);//32
            gripper.wristRotateTo_Pitch(85); //243.33
            gripper.wristRotateTo_Roll(0);
            gripper.liftTo(0);

            // Go to Hang 1st specimen
            ArrayList<double[]> firstHang = new ArrayList<double[]>();
            firstHang.add(new double[]{155, 60, 0});
            firstHang.add(new double[]{157, 95, 0});
            robot.toWaypointBezier(firstHang, 1, 2.25, 2.5);

            gripper.liftTo(25);
            while(!gripper.eqWT(gripper.getLiftHeight(), 23, 0.5)){};
            //Thread.sleep(820);
            gripper.openBeak();

            // Move to Specimen Extract Position
            gripper.liftTo(0);
            gripper.elbowRotateTo(190, 1);
            gripper.wristRotateTo_Pitch(160);
            gripper.wristRotateTo_Roll(0);
            gripper.openBeakWide();

            // Extract 2nd Specimen
            ArrayList<double[]> secondSpecimenExtractMov = new ArrayList<double[]>();
            secondSpecimenExtractMov.add(new double[]{210, 60, 0});
            secondSpecimenExtractMov.add(new double[]{240, 80, 0});
            secondSpecimenExtractMov.add(new double[]{320, 100, 0});
            secondSpecimenExtractMov.add(new double[]{290, 60, 0});
            secondSpecimenExtractMov.add(new double[]{277, 25, 0});
            robot.toWaypointBezier(secondSpecimenExtractMov, 1,2.3, 2.8);

            gripper.closeBeak();
            Thread.sleep(250);
            gripper.liftTo(11.25);
            gripper.elbowRotateTo(112, 1);//32
            gripper.wristRotateTo_Pitch(85); //243.33
            gripper.wristRotateTo_Roll(0);
            gripper.liftTo(0);

            //Go to Hang 2nd specimen and hang
            ArrayList<double[]> secondHang = new ArrayList<double[]>();
            secondHang.add(new double[]{157, 60, 0});
            secondHang.add(new double[]{165, 95, 0});
            robot.toWaypointBezier(secondHang, 1, 2.25, 2.5);

            gripper.liftTo(25);
            while(!gripper.eqWT(gripper.getLiftHeight(), 23, 0.5)){};
            //Thread.sleep(820);
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
            thirdSpecimenExtractMov.add(new double[]{240, 80, 0});
            thirdSpecimenExtractMov.add(new double[]{320, 100, 0});
            thirdSpecimenExtractMov.add(new double[]{290, 60, 0});
            thirdSpecimenExtractMov.add(new double[]{277, 25, 0});
            robot.toWaypointBezier(thirdSpecimenExtractMov, 1,2.3, 2.7);

            gripper.closeBeak();
            Thread.sleep(250);
            gripper.liftTo(11.25);
            gripper.elbowRotateTo(112, 1);//32
            gripper.wristRotateTo_Pitch(85); //243.33
            gripper.wristRotateTo_Roll(0);
            gripper.liftTo(0);

            //Go to Hang third specimen and hang
            ArrayList<double[]> thirdHang = new ArrayList<double[]>();
            thirdHang.add(new double[]{157, 60, 0});
            thirdHang.add(new double[]{170, 95, 0});
            robot.toWaypointBezier(thirdHang, 1, 2.25, 2.5);

            gripper.liftTo(25);
            while(!gripper.eqWT(gripper.getLiftHeight(), 23, 0.5)){};
            //Thread.sleep(820);
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
            fourthSpecimenExtractMov.add(new double[]{240, 80, 0});
            fourthSpecimenExtractMov.add(new double[]{320, 100, 0});
            fourthSpecimenExtractMov.add(new double[]{290, 60, 0});
            fourthSpecimenExtractMov.add(new double[]{277, 25, 0});
            robot.toWaypointBezier(fourthSpecimenExtractMov, 1,2.3, 2.7);

            gripper.closeBeak();
            Thread.sleep(250);
            gripper.liftTo(11.25);
            gripper.elbowRotateTo(112, 1);//32
            gripper.wristRotateTo_Pitch(85); //243.33
            gripper.wristRotateTo_Roll(0);
            gripper.liftTo(0);

            //Go to Hang 4th specimen and hang
            ArrayList<double[]> fourthHang = new ArrayList<double[]>();
            fourthHang.add(new double[]{170, 60, 0});
            fourthHang.add(new double[]{175, 95, 0});
            robot.toWaypointBezier(fourthHang, 1, 2.25, 2.5);

            gripper.liftTo(25);
            while(!gripper.eqWT(gripper.getLiftHeight(), 23, 0.5)){};
            //Thread.sleep(900);
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
