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
public class AutoStartRight_5s extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        DcMotor fL = hardwareMap.get(DcMotor.class, "FrontLeft");
        DcMotor fR = hardwareMap.get(DcMotor.class, "FrontRight");
        DcMotor bL = hardwareMap.get(DcMotor.class, "BackLeft");
        DcMotor bR = hardwareMap.get(DcMotor.class, "BackRight");
        BHI260IMU IMU = hardwareMap.get(BHI260IMU.class, "imu");
        VoltageSensor voltmeter = hardwareMap.voltageSensor.iterator().next();
        //WebcamName myCamera = hardwareMap.get(WebcamName.class, "Webcam 1");

        chassis robot = new chassis(fL, fR, bL, bR, IMU, "IMU", 203, 12, 0, voltmeter);
        //chassis robot = new chassis(fL, fR, bL, bR, IMU, "IMU", 240, 12, 0, voltmeter, myCamera, new double[]{14.605, 32.385, 0});
        actuators gripper = new actuators(hardwareMap.get(Servo.class, "lwServo"), hardwareMap.get(Servo.class, "rwServo"), hardwareMap.get(Servo.class, "wrollServo"), hardwareMap.get(Servo.class,
                "beak"), hardwareMap.get(Servo.class, "sweeperServo"), hardwareMap.get(DcMotor.class, "lifterLeft"), hardwareMap.get(DcMotor.class, "lifterRight"), hardwareMap.get(DcMotor.class, "elbow"), hardwareMap.get(TouchSensor.class,"lifterTouchSensor"), hardwareMap.get(TouchSensor.class,"elbowTouchSensor"));


        gripper.resetElbow(); // Safety
        gripper.resetLifters(); // Safety
        gripper.closeBeak();
        gripper.sweeperSetup();
        waitForStart();

        if (opModeIsActive()) {

            // Move to Hang Insert Position
            gripper.elbowRotateTo(112, 1);//32
            gripper.wristRotateTo_Pitch(85); //243.33
            gripper.wristRotateTo_Roll(0);

            // Go to Hang 1st specimen and hang specimen
            robot.translateRadDeg(1,55, 0.7, false);
            robot.toWaypoint(155, 95, 0, 1, 0.5);
            gripper.liftTo(18);
            while(!gripper.eqWT(gripper.getLiftHeight(), 16, 0.5)){};
            gripper.openBeakWide();
            gripper.liftTo(0);
            gripper.elbowRotateTo(190, 1);
            gripper.wristRotateTo_Pitch(30);
            gripper.wristRotateTo_Roll(0);
            //gripper.openBeakWide();


            // Get to first sample
            ArrayList<double[]> initializeCollectionPos = new ArrayList<double[]>();
            initializeCollectionPos.add(new double[]{180, 60, 0});
            initializeCollectionPos.add(new double[]{300, 40, 0});
            initializeCollectionPos.add(new double[]{280, 100, 0});
            robot.toWaypointBezier(initializeCollectionPos, 1, 1.3,1.55);

            // Collect first sample
            gripper.sweeperOpen();
            while(!gripper.eqWT(gripper.getSweeperPosition(), 0.0, 0.1)){};
            ArrayList<double[]> collectMov1 = new ArrayList<double[]>();
            collectMov1.add(new double[]{280, 60, -45});
            collectMov1.add(new double[]{300, 60, -45});
            collectMov1.add(new double[]{260, 45, -75});
            collectMov1.add(new double[]{260, 27, -90});
            robot.toWaypointBezier(collectMov1, 1, 0.95, 1.2);
            //gripper.sweeperOpenSmall();
            robot.toWaypoint(300, 80, 0,1, 1.35);

            //Collect second sample
            //gripper.sweeperOpen();
            //while(!gripper.eqWT(gripper.getSweeperPosition(), 0.0, 0.1)){};
            ArrayList<double[]> collectMov2 = new ArrayList<double[]>();
            collectMov2.add(new double[]{310, 60, -45});
            collectMov2.add(new double[]{330, 60, -45});
            collectMov2.add(new double[]{290, 45, -75});
            collectMov2.add(new double[]{290, 27, -90});
            robot.toWaypointBezier(collectMov2, 1, 1.05, 1.2);

            //Prepare to collect third sample
            gripper.sweeperClose();
            gripper.liftTo(0);
            gripper.elbowRotateTo(188, 1);
            gripper.wristRotateTo_Pitch(170);
            gripper.wristRotateTo_Roll(0);
            gripper.openBeakWide();

            ArrayList<double[]> initializeThirdSamplePickup = new ArrayList<double []>();
            initializeThirdSamplePickup.add(new double[]{300, 40, 50});
            initializeThirdSamplePickup.add(new double[]{235, 153, 0});
            initializeThirdSamplePickup.add(new double[]{347, 153, 7.5});
            robot.toWaypointBezier(initializeThirdSamplePickup, 1, 1.5, 1.75);



            //Collect third sample and extract second specimen
            robot.translateRadDeg(1, -88, 1.4, true);//-95
            robot.toWaypoint(robot.getPosition()[0], 25, 0, 1, 0.7);

            /*ArrayList<double[]> secondSpecimenExtract = new ArrayList<double []>();
            secondSpecimenExtract.add(new double[]{330, 120, 0});
            secondSpecimenExtract.add(new double[]{330, 75, 0});
            secondSpecimenExtract.add(new double[]{330, 33, 0});
            robot.toWaypointBezier(secondSpecimenExtract, 1, 1.5, 2);*/

            gripper.closeBeak();
            Thread.sleep(160);
            gripper.liftTo(11.25);
            gripper.elbowRotateTo(112, 1);//32
            gripper.wristRotateTo_Pitch(85); //243.33
            gripper.wristRotateTo_Roll(0);
            gripper.liftTo(0);

            //Go to Hang 2nd specimen and hang
            ArrayList<double[]> secondHang = new ArrayList<double[]>();
            secondHang.add(new double[]{170, 60, 0});
            secondHang.add(new double[]{180, 95, 0});
            robot.toWaypointBezier(secondHang, 1, 2.5, 2.66);

            gripper.liftTo(18);
            while(!gripper.eqWT(gripper.getLiftHeight(), 16, 0.5)){};
            gripper.openBeakWide();

            // Extract 3rd Specimen
            gripper.liftTo(0);
            gripper.elbowRotateTo(190, 1);
            gripper.wristRotateTo_Pitch(160);
            gripper.wristRotateTo_Roll(0);
            //gripper.openBeakWide();

            ArrayList<double[]> thirdSpecimenExtractMov = new ArrayList<double[]>();
            thirdSpecimenExtractMov.add(new double[]{210, 20, 0});
            thirdSpecimenExtractMov.add(new double[]{290, 130, 0});
            thirdSpecimenExtractMov.add(new double[]{277, 25, 0});
            robot.toWaypointBezier(thirdSpecimenExtractMov, 1,1.5, 1.8);

            gripper.closeBeak();
            Thread.sleep(160);
            gripper.liftTo(11.25);
            gripper.elbowRotateTo(112, 1);//32
            gripper.wristRotateTo_Pitch(85); //243.33
            gripper.wristRotateTo_Roll(0);
            gripper.liftTo(0);

            //Go to Hang 3rd specimen and hang
            ArrayList<double[]> thirdHang = new ArrayList<double[]>();
            thirdHang.add(new double[]{170, 50, 0});
            thirdHang.add(new double[]{163, 95, 0});
            robot.toWaypointBezier(thirdHang, 1, 2, 2.10);

            gripper.liftTo(18);
            while(!gripper.eqWT(gripper.getLiftHeight(), 16, 0.5)){};
            //Thread.sleep(900);
            gripper.openBeakWide();

            // Extract 4th Specimen
            gripper.liftTo(0);
            gripper.elbowRotateTo(190, 1);
            gripper.wristRotateTo_Pitch(160);
            gripper.wristRotateTo_Roll(0);
            //gripper.openBeakWide();

            ArrayList<double[]> fourthSpecimenExtractMov = new ArrayList<double[]>();
            fourthSpecimenExtractMov.add(new double[]{210, 20, 0});
            fourthSpecimenExtractMov.add(new double[]{290, 130, 0});
            fourthSpecimenExtractMov.add(new double[]{277, 25, 0});
            robot.toWaypointBezier(fourthSpecimenExtractMov, 1,1.5, 1.8);

            gripper.closeBeak();
            Thread.sleep(160);
            gripper.liftTo(11.25);
            gripper.elbowRotateTo(112, 1);//32
            gripper.wristRotateTo_Pitch(85); //243.33
            gripper.wristRotateTo_Roll(0);
            gripper.liftTo(0);

            //Go to Hang 4th specimen and hang
            ArrayList<double[]> fourthdHang = new ArrayList<double[]>();
            fourthdHang.add(new double[]{170, 50, 0});
            fourthdHang.add(new double[]{170, 95, 0});
            robot.toWaypointBezier(fourthdHang, 1, 2, 2.15);

            gripper.liftTo(18);
            while(!gripper.eqWT(gripper.getLiftHeight(), 16, 0.5)){};
            //Thread.sleep(900);
            gripper.openBeakWide();

            // Extract 5th Specimen
            gripper.liftTo(0);
            gripper.elbowRotateTo(190, 1);
            gripper.wristRotateTo_Pitch(160);
            gripper.wristRotateTo_Roll(0);
            //gripper.openBeakWide();

            ArrayList<double[]> fifthSpecimenExtractMov = new ArrayList<double[]>();
            fifthSpecimenExtractMov.add(new double[]{210, 20, 0});
            fifthSpecimenExtractMov.add(new double[]{290, 130, 0});
            fifthSpecimenExtractMov.add(new double[]{277, 25, 0});
            robot.toWaypointBezier(fifthSpecimenExtractMov, 1,1.5, 1.8);

            gripper.closeBeak();
            Thread.sleep(160);
            gripper.liftTo(11.25);
            gripper.elbowRotateTo(112, 1);//32
            gripper.wristRotateTo_Pitch(85); //243.33
            gripper.wristRotateTo_Roll(0);
            gripper.liftTo(0);

            //Go to Hang 5th specimen and hang
            ArrayList<double[]> fifthhdHang = new ArrayList<double[]>();
            fifthhdHang.add(new double[]{170, 50, 0});
            fifthhdHang.add(new double[]{178, 95, 0});
            robot.toWaypointBezier(fifthhdHang, 1, 2, 2.10);

            gripper.liftTo(18);
            while(!gripper.eqWT(gripper.getLiftHeight(), 16, 0.5)){};
            //Thread.sleep(900);
            gripper.openBeakWide();
            gripper.elbowRotateTo(180,1);
            gripper.liftTo(0);

        }
    }
}
