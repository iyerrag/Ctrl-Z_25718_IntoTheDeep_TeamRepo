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
public class AutoStartLeft_d extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        DcMotor fL = hardwareMap.get(DcMotor.class, "FrontLeft");
        DcMotor fR = hardwareMap.get(DcMotor.class, "FrontRight");
        DcMotor bL = hardwareMap.get(DcMotor.class, "BackLeft");
        DcMotor bR = hardwareMap.get(DcMotor.class, "BackRight");
        BHI260IMU IMU = hardwareMap.get(BHI260IMU.class, "imu");
        VoltageSensor voltmeter = hardwareMap.voltageSensor.iterator().next();

        chassis robot = new chassis(fL, fR, bL, bR, IMU, "IMU", 90, 12, 0, voltmeter);
        actuators gripper = new actuators(hardwareMap.get(Servo.class, "lwServo"), hardwareMap.get(Servo.class, "rwServo"), hardwareMap.get(Servo.class, "wrollServo"), hardwareMap.get(Servo.class,
                "beak"), hardwareMap.get(Servo.class, "sweeperServo"), hardwareMap.get(DcMotor.class, "lifterLeft"), hardwareMap.get(DcMotor.class, "lifterRight"), hardwareMap.get(DcMotor.class, "elbow"), hardwareMap.get(TouchSensor.class,"lifterTouchSensor"), hardwareMap.get(TouchSensor.class,"elbowTouchSensor"));

        gripper.resetElbow(); // Safety
        gripper.resetLifters(); // Safety
        gripper.closeBeak();
        gripper.sweeperClose();

        waitForStart();

        if (opModeIsActive()) {
            //gripper.initializePosition();

            //Defend
            ArrayList<double[]> defencePos = new ArrayList<double[]>();
            defencePos.add(new double[]{90, 120, 0});
            defencePos.add(new double[]{90, 180, 0});
            defencePos.add(new double[]{90, 240, 0});
            robot.toWaypointBezier(defencePos, 1, 1, 1.5);

            // Got to defensive position and maintain position for for 10 seconds
            robot.toWaypoint(90,270,0,3, 7.5); // Controller 3 provides 0 tolerance

            // First Sample basket drop

            gripper.liftTo(72);
            gripper.elbowRotateTo(160, 1);
            gripper.wristRotateTo_Pitch(170);
            gripper.wristRotateTo_Roll(0);//-180

            ArrayList<double[]> firstHighBucketDropMov1 = new ArrayList<double[]>();
            firstHighBucketDropMov1.add(new double[]{90, 240, 0});
            firstHighBucketDropMov1.add(new double[]{90, 180, 0});
            firstHighBucketDropMov1.add(new double[]{90, 100, 0});
            robot.toWaypointBezier(firstHighBucketDropMov1, 1, 2, 2.25);

            ArrayList<double[]> firstHighfirstHighBucketDropMov2 = new ArrayList<double[]>();
            firstHighfirstHighBucketDropMov2.add(new double[]{60, 60, -25});
            firstHighfirstHighBucketDropMov2.add(new double[]{40, 35, -45});
            robot.toWaypointBezier(firstHighfirstHighBucketDropMov2, 1, 1.5, 1.75);
            while (!gripper.eqWT(gripper.getElbowAngle(), 160, 1)) {}
            while (!gripper.eqWT(gripper.getWristAngle_Pitch(), 170, 1)) {}
            gripper.declareHighBucketStatusTrue();
            Thread.sleep(150);
            gripper.openBeakWide();
            Thread.sleep(200);

            //Second Sample pick up
            gripper.elbowRotateTo(90, 1);
            gripper.wristRotateTo_Pitch(115);//95
            gripper.wristRotateTo_Roll(0);//-180
            while (!gripper.eqWT(gripper.getWristAngle_Pitch(), 115, 1)) {} // clear the basket
            gripper.liftTo(0);

            ArrayList<double[]> sample2Pickup = new ArrayList<double[]>();
            sample2Pickup.add(new double[]{50, 50, 0});
            sample2Pickup.add(new double[]{52, 56, 0});
            robot.toWaypointBezier(sample2Pickup, 1, 2, 2.5);
            gripper.elbowRotateTo(-16, 1);
            gripper.resetLifters();
            while (!gripper.eqWT(gripper.getElbowAngle(), -16, 1)) {}
            while (!gripper.eqWT(gripper.getWristAngle_Pitch(), 115, 1)) {}
            gripper.declareHighBucketStatusFalse();

            Thread.sleep(150);
            gripper.closeBeak();
            Thread.sleep(150);

            // Second Sample basket drop
            gripper.liftTo(72);
            ArrayList<double[]> sample2DropOff = new ArrayList<double[]>();
            sample2DropOff.add(new double[]{50, 50, -30});
            sample2DropOff.add(new double[]{38, 33, -45});
            robot.toWaypointBezier(sample2DropOff, 1, 2.25, 2.5);
            gripper.elbowRotateTo(160, 1);
            gripper.wristRotateTo_Pitch(170);
            gripper.wristRotateTo_Roll(0);//-180
            while (!gripper.eqWT(gripper.getElbowAngle(), 160, 1)) {}
            while (!gripper.eqWT(gripper.getWristAngle_Pitch(), 170, 1)) {}
            gripper.declareHighBucketStatusTrue();
            Thread.sleep(150);
            gripper.openBeakWide();
            Thread.sleep(200);

            //Third Sample pick up
            gripper.elbowRotateTo(90, 1);
            gripper.wristRotateTo_Pitch(115);//95
            gripper.wristRotateTo_Roll(0);//-180
            while (!gripper.eqWT(gripper.getWristAngle_Pitch(), 115, 1)) {} // clear the basket
            gripper.liftTo(0);

            ArrayList<double[]> sample3Pickup = new ArrayList<double[]>();
            sample3Pickup.add(new double[]{36, 50, 0});
            sample3Pickup.add(new double[]{25, 58, 0});
            robot.toWaypointBezier(sample3Pickup, 1, 1.9, 2.6);
            gripper.elbowRotateTo(-16, 1);
            gripper.resetLifters();
            while (!gripper.eqWT(gripper.getElbowAngle(), -16, 1)) {}
            while (!gripper.eqWT(gripper.getWristAngle_Pitch(), 115, 1)) {}
            gripper.declareHighBucketStatusFalse();
            Thread.sleep(150);
            gripper.closeBeak();
            Thread.sleep(150);

            // Third sample basket drop
            gripper.liftTo(72);
            ArrayList<double[]> sample3DropOff = new ArrayList<double[]>();
            sample3DropOff.add(new double[]{34, 50, -30});
            sample3DropOff.add(new double[]{38, 33, -45});
            robot.toWaypointBezier(sample3DropOff, 1, 2.25, 2.5);
            gripper.elbowRotateTo(160, 1);
            gripper.wristRotateTo_Pitch(170);
            gripper.wristRotateTo_Roll(0);//-180
            while (!gripper.eqWT(gripper.getElbowAngle(), 160, 1)) {}
            while (!gripper.eqWT(gripper.getWristAngle_Pitch(), 170, 1)) {}
            gripper.declareHighBucketStatusTrue();
            Thread.sleep(150);
            gripper.openBeakWide();
            Thread.sleep(200);

            // Move forward to avoid basket and stop
            fL.setPower(0.25);
            fR.setPower(0.25);
            bL.setPower(0.25);
            bR.setPower(0.25);
            gripper.elbowRotateTo(85, 1);
            gripper.wristRotateTo_Pitch(115);
            while (!gripper.eqWT(gripper.getElbowAngle(), 85, 5)) {}
            gripper.moveToStartingPosition();
            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);
            bR.setPower(0);
        }
    }
}
