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

        chassis robot = new chassis(fL, fR, bL, bR, IMU, "IMU", 120, 12, 0, voltmeter);
        actuators gripper = new actuators(hardwareMap.get(Servo.class, "lwServo"), hardwareMap.get(Servo.class, "rwServo"), hardwareMap.get(Servo.class, "wrollServo"), hardwareMap.get(Servo.class,
                "beak"), hardwareMap.get(Servo.class, "sweeperServo"), hardwareMap.get(DcMotor.class, "lifterLeft"), hardwareMap.get(DcMotor.class, "lifterRight"), hardwareMap.get(DcMotor.class, "elbow"), hardwareMap.get(TouchSensor.class,"lifterTouchSensor"), hardwareMap.get(TouchSensor.class,"elbowTouchSensor"));

        gripper.resetElbow(); // Safety
        gripper.resetLifters(); // Safety
        gripper.closeBeak();
        gripper.sweeperClose();

        waitForStart();

        if (opModeIsActive()) {
            //gripper.initializePosition();

            // First Sample basket drop

            gripper.liftTo(72);
            gripper.elbowRotateTo(160, 1);
            gripper.wristRotateTo_Pitch(170);
            gripper.wristRotateTo_Roll(0);//-180
            ArrayList<double[]> firstHighBucketDropMov = new ArrayList<double[]>();
            firstHighBucketDropMov.add(new double[]{120, 60, 0});
            firstHighBucketDropMov.add(new double[]{34, 25, -45});
            robot.toWaypointBezier(firstHighBucketDropMov, 1, 1.75, 2.75);
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
            sample2Pickup.add(new double[]{60, 53, 0});
            robot.toWaypointBezier(sample2Pickup, 1, 1.5, 2.5);
            gripper.elbowRotateTo(-15, 1);
            gripper.resetLifters();
            while (!gripper.eqWT(gripper.getElbowAngle(), -15, 1)) {}
            while (!gripper.eqWT(gripper.getWristAngle_Pitch(), 115, 1)) {}
            gripper.declareHighBucketStatusFalse();

            Thread.sleep(150);
            gripper.closeBeak();
            Thread.sleep(150);

            // Second Sample basket drop
            gripper.liftTo(72);
            ArrayList<double[]> sample2DropOff = new ArrayList<double[]>();
            sample2DropOff.add(new double[]{50, 50, -30});
            sample2DropOff.add(new double[]{32, 23, -45});
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
            sample3Pickup.add(new double[]{28, 56, 0});
            robot.toWaypointBezier(sample3Pickup, 1, 2, 2.5);
            gripper.elbowRotateTo(-15, 1);
            gripper.resetLifters();
            while (!gripper.eqWT(gripper.getElbowAngle(), -15, 1)) {}
            while (!gripper.eqWT(gripper.getWristAngle_Pitch(), 115, 1)) {}
            gripper.declareHighBucketStatusFalse();
            Thread.sleep(150);
            gripper.closeBeak();
            Thread.sleep(150);

            // Third sample basket drop
            gripper.liftTo(72);
            ArrayList<double[]> sample3DropOff = new ArrayList<double[]>();
            sample3DropOff.add(new double[]{34, 50, -30});
            sample3DropOff.add(new double[]{32, 23, -45});
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

            //Fourth Sample pick up
            gripper.elbowRotateTo(90, 1);
            gripper.wristRotateTo_Pitch(115);//95
            gripper.wristRotateTo_Roll(0);//-180
            while (!gripper.eqWT(gripper.getWristAngle_Pitch(), 115, 1)) {} // clear the basket
            gripper.liftTo(0);

            gripper.wristRotateTo_Roll(-45);
            ArrayList<double[]> sample4Pickup = new ArrayList<double[]>();
            sample4Pickup.add(new double[]{60, 66, 35});
            sample4Pickup.add(new double[]{45, 55, 35});//{45, 56.5, 45}
            gripper.elbowRotateTo(0,1);

            robot.toWaypointBezier(sample4Pickup, 1, 2.0, 2.25);
            while (!gripper.eqWT(gripper.getElbowAngle(), 0, 1)) {}
            while (!gripper.eqWT(gripper.getWristAngle_Pitch(), 115, 1)) {}
            gripper.resetLifters();
            Thread.sleep(150);
            robot.toWaypoint(41, 63, 45, 1, 0.75);
            gripper.elbowRotateTo(-17, 0.6);//-5
            while (!gripper.eqWT(gripper.getElbowAngle(), -17, 1)) {}
            gripper.declareHighBucketStatusFalse();

            Thread.sleep(150);
            gripper.closeBeak();
            Thread.sleep(150);

            // Fourth sample basket drop
            gripper.liftTo(72);
            ArrayList<double[]> sample4DropOff = new ArrayList<double[]>();
            sample3DropOff.add(new double[]{60, 60, -15});
            sample3DropOff.add(new double[]{120, 60, -30});//Added to move robot away from wall
            sample3DropOff.add(new double[]{34, 23, -50}); //{37, 28, -45}
            robot.toWaypointBezier(sample3DropOff, 1, 2.5, 2.75);
            gripper.elbowRotateTo(160, 1);
            gripper.wristRotateTo_Pitch(170);
            gripper.wristRotateTo_Roll(0);//-180
            while (!gripper.eqWT(gripper.getElbowAngle(), 160, 1)) {}
            while (!gripper.eqWT(gripper.getWristAngle_Pitch(), 170, 1)) {}
            gripper.declareHighBucketStatusTrue();
            Thread.sleep(150);
            gripper.openBeak();
            Thread.sleep(200);

            // Go to Reset Position for TeleOp
            fL.setPower(0.25);
            fR.setPower(0.25);
            bL.setPower(0.25);
            bR.setPower(0.25);
            gripper.elbowRotateTo(90, 1);
            while (!gripper.eqWT(gripper.getElbowAngle(), 90, 5)) {}
            gripper.moveToStartingPosition();
            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);
            bR.setPower(0);
        }
    }
}
