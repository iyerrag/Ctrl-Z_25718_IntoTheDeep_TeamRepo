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

        chassis robot = new chassis(fL, fR, bL, bR, IMU, "IMU", 120, 12, 0, voltmeter, hardwareMap.get(DistanceSensor.class, "frontDistanceSensor"));
        actuators gripper = new actuators(hardwareMap.get(Servo.class, "lwServo"), hardwareMap.get(Servo.class, "rwServo"), hardwareMap.get(Servo.class, "wrollServo"), hardwareMap.get(Servo.class,
                "beak"), hardwareMap.get(DcMotor.class, "lifterLeft"), hardwareMap.get(DcMotor.class, "lifterRight"), hardwareMap.get(DcMotor.class, "elbow"), hardwareMap.get(DistanceSensor.class, "frontDistanceSensor"), hardwareMap.get(TouchSensor.class,"lifterTouchSensor"), hardwareMap.get(TouchSensor.class,"elbowTouchSensor"));

        gripper.resetElbow(); // Safety
        gripper.resetLifters(); // Safety
        gripper.closeBeak();
        waitForStart();

        // First Sample

        gripper.liftTo(72);
        gripper.elbowRotateTo(160, 1);
        while(!gripper.eqWT(gripper.getElbowAngle(), 5, 160)){}
        gripper.wristRotateTo_Pitch(170);
        gripper.wristRotateTo_Roll(0);//-180
        ArrayList<double[]> firstHighBucketDropMov = new ArrayList<double[]>();
        firstHighBucketDropMov.add(new double[]{120, 60, 0});
        firstHighBucketDropMov.add(new double[]{34, 25, -45});
        robot.toWaypointBezier(firstHighBucketDropMov, 1, 1.75, 2.75);
        while(!gripper.eqWT(gripper.getElbowAngle(), 160, 1)){}
        while(!gripper.eqWT(gripper.getWristAngle_Pitch(), 170, 1)){}
        gripper.declareHighBucketStatusTrue();
        Thread.sleep(150);
        gripper.openBeakWide();
        Thread.sleep(200);

        //Second Sample
        gripper.elbowRotateTo(0, 1);
        gripper.wristRotateTo_Pitch(115);
        gripper.wristRotateTo_Roll(0);
        while(!gripper.eqWT(gripper.getElbowAngle(), 90, 5)){}
        gripper.liftTo(0);
        ArrayList<double[]> sample2Pickup = new ArrayList<double[]>();
        sample2Pickup.add(new double[]{50, 53, 0});
        sample2Pickup.add(new double[]{60, 53, 0});
        robot.toWaypointBezier(sample2Pickup, 1, 1.5, 2.5);

        gripper.resetLifters();
        gripper.elbowRotateTo(-20, 0.5);
        while(!gripper.eqWT(gripper.getElbowAngle(), -17, 1)){}
        while(!gripper.eqWT(gripper.getWristAngle_Pitch(), 115, 1)){}
        gripper.declareHighBucketStatusFalse();

        Thread.sleep(150);
        gripper.closeBeak();
        Thread.sleep(150);
        gripper.liftTo(72);
        ArrayList<double[]> sample2DropOff = new ArrayList<double[]>();
        sample2DropOff.add(new double[]{50, 50, -30});
        sample2DropOff.add(new double[]{34, 25, -45});
        robot.toWaypointBezier(sample2DropOff, 1, 2.25, 2.5);
        gripper.elbowRotateTo(160, 1);
        gripper.wristRotateTo_Pitch(170);
        gripper.wristRotateTo_Roll(0);//-180
        while(!gripper.eqWT(gripper.getElbowAngle(), 160, 1)){}
        while(!gripper.eqWT(gripper.getWristAngle_Pitch(), 170, 1)){}
        gripper.declareHighBucketStatusTrue();
        Thread.sleep(150);
        gripper.openBeakWide();
        Thread.sleep(200);

        //Third Sample
        gripper.elbowRotateTo(0, 1);
        gripper.wristRotateTo_Pitch(115);
        gripper.wristRotateTo_Roll(0);
        while(!gripper.eqWT(gripper.getElbowAngle(), 90, 5)){}
        gripper.liftTo(0);
        ArrayList<double[]> sample3Pickup = new ArrayList<double[]>();
        sample3Pickup.add(new double[]{36, 53, 0});
        sample3Pickup.add(new double[]{29, 55.6, 0});
        robot.toWaypointBezier(sample3Pickup, 1, 1.5, 2.5);
        gripper.resetLifters();

        gripper.elbowRotateTo(-20, 0.5);
        while(!gripper.eqWT(gripper.getElbowAngle(), -17, 1)){}
        while(!gripper.eqWT(gripper.getWristAngle_Pitch(), 115, 1)){}
        gripper.declareHighBucketStatusFalse();

        Thread.sleep(150);
        gripper.closeBeak();
        Thread.sleep(150);
        gripper.liftTo(72);
        ArrayList<double[]> sample3DropOff = new ArrayList<double[]>();
        sample3DropOff.add(new double[]{34, 50, -30});
        sample3DropOff.add(new double[]{34, 26, -45});
        robot.toWaypointBezier(sample3DropOff, 1, 2.25, 2.5);
        gripper.elbowRotateTo(160, 1);
        gripper.wristRotateTo_Pitch(170);
        gripper.wristRotateTo_Roll(0);//-180
        while(!gripper.eqWT(gripper.getElbowAngle(), 160, 1)){}
        while(!gripper.eqWT(gripper.getWristAngle_Pitch(), 170, 1)){}
        gripper.declareHighBucketStatusTrue();
        Thread.sleep(150);
        gripper.openBeakWide();
        Thread.sleep(200);

        //Fourth Sample
        gripper.elbowRotateTo(-10, 0.6);
        gripper.wristRotateTo_Pitch(115);
        gripper.wristRotateTo_Roll(-45);
        while(!gripper.eqWT(gripper.getElbowAngle(), 90, 5)){}
        gripper.liftTo(0);
        ArrayList<double[]> sample4Pickup = new ArrayList<double[]>();
        sample4Pickup.add(new double[]{60, 69, 45});
        sample4Pickup.add(new double[]{51, 57.5, 45});//{45, 56.5, 45}
        robot.toWaypointBezier(sample4Pickup, 1, 2.0, 2.25);
        while(!gripper.eqWT(gripper.getWristAngle_Pitch(), 115, 1)){}

        gripper.resetLifters();
        gripper.elbowRotateTo(-20, 0.5);
        while(!gripper.eqWT(gripper.getElbowAngle(),-17, 1)){}
        gripper.declareHighBucketStatusFalse();

        Thread.sleep(150);
        gripper.closeBeak();
        Thread.sleep(150);
        gripper.liftTo(72);
        ArrayList<double[]> sample4DropOff = new ArrayList<double[]>();
        sample3DropOff.add(new double[]{45, 45, -15});
        sample3DropOff.add(new double[]{34, 26, -50});
        robot.toWaypointBezier(sample3DropOff, 1, 2.5, 2.75);
        gripper.elbowRotateTo(160, 1);
        gripper.wristRotateTo_Pitch(170);
        gripper.wristRotateTo_Roll(0);//-180
        while(!gripper.eqWT(gripper.getElbowAngle(), 160, 1)){}
        while(!gripper.eqWT(gripper.getWristAngle_Pitch(), 170, 1)){}
        gripper.declareHighBucketStatusTrue();
        Thread.sleep(150);
        gripper.openBeak();
        Thread.sleep(200);

        // Go to Reset Position for TeleOp
        gripper.elbowRotateTo(90, 1);
        while(!gripper.eqWT(gripper.getElbowAngle(), 90, 5)){}
        gripper.moveToStartingPosition();
    }
}
