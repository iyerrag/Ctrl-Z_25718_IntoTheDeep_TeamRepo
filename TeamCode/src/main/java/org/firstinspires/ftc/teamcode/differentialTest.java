package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
public class differentialTest extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        actuators gripper = new actuators(hardwareMap.get(Servo.class, "lwServo"), hardwareMap.get(Servo.class, "rwServo"), hardwareMap.get(Servo.class, "wrollServo"), hardwareMap.get(Servo.class,
                "beak"), hardwareMap.get(DcMotor.class, "lifterLeft"), hardwareMap.get(DcMotor.class, "lifterRight"), hardwareMap.get(DcMotor.class, "elbow"), hardwareMap.get(DistanceSensor.class,
                "lifterHeightSensor"), hardwareMap.get(DistanceSensor.class, "frontDistanceSensor"), hardwareMap.get(TouchSensor.class,"lifterTouchSensor"),hardwareMap.get(TouchSensor.class,"elbowTouchSensor") );

        Servo lwServo = hardwareMap.get(Servo.class, "lwServo");
        Servo rwServo = hardwareMap.get(Servo.class, "rwServo");
        Servo wrollServo = hardwareMap.get(Servo.class, "wrollServo");
        waitForStart();

        lwServo.setPosition(0.5);
        rwServo.setPosition(0.5);
        wrollServo.setPosition(0.94);

       double rightPos = 0;
       double leftPos = 0;

       while(opModeIsActive()){
           if(gamepad1.left_bumper){
               lwServo.setPosition(lwServo.getPosition() + 0.01);
               rwServo.setPosition(rwServo.getPosition() - 0.01);
               Thread.sleep(10);
           }
           else if (gamepad1.left_trigger == 1) {
               lwServo.setPosition(lwServo.getPosition() - 0.01);
               rwServo.setPosition(rwServo.getPosition() + 0.01);
               Thread.sleep(10);
           }
           else if (gamepad1.right_bumper){
               wrollServo.setPosition(wrollServo.getPosition() + 0.01);
               Thread.sleep(10);
           }
           else if (gamepad1.right_trigger == 1){
               wrollServo.setPosition(wrollServo.getPosition() - 0.01);
               Thread.sleep(10);
           }
           else if(gamepad1.a){
               gripper.wristRotateTo_Pitch(180);
           }
           telemetry.addData("lwServoPos", lwServo.getPosition());
           telemetry.addData("rwServoPos", rwServo.getPosition());
           telemetry.addData("rollPos", wrollServo.getPosition());
           telemetry.update();

           //gripper.setAngularPosition(0,0);
       }
    }
}
