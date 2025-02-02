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

       actuators gripper = new actuators(hardwareMap.get(Servo.class, "differential_left"), hardwareMap.get(Servo.class, "differential_right"), hardwareMap.get(Servo.class,
               "beak"), hardwareMap.get(DcMotor.class, "lifterLeft"), hardwareMap.get(DcMotor.class, "lifterRight"), hardwareMap.get(DcMotor.class, "elbow"), hardwareMap.get(DistanceSensor.class,
               "lifterHeightSensor"), hardwareMap.get(TouchSensor.class,"lifterTouchSensor"));


       waitForStart();



       double rightPos = 0;
       double leftPos = 0;

       while(opModeIsActive()){
           if(gamepad1.right_bumper){
               gripper.setAngularPosition(90, 0);
           }
           else if(gamepad1.right_trigger == 1){
               gripper.setAngularPosition(-90, 0);
           }
           else if(gamepad1.left_bumper){
               gripper.setAngularPosition(-90, 90);
           }
           else if(gamepad1.left_trigger == 1){
               gripper.setAngularPosition(-90, -45);
           }

           telemetry.addData("rollPos", gripper.getAngularPosition_Roll());
           telemetry.addData("pitchPos", gripper.getAngularPosition_Pitch());
           telemetry.update();

           //gripper.setAngularPosition(0,0);
       }
    }
}
