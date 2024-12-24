/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;

@TeleOp
public class TeleOpSubmersible extends LinearOpMode {

    //Define Rate Limiter Constants (Rate Limiters = Limits Robot Acceleration)
    private static final double tolX = .03;
    private static final double tolY = .03;
    private static final double tolTheta = 1;
    private static final double deccelScale = 3.0;

    //Define Wheel Biases to Manage Robot Drift
    static final double frontLeftBias = 0.97;
    static final double frontRightBias = 0.92;
    static final double backLeftBias = .93;
    static final double backRightBias = 0.96;

    //Define Timer Object
    private ElapsedTime runtime = new ElapsedTime();

    //Declare Motor Objects
    private DcMotor fL;
    private DcMotor fR;
    private DcMotor bL;
    private DcMotor bR;

    //Enter Run-Mode:
    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize Telemetry Print:
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Assign Motors
        fL = hardwareMap.get(DcMotor.class, "FrontLeft");
        fR = hardwareMap.get(DcMotor.class, "FrontRight");
        bL = hardwareMap.get(DcMotor.class, "BackLeft");
        bR = hardwareMap.get(DcMotor.class, "BackRight");

        //Define Sensors (IMU, battery voltmeter, webcam)
        BHI260IMU IMU = hardwareMap.get(BHI260IMU.class, "imu");
        VoltageSensor voltmeter = hardwareMap.voltageSensor.iterator().next();
        WebcamName myCamera = hardwareMap.get(WebcamName.class, "Webcam 1");

        //Define DriveBase: new chassis(motor1, motor2, motor3, motor4, IMU, angleMode, startingX, startingY, startingTheta, voltmeter, webcam, camera offset array)
        chassis robot = new chassis(fL, fR, bL, bR, IMU, "IMU", 180, 15, 0, voltmeter, myCamera, new double[]{14.605, 32.385, 0});

        //Define TaskManipulator: new claw(wrist servo, beak servo, left lifter, right lifter, elbow);
        actuators gripper = new actuators(hardwareMap.get(Servo.class, "wrist"), hardwareMap.get(Servo.class, "beak"), hardwareMap.get(DcMotor.class, "lifterLeft"), hardwareMap.get(DcMotor.class, "lifterRight"), hardwareMap.get(DcMotor.class, "elbow"));

        //Define Previous Var (stores the previous velocity command sig. to motors)
        double[] previous = {0, 0};

        //Wait for Driver to Start
        waitForStart();

        //Re-initialize Runtime
        runtime.reset();

        //Until the Match-End:
        while (opModeIsActive()) {

            //If Left Trigger is pressed on the task controller, lower the lift
            if(gamepad2.left_trigger == 1){

                //Set lift to moving downward
                gripper.lift(-1);

                //Allow DriveBase movement while lift is moving (until the left trigger is released)
                while(gamepad2.left_trigger == 1){
                    previous = driveBase(robot, previous[0], previous[1]);
                }

                //Set TaskManipulator to hold
                gripper.hold();
            }

            //If Left Bumper is pressed on the task controller, raise the lift
            else if(gamepad2.left_bumper){

                //Set lift to moving upward
                gripper.lift(1);

                //Allow DriveBase movement while lift is moving (until the left bumper is released)
                while(gamepad2.left_bumper){
                    previous = driveBase(robot, previous[0], previous[1]);
                }

                //Set TaskManipulator to hold
                gripper.hold();
            }

            //If Right Trigger is pressed on the task controller, rotate the elbow forward
            else if(gamepad2.right_trigger == 1){

                //Set elbow to forward rotation
                gripper.rotateElbow(-0.50);

                //Allow DriveBase movement while elbow is moving (until the right trigger is released)
                while(gamepad2.right_trigger == 1){
                    previous = driveBase(robot, previous[0], previous[1]);
                }

                //Set TaskManipulator to hold
                gripper.hold();
            }

            //If Right Bumper is pressed on the task controller, rotate the elbow backward
            else if(gamepad2.right_bumper){

                //Set the elbow to backward rotation
                gripper.rotateElbow(0.50);

                //Allow DriveBase movement while elbow is moving (until the right bumper is released)
                while(gamepad2.right_bumper){
                    previous = driveBase(robot, previous[0], previous[1]);
                }

                //Set TaskManipulator to hold
                gripper.hold();
            }

            //If button "a" is pressed on the driver controller, insert claw into submersible
            else if(gamepad1.a){

                //Initialize Robot Controller Settings
                robot.waypointSettings(1, 1, 1,
                        .01575, .020125, .0025, 0,
                        .012, .006025, 0.0025, 0,
                        .35, .035, .0035, 0,
                        .024, .03, 10,
                        .15, .15, .4);

                //Movement
                robot.toWaypoint(60, 60, -90, 2.5);
                gripper.moveToTransportPosition();
                robot.toWaypoint(60, 180, -90, 2.5);
                gripper.moveToInsertPosition();
                Thread.sleep(1250);
                robot.toWaypoint(110, 180, -90, 1);
                gripper.moveToPickupPosition();

            }

            //If the button "b" is pressed on the driver controller, extract the claw from the submersible
            else if(gamepad1.b){

                //Initialize Robot Controller Settings
                robot.waypointSettings(1, 1, 1,
                        .01575, .020125, .0025, 0,
                        .012, .006025, 0.0025, 0,
                        .35, .035, .0035, 0,
                        .024, .03, 10,
                        .15, .15, .4);

                //Movement
                gripper.moveToInsertPosition();
                robot.toWaypoint(60, 180, -90, 0.75);
                gripper.moveToTransportPosition();
                robot.toWaypoint(60, 180, 0, 1);

            }

            //If the button "x" is pressed on the driver controller, position the robot for extracting specimen from human player
            else if(gamepad1.x){

                //Initialize Robot Controller Settings
                robot.waypointSettings(1, 1, 1,
                        .01575, .020125, .0025, 0,
                        .012, .006025, 0.0025, 0,
                        .35, .035, .0035, 0,
                        .024, .03, 10,
                        .15, .15, .4);

                //Movement
                gripper.moveToTransportPosition();
                robot.toWaypoint(300, 45, 0, 2.5);
                gripper.moveToSpecimenExtractPos();

            }

            //If the button "y" on the driver controller is pressed, position the robot for inserting the specimen below the high bar (for hanging)
            else if(gamepad1.y){

                //Initialize Robot Controller Settings
                robot.waypointSettings(1, 1, 1,
                        .01575, .020125, .0025, 0,
                        .012, .006025, 0.0025, 0,
                        .35, .035, .0035, 0,
                        .024, .03, 10,
                        .15, .15, .4);

                //Movement
                robot.toWaypoint(180, 45, 0, 1.75);
                gripper.moveToHangInsertPosition();
                robot.toWaypoint(180, 90, 0, 0.5);

            }

            //If button "a" on the task controller is pressed, close or open the claw (with alternating function)
            else if(gamepad2.a){
                fL.setPower(0);
                fR.setPower(0);
                bL.setPower(0);
                bR.setPower(0);
                gripper.changeClawState();
            }

            //If button "b" on the task controller is pressed, hang the specimen and back away from the bar
            else if(gamepad2.b){
                fL.setPower(-.05);
                fR.setPower(-.05);
                bL.setPower(-.05);
                bR.setPower(-.05);
                gripper.hang();
                Thread.sleep(1250);
                gripper.changeClawState();
                robot.waypointSettings(1, 1, 1,
                        .01575, .020125, .0025, 0,
                        .012, .006025, 0.0025, 0,
                        .35, .035, .0035, 0,
                        .024, .03, 10,
                        .15, .15, .4);
                robot.toWaypoint(180, 45, 0, .75);
                gripper.moveToTransportPosition();
            }

            //If button "x" on the task controller is pressed, grab the specimen from its position (it was put there by the human player)
            else if(gamepad2.x){
                gripper.moveToSpecimenExtractPos();
                if(!gripper.getCloseState()){gripper.changeClawState();}
                gripper.liftTo(1000);
            }

            //If button "y" on the task controller is pressed, move the robot to the bucket and drop off the sample
            else if(gamepad2.y){
                robot.waypointSettings(1, 1, 1,
                        .01575, .020125, .0025, 0,
                        .012, .006025, 0.0025, 0,
                        .35, .035, .0035, 0,
                        .024, .03, 10,
                        .15, .15, .4);
                gripper.moveToTransportPosition();
                robot.toWaypoint(50, 50, -45, 2);
                gripper.moveToHighBucketPosition();
                Thread.sleep(2500);
                robot.toWaypoint(25, 25, -45, 0.75);
                gripper.changeClawState();
                robot.toWaypoint(50, 50, -45, 1);
                gripper.moveToTransportPosition();
            }

            //If no buttons are pressed on any controller, exhibit normal DriveBase functionality
            else{
                previous = driveBase(robot, previous[0], previous[1]);
            }

        }
    }

    //Normal DriveBase Functionality
    public double[] driveBase(chassis robot, double previousX, double previousY) {

        //Declare Position Variables
        double[] position;
        double gyroAngle;

        //Responsive Control: Local Vectors w/o Rate Limiters
        if(gamepad1.left_bumper){
            double powX;
            double powY;
            double addLeft;
            double addRight;
            if(gamepad1.right_stick_x >= 0){
                powX = Math.pow(Math.abs(gamepad1.right_stick_x), 2);
            }
            else{
                powX = -1.0 * Math.pow(Math.abs(gamepad1.right_stick_x), 2);
            }
            if(gamepad1.right_stick_y >= 0){
                powY = -1 * Math.pow(Math.abs(gamepad1.right_stick_y), 2);
            }
            else{
                powY = Math.pow(Math.abs(gamepad1.right_stick_y), 2);
            }
            if(gamepad1.left_stick_x >= 0){
                addLeft = 0.5 * Math.pow(Math.abs(gamepad1.left_stick_x), 2);
                addRight = -0.5 * Math.pow(Math.abs(gamepad1.left_stick_x), 2);
            }
            else{
                addLeft = -0.5 * Math.pow(Math.abs(gamepad1.left_stick_x), 2);
                addRight = 0.5 * Math.pow(Math.abs(gamepad1.left_stick_x), 2);
            }

            // Send calculated power to wheels
            double a = (powX + powY) * (Math.pow(2, -0.5));
            double b = (-powX + powY) * (Math.pow(2, -0.5));
            fL.setPower((a + addLeft) * frontLeftBias);
            fR.setPower((b + addRight) * frontRightBias);
            bL.setPower((b + addLeft) * backLeftBias);
            bR.setPower((a + addRight) * backRightBias);

            robot.updateOdometry();
            position = robot.getPosition();
            gyroAngle = Math.round(robot.getAngle() * 100.0) / 100.0;
        }

        //Global Control: Global Vectors + Rate Limiters
        else if(gamepad1.right_bumper){
            double powX;
            double powY;
            double addLeft;
            double addRight;
            if(gamepad1.right_stick_x >= 0){
                powX = Math.pow(Math.abs(gamepad1.right_stick_x), 2);
            }
            else{
                powX = -1.0 * Math.pow(Math.abs(gamepad1.right_stick_x), 2);
            }
            if(gamepad1.right_stick_y >= 0){
                powY = -1 * Math.pow(Math.abs(gamepad1.right_stick_y), 2);
            }
            else{
                powY = Math.pow(Math.abs(gamepad1.right_stick_y), 2);
            }
            if(gamepad1.left_stick_x >= 0){
                addLeft = 0.5 * Math.pow(Math.abs(gamepad1.left_stick_x), 2);
                addRight = -0.5 * Math.pow(Math.abs(gamepad1.left_stick_x), 2);
            }
            else{
                addLeft = -0.5 * Math.pow(Math.abs(gamepad1.left_stick_x), 2);
                addRight = 0.5 * Math.pow(Math.abs(gamepad1.left_stick_x), 2);
            }


            if(Math.abs(powX) > Math.abs(previousX)){
                if(powX > previousX && powX - previousX > tolX){
                    powX = previousX + tolX;
                }
                else if(powX < previousX && previousX - powX > tolX){
                    powX = previousX - tolX;
                }
            }
            else{
                if(powX > previousX && powX - previousX > deccelScale * tolX){
                    powX = previousX + deccelScale * tolX;
                }
                else if(powX < previousX && previousX - powX > deccelScale * tolX){
                    powX = previousX - deccelScale * tolX;
                }
            }
            if(Math.abs(powY) > Math.abs(previousY)){
                if(powY > previousY && powY - previousY > tolY){
                    powY = previousY + tolY;
                }
                else if(powY < previousY && previousY - powY > tolY){
                    powY = previousY - tolY;
                }
            }
            else{
                if(powY > previousY && powY - previousY > deccelScale * tolY){
                    powY = previousY + deccelScale * tolY;
                }
                else if(powY < previousY && previousY - powY > deccelScale * tolY){
                    powY = previousY - deccelScale * tolY;
                }
            }
            double theta = robot.getPosition()[2];
            double tempPowX = (powY * Math.sin(theta) + powX * Math.cos(theta));
            double tempPowY = (powY * Math.cos(theta) + powX * Math.sin(-theta));

            previousX = powX;
            previousY = powY;

            powY = tempPowY;
            powX = tempPowX;

            // Send calculated power to wheels
            double a = (powX + powY) * (Math.pow(2, -0.5));
            double b = (-powX + powY) * (Math.pow(2, -0.5));
            fL.setPower((a + addLeft) * frontLeftBias);
            fR.setPower((b + addRight) * frontRightBias);
            bL.setPower((b + addLeft) * backLeftBias);
            bR.setPower((a + addRight) * backRightBias);

            robot.updateOdometry();
            position = robot.getPosition();
            gyroAngle = Math.round(robot.getAngle() * 100.0) / 100.0;
        }

        //Local Y-Lock w/o Rate Limiters:
        else if(gamepad1.right_trigger != 0){

            double powX = 0;
            double powY;
            double addLeft = 0;
            double addRight = 0;

            if(gamepad1.right_stick_y >= 0){
                powY = -1 * Math.pow(Math.abs(gamepad1.right_stick_y), 2);
            }
            else{
                powY = Math.pow(Math.abs(gamepad1.right_stick_y), 2);
            }

            // Send calculated power to wheels
            double a = (powX + powY) * (Math.pow(2, -0.5));
            double b = (-powX + powY) * (Math.pow(2, -0.5));
            fL.setPower((a + addLeft) * frontLeftBias);
            fR.setPower((b + addRight) * frontRightBias);
            bL.setPower((b + addLeft) * backLeftBias);
            bR.setPower((a + addRight) * backRightBias);

            robot.updateOdometry();
            position = robot.getPosition();
            gyroAngle = Math.round(robot.getAngle() * 100.0) / 100.0;
        }

        //Local X-Lock w/o Rate Limiters:
        else if(gamepad1.left_trigger != 0){

            double powX;
            double powY = 0;
            double addLeft = 0;
            double addRight = 0;

            if(gamepad1.right_stick_x >= 0){
                powX = Math.pow(Math.abs(gamepad1.right_stick_x), 2);
            }
            else{
                powX = -1.0 * Math.pow(Math.abs(gamepad1.right_stick_x), 2);
            }

            // Send calculated power to wheels
            double a = (powX + powY) * (Math.pow(2, -0.5));
            double b = (-powX + powY) * (Math.pow(2, -0.5));
            fL.setPower((a + addLeft) * frontLeftBias);
            fR.setPower((b + addRight) * frontRightBias);
            bL.setPower((b + addLeft) * backLeftBias);
            bR.setPower((a + addRight) * backRightBias);

            robot.updateOdometry();
            position = robot.getPosition();
            gyroAngle = Math.round(robot.getAngle() * 100.0) / 100.0;
        }

        //Default Control: Local Vectors + Rate Limiters
        else{

            double powX;
            double powY;
            double addLeft;
            double addRight;
            if(gamepad1.right_stick_x >= 0){
                powX = Math.pow(Math.abs(gamepad1.right_stick_x), 2);
            }
            else{
                powX = -1.0 * Math.pow(Math.abs(gamepad1.right_stick_x), 2);
            }
            if(gamepad1.right_stick_y >= 0){
                powY = -1 * Math.pow(Math.abs(gamepad1.right_stick_y), 2);
            }
            else{
                powY = Math.pow(Math.abs(gamepad1.right_stick_y), 2);
            }
            if(gamepad1.left_stick_x >= 0){
                addLeft = 0.5 * Math.pow(Math.abs(gamepad1.left_stick_x), 2);
                addRight = -0.5 * Math.pow(Math.abs(gamepad1.left_stick_x), 2);
            }
            else{
                addLeft = -0.5 * Math.pow(Math.abs(gamepad1.left_stick_x), 2);
                addRight = 0.5 * Math.pow(Math.abs(gamepad1.left_stick_x), 2);
            }

            if(Math.abs(powX) > Math.abs(previousX)){
                if(powX > previousX && powX - previousX > tolX){
                    powX = previousX + tolX;
                }
                else if(powX < previousX && previousX - powX > tolX){
                    powX = previousX - tolX;
                }
            }
            else{
                if(powX > previousX && powX - previousX > deccelScale * tolX){
                    powX = previousX + deccelScale * tolX;
                }
                else if(powX < previousX && previousX - powX > deccelScale * tolX){
                    powX = previousX - deccelScale * tolX;
                }
            }
            if(Math.abs(powY) > Math.abs(previousY)){
                if(powY > previousY && powY - previousY > tolY){
                    powY = previousY + tolY;
                }
                else if(powY < previousY && previousY - powY > tolY){
                    powY = previousY - tolY;
                }
            }
            else{
                if(powY > previousY && powY - previousY > deccelScale * tolY){
                    powY = previousY + deccelScale * tolY;
                }
                else if(powY < previousY && previousY - powY > deccelScale * tolY){
                    powY = previousY - deccelScale * tolY;
                }
            }

            // Send calculated power to wheels
            double a = (powX + powY) * (Math.pow(2, -0.5));
            double b = (-powX + powY) * (Math.pow(2, -0.5));
            fL.setPower((a + addLeft) * frontLeftBias);
            fR.setPower((b + addRight) * frontRightBias);
            bL.setPower((b + addLeft) * backLeftBias);
            bR.setPower((a + addRight) * backRightBias);

            robot.updateOdometry();
            position = robot.getPosition();
            gyroAngle = Math.round(robot.getAngle() * 100.0) / 100.0;

            previousX = powX;
            previousY = powY;
        }

        // DriveBase Data Update (Via Telemetry)
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("X:", "" + Math.round(position[0] * 100.0) / 100.0);
        telemetry.addData("Y:", "" + Math.round(position[1] * 100.0) / 100.0);
        telemetry.addData("Theta:", "" + (Math.round(position[2] * 100.0) / 100.0) * 180.0 / Math.PI);
        telemetry.addData("Gyro Angle:", "" + gyroAngle * 180.0 / Math.PI);
        telemetry.update();
        return (new double[] {previousX, previousY});
    }
}