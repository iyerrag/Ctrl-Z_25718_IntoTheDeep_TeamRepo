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


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode liTt
 */


@TeleOp
public class BasicOpMode_Linear extends LinearOpMode {

    private static final double tolX = .05;
    private static final double tolY = .05;
    private static final double tolTheta = 1;
    private static final double deccelScale = 3.0;

    static final double frontLeftBias = 0.97;
    static final double frontRightBias = 0.92;
    static final double backLeftBias = 1.0;
    static final double backRightBias = 0.96;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fL;
    private DcMotor fR;
    private DcMotor bL;
    private DcMotor bR;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fL = hardwareMap.get(DcMotor.class, "FrontLeft");
        fR = hardwareMap.get(DcMotor.class, "FrontRight");
        bL = hardwareMap.get(DcMotor.class, "BackLeft");
        bR = hardwareMap.get(DcMotor.class, "BackRight");
        BHI260IMU IMU = hardwareMap.get(BHI260IMU.class, "imu");
        VoltageSensor voltmeter = hardwareMap.voltageSensor.iterator().next();
        WebcamName myCamera = hardwareMap.get(WebcamName.class, "Webcam 1");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        chassis robot = new chassis(fL, fR, bL, bR, IMU, "IMU", 0, 0, 0, voltmeter, myCamera, new double[]{14.605, 32.385, 0});
        claw gripper = new claw(hardwareMap.get(Servo.class, "wrist"), hardwareMap.get(Servo.class, "leftTalon"), hardwareMap.get(Servo.class, "rightTalon"), hardwareMap.get(CRServo.class,
                "intakeServo"), hardwareMap.get(DcMotor.class, "lifterLeft"), hardwareMap.get(DcMotor.class, "lifterRight"), hardwareMap.get(DcMotor.class, "elbow"));
        double[] position = new double[3];
        double[] differentials = new double[3];
        double primes;
        double gyroAngle = 0.0;
        double[] storage = new double[] {0, 0, 0, 0};
        double previousX = 0;
        double previousY = 0;
        double previousTurn = 0;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        double[] ab = {0, 0, 0, 0, 0, 0};

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.
            if(gamepad2.left_trigger == 1){
                while(gamepad2.left_trigger == 1){
                    gripper.lift(-1);
                }
                gripper.hold();
            }
            else if(gamepad2.left_bumper){
                while(gamepad2.left_bumper){
                    gripper.lift(1);
                }
                gripper.hold();
            }
            /*else if(gamepad2.left_bumper){
                gripper.wristRotateTo((double) Math.abs(Math.round(gamepad2.left_stick_y * 1000)) / 1000);
            }*/
            /*else if(gamepad2.y){
                if(gripper.state()){
                    //gripper.open();
                }
                else{
                    //gripper.close();
                }
            }*/
            else if(gamepad2.right_trigger == 1){
                while(gamepad2.right_trigger == 1){
                    gripper.rotateElbow(0.50);
                }
                gripper.hold();
            }
            else if(gamepad2.right_bumper){
                while(gamepad2.right_bumper){
                    gripper.rotateElbow(-0.50);
                }
                gripper.hold();
            }
            else if(gamepad2.a){
                gripper.intakeRotate("IN");
               /* gripper.wristRotateTo(0.12);
                gripper.elbowTo(-4600, 0.5);
                gripper.liftTo(0);
                robot.toWaypoint(robot.getPosition()[0] - 15, robot.getPosition()[1], robot.getPosition()[2], 0.5);
                gripper.elbowTo(-4900, 0.5);
                // Placeholder for intake
                Thread.sleep(500);
                gripper.elbowTo(4600, 0.5);
                robot.toWaypoint(robot.getPosition()[0] + 15, robot.getPosition()[1], robot.getPosition()[2], 0.5);*/

            }
            else if(gamepad2.b){
                gripper.intakeRotate("OUT");
                /*gripper.liftTo(600);
                gripper.elbowTo(2600, 0.5);
                gripper.wristRotateTo(0);*/
            }
            else if(gamepad2.x){
                gripper.leftTalonRotateTo(0.1);
                gripper.rightTalonRotateTo(0.8);
            }
            else if(gamepad2.y){
                gripper.leftTalonRotateTo(.43);
                gripper.rightTalonRotateTo(0.47);
            }
            else if(gamepad1.left_bumper){
                //Responsive Control: Local Vectors w/o Rate Limiters
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

                // telemetry.addData("dx: ", "" + temp[0]);

                //telemetry.addData("dy: ", "" + temp[1]);

                // telemetry.addData("dTheta: ", "" + temp[2]);
            }
            else if(gamepad1.right_bumper){
                //Global Control: Global Vectors + Rate Limiters
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


                // telemetry.addData("dx: ", "" + temp[0]);

                //telemetry.addData("dy: ", "" + temp[1]);

                // telemetry.addData("dTheta: ", "" + temp[2]);
            }
            /*&if else(gamepad2.left_stick_y != 0){
                claw.close();
                claw.liftTo(gamepad2.)

            }*/
            else if(gamepad1.right_trigger != 0){
                //Local Y-Lock w/o Rate Limiters:
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
            else if(gamepad1.left_trigger != 0){
                //Local X-Lock w/o Rate Limiters:
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
            else{
                //Default Control: Local Vectors + Rate Limiters
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


                // telemetry.addData("dx: ", "" + temp[0]);

                //telemetry.addData("dy: ", "" + temp[1]);

                // telemetry.addData("dTheta: ", "" + temp[2]);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());


            telemetry.addData("X:", "" + Math.round(position[0] * 100.0) / 100.0);
            telemetry.addData("Y:", "" + Math.round(position[1] * 100.0) / 100.0);
            telemetry.addData("Theta:", "" + (Math.round(position[2] * 100.0) / 100.0) * 180.0 / Math.PI);
            telemetry.addData("Gyro Angle:", "" + gyroAngle * 180.0 / Math.PI);
            telemetry.addData("Optimal Clamp: ", "" + storage[0]);
            telemetry.addData("Battery Voltage Reading: ", "" + storage[1]);
            telemetry.addData("Elbow Pos:", gripper.getElbowPos());
            /*telemetry.addData("globalCorrectionX: ", "" + ab[0]);
            telemetry.addData("globalCorrectionY: ", "" + ab[1]);
            telemetry.addData("correctionX: ", "" + ab[2]);
            telemetry.addData("correctionY: ", "" + ab[3]);
            telemetry.addData("correctionVectorMagnitude: ", "" + ab[4]);
            telemetry.addData("correctionVectorAngle: ", "" + ab[5]);*/

            /*telemetry.addData("dx:", "" + differentials[0]);
            telemetry.addData("dy:", "" + differentials[1]);
            telemetry.addData("dtheta:", "" + differentials[2]);
            telemetry.addData("leftCount':", "" + primes);
            telemetry.addData("y':", "" + primes[1]);
            telemetry.addData("theta':", "" + primes[2]);
            telemetry.addData("", "" + c[0][0] + "" + c[0][1] + "" + c[0][2] + "" + c[0][3]);
            telemetry.addData("", "" + c[1][0] + "" + c[1][1] + "" + c[1][2] + "" + c[1][3]);
            telemetry.addData("", "" + c[2][0] + "" + c[2][1] + "" + c[2][2] + "" + c[2][3]);
            telemetry.addData("", "" + c[3][0] + "" + c[3][1] + "" + c[3][2] + "" + c[3][3]);*/

            telemetry.update();
        }
    }
}