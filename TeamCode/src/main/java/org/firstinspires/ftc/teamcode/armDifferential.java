package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

public class armDifferential {

    private Servo leftServo;
    private Servo rightServo;

    private double inputToOutputGearRatio;

    private double originPosition_leftServo;
    private double originPosition_rightServo;

    public armDifferential(Servo leftServo, Servo rightServo, double originPosition_leftServo, double originPosition_rightServo, double inputToOutputGearRatio){
        this.leftServo = leftServo;
        this.rightServo = rightServo;
        this.originPosition_leftServo = originPosition_leftServo;
        this.originPosition_rightServo = originPosition_rightServo;
        this.inputToOutputGearRatio = inputToOutputGearRatio;
    }

    public double getLeftServoPos(){
        return leftServo.getPosition();
    }

    public double getRightServoPos(){
        return rightServo.getPosition();
    }

    private double thetaPositionToServoPosition_leftServo(double theta){
        return originPosition_leftServo + (theta / 300.0);
    }
    private double thetaPositionToServoPosition_rightServo(double theta){
        return originPosition_rightServo + (theta / 300.0);
    }

    private boolean isViableTarget_leftServo(double target){
        return Math.abs(thetaPositionToServoPosition_leftServo(target) - 0.5) <= 0.5;
    }

    private boolean isViableTarget_rightServo(double target){
        return Math.abs(thetaPositionToServoPosition_rightServo(target) - 0.5) <= 0.5;
    }

    private boolean isViableTarget(double pitchPosition, double rollPosition){
        double theta_left = rollPosition - pitchPosition;
        double theta_right = rollPosition + pitchPosition;
        return isViableTarget_leftServo(theta_left) && isViableTarget_rightServo(theta_right);
    }

    private double getAngularPosition_leftServo(){
        return ((leftServo.getPosition() - originPosition_leftServo) * 300.0);
    }

    private double getAngularPosition_rightServo(){
        return ((rightServo.getPosition() - originPosition_rightServo) * 300.0);
    }

    public double getAngularPosition_Roll(){

        double theta_left = getAngularPosition_leftServo();
        double theta_right = getAngularPosition_rightServo();

        return (theta_left + theta_right) / 2;

        /*double sign_left = (theta_left / Math.abs(theta_left));
        double sign_right = (theta_right / Math.abs(theta_right));

        if(sign_left == sign_right){
            return (0.5 * sign_right * Math.abs(theta_left - theta_right));
        }
        else{
            return (((theta_right - theta_left) / 2) - 0.5 * sign_right * Math.abs(theta_left + theta_right));
        }*/
    }

    public double getAngularPosition_Pitch(){
        double theta_left = getAngularPosition_leftServo();
        double theta_right =getAngularPosition_rightServo();

        /*double sign_left = (theta_left / Math.abs(theta_left));
        double sign_right = (theta_right / Math.abs(theta_right));

        if(sign_left == sign_right){
            return (((theta_right + theta_left) / 2) - 0.5 * sign_right * Math.abs(theta_left - theta_right));
        }
        else{
            return (0.5 * sign_right * Math.abs(theta_left + theta_right));
        }*/

        return (theta_right - theta_left) / 2;
    }

    public void setAngularPosition_leftServo(double targetTheta){
        leftServo.setPosition(thetaPositionToServoPosition_leftServo(targetTheta));
    }

    public void setAngularPosition_rightServo(double targetTheta){
        rightServo.setPosition(thetaPositionToServoPosition_rightServo(targetTheta));
    }

    public void setAngularPosition(double pitchPosition, double rollPosition){
        /*if(Math.abs(pitchPosition) > Math.abs(rollPosition)){
            double theta_left = pitchPosition + rollPosition;
            double theta_right = pitchPosition - rollPosition;
        }
        else{
            double theta_left = -1.0 * (pitchPosition + rollPosition);
            double theta_right = pitchPosition - rollPosition;
        }*/

        double theta_left = rollPosition - pitchPosition;
        double theta_right = rollPosition + pitchPosition;

        setAngularPosition_leftServo(theta_left);
        setAngularPosition_rightServo(theta_right);
    }

    public void moveToAngularPosition(double pitchPosition, double rollPosition){}

}
