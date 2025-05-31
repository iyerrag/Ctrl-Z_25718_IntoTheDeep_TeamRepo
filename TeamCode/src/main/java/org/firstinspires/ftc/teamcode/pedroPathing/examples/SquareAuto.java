package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous
public class SquareAuto extends OpMode {

    private Follower follower;

    private Timer pathTimer;
    private Timer opmodeTimer;

    private int pathState;

    //Key Robot Positions:
    Pose startPose = new Pose(0, 0, 0);
    Pose firstCorner = new Pose(24, 0, Math.toRadians(45));
    Pose secondCorner = new Pose(24, 48, -Math.toRadians(45));
    Pose thirdCorner = new Pose(0, 48, Math.toRadians(45));

    Pose finalTurn = new Pose(0, 0 , Math.toRadians(90));

    private Path firstSide, secondSide, thirdSide, fourthSide, turn;



    public void buildPaths(){

        firstSide = new Path(new BezierLine(new Point(startPose), new Point(firstCorner)));
        firstSide.setLinearHeadingInterpolation(0, Math.toRadians(45));

        secondSide = new Path(new BezierLine(new Point(firstCorner), new Point(secondCorner)));
        secondSide.setLinearHeadingInterpolation(Math.toRadians(45), -Math.toRadians(45));

        thirdSide = new Path(new BezierLine(new Point(secondCorner), new Point(thirdCorner)));
        thirdSide.setLinearHeadingInterpolation(-Math.toRadians(45), Math.toRadians(45));

        fourthSide = new Path(new BezierLine(new Point(thirdCorner), new Point(startPose)));
        fourthSide.setLinearHeadingInterpolation(Math.toRadians(45), 0);

        turn = new Path(new BezierLine(new Point(startPose), new Point(finalTurn)));
        turn.setLinearHeadingInterpolation(0, Math.toRadians(90));
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                if(!follower.isBusy()){
                    follower.followPath(firstSide, true);
                    setPathState(1);
                }

                break;

            case 1:

                if(!follower.isBusy()){
                    follower.followPath(secondSide, true);
                    setPathState(2);
                }

                break;

            case 2:

                if(!follower.isBusy()){
                    follower.followPath(thirdSide, true);
                    setPathState(3);
                }

                break;

            case 3:

                if(!follower.isBusy()){
                    follower.followPath(fourthSide, true);
                    setPathState(4);
                }

                break;

            case 4:

                if(!follower.isBusy()){
                    follower.followPath(turn, true);
                    setPathState(5);
                }
                break;

            case 5:

                if(!follower.isBusy()) {

                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose(0,0,0));
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}
