/*
 * Copyright (c) 2024 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.autocommands;

import teamcode.FtcAuto;
import teamcode.Robot;

import teamcode.vision.Vision;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcStateMachine;
import trclib.timer.TrcTimer;
import trclib.pathdrive.TrcPose2D;

/**
 * This class implements an autonomous strategy.
 */
public class SampleAuto implements TrcRobot.RobotCommand
{
    private final String moduleName = getClass().getSimpleName();

    private final TrcPose2D sampleScorePose = new TrcPose2D();
    private final TrcPose2D firstSamplePickupPose = new TrcPose2D();
    private final TrcPose2D secondSamplePickupPose = new TrcPose2D();
    private final TrcPose2D thirdSamplePickupPose = new TrcPose2D();
    private final TrcPose2D ascentPose = new TrcPose2D();
    private enum State
    {
        START,
        MOVE_TO_SAMPLE_SCORE_POSE,
        SCORE_SAMPLE,
        MOVE_TO_FIRST_SAMPLE_PICKUP,
        PICKUP_SAMPLE,
        MOVE_TO_SECOND_SAMPLE_PICKUP,
        MOVE_TO_THIRD_SAMPLE_PICKUP,
        MOVE_TO_ASCENT,
        EXTEND_ARM,
        DONE
    }   //enum FirstState

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> SM;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies the autoChoices object.
     */
    public SampleAuto(Robot robot, FtcAuto.AutoChoices autoChoices)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;

        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        SM = new TrcStateMachine<>(moduleName);

        robot.vision.setSampleVisionEnabled(Vision.SampleType.YellowSample, true);

        SM.start(State.START);
    }   //CmdAuto

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method checks if the current RobotCommand  is running.
     *
     * @return true if the command is running, false otherwise.
     */
    @Override
    public boolean isActive()
    {
        return SM.isEnabled();
    }   //isActive

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel()
    {
        timer.cancel();
        SM.stop();
    }   //cancel



    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        State state = SM.checkReadyAndGetState();

        if (state == null)
        {
            robot.dashboard.displayPrintf(8, "State: disabled or waiting (nextState=" + SM.getNextState() + ")...");
            robot.dashboard.displayPrintf(9, "Robot Pose: (%f, %f, %f)",
                    robot.robotDrive.driveBase.getXPosition(),
                    robot.robotDrive.driveBase.getYPosition(),
                    robot.robotDrive.driveBase.getHeading());
        }
        else {
            robot.dashboard.displayPrintf(8, "State: " + state);
            robot.globalTracer.tracePreStateInfo(SM.toString(), state);
            robot.dashboard.displayPrintf(9, "Robot Pose: (%f, %f, %f)",
                    robot.robotDrive.driveBase.getXPosition(),
                    robot.robotDrive.driveBase.getYPosition(),
                    robot.robotDrive.driveBase.getHeading());
            switch (state) {
                case START:
                    if (autoChoices.delay > 0.0) {
                        timer.set(autoChoices.delay, event);
                        //robot.LClaw.setLogicalPosition(0.5);
                        //robot.RClaw.setLogicalPosition(0);
                        SM.waitForSingleEvent(event, State.MOVE_TO_SAMPLE_SCORE_POSE);
                    }
                    else {
                        //robot.LClaw.setLogicalPosition(0.5);
                        //robot.RClaw.setLogicalPosition(0);
                        SM.setState(State.MOVE_TO_SAMPLE_SCORE_POSE);
                    }
                    break;
                case DONE:
                    cancel();
                    break;
                default:
            }

            robot.globalTracer.tracePostStateInfo(
                    SM.toString(), state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                    robot.robotDrive.purePursuitDrive, null);
        }

        return !SM.isEnabled();
    }   //cmdPeriodic

}   //class CmdAuto
