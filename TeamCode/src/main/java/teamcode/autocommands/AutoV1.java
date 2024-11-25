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

import com.qualcomm.robotcore.hardware.DcMotor;

import teamcode.FtcAuto;
import teamcode.Robot;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcStateMachine;
import trclib.timer.TrcTimer;

/**
 * This class implements an autonomous strategy.
 */
public class AutoV1 implements TrcRobot.RobotCommand
{
    private final String moduleName = getClass().getSimpleName();

    private enum State
    {
        START,
        MOVE,
        DONE
    }   //enum State

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies the autoChoices object.
     */
    public AutoV1(Robot robot, FtcAuto.AutoChoices autoChoices)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;

        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START);
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
        return sm.isEnabled();
    }   //isActive

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel()
    {
        timer.cancel();
        sm.stop();
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
        State state = sm.checkReadyAndGetState();

        if (state == null)
        {
            robot.dashboard.displayPrintf(8, "State: disabled or waiting (nextState=" + sm.getNextState() + ")...");
        }
        else
        {
            robot.dashboard.displayPrintf(8, "State: " + state);
            robot.globalTracer.tracePreStateInfo(sm.toString(), state);
            switch (state)
            {
                case START:
                    if (autoChoices.delay > 0.0)
                    {
                        robot.globalTracer.traceInfo(moduleName, "***** Do delay " + autoChoices.delay + "s.");
                        timer.set(autoChoices.delay, event);
                        sm.waitForSingleEvent(event, State.MOVE);
                    }
                    else
                    {
                        sm.setState(State.MOVE);
                    }
                    break;

                default:

                case MOVE:
                    //TrcPose2D point = new TrcPose2D(0,0,0);
                    //TrcPose2D point1 = new TrcPose2D(0,3,0);

                    robot.robotDrive.driveBase.holonomicDrive(0.0, 0.5, 0.0, 1.0, event);

                    robot.arm.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    robot.arm.motor.setTargetPosition(0);
                    robot.arm.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    robot.arm.motor.setTargetPosition(75);
                    robot.arm.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setMotorPower(0.5);

                    robot.arm.motor.setTargetPosition(95);
                    robot.arm.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setMotorPower(0.75);

                    robot.Lclaw.setLogicalPosition(0);
                    robot.Rclaw.setLogicalPosition(1);

                    robot.arm.motor.setTargetPosition(0);
                    robot.arm.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setMotorPower(0.5);

                    robot.robotDrive.driveBase.holonomicDrive(0.0, -0.5, 0.0, 1.0, event);
                    robot.robotDrive.driveBase.holonomicDrive(0.5, 0.0, 0.0, 2.0, event);

                    //robot.robotDrive.purePursuitDrive.start(event, 10, point, false, point1);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                    // We are done.
                    cancel();
                    break;
            }

            robot.globalTracer.tracePostStateInfo(
                sm.toString(), state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                robot.robotDrive.purePursuitDrive, null);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdAuto
