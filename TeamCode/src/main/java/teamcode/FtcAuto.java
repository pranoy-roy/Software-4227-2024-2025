/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
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

package teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Locale;

import ftclib.driverio.FtcChoiceMenu;
import ftclib.driverio.FtcMatchInfo;
import ftclib.driverio.FtcMenu;
import ftclib.driverio.FtcValueMenu;
import ftclib.robotcore.FtcOpMode;

import teamcode.autocommands.SampleAuto;
import teamcode.autocommands.SpecimenAuto;
import trclib.command.CmdPidDrive;
import trclib.command.CmdTimedDrive;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcRobot;
import trclib.timer.TrcTimer;

/**
 * This class contains the Autonomous Mode program.
 */
@Autonomous(name="FtcAutonomous", group="Ftcxxxx")
public class FtcAuto extends FtcOpMode
{
    private final String moduleName = getClass().getSimpleName();

    public enum Alliance
    {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance

    public enum AutoStrategy
    {
        SampleAuto,
        SpecimenAuto,
        TimedDrive
    }   //enum AutoStrategy

    /**
     * This class stores the autonomous menu choices.
     */
    public static class AutoChoices
    {
        public double delay = 0.0;
        public Alliance alliance = Alliance.RED_ALLIANCE;
        public AutoStrategy strategy = AutoStrategy.SampleAuto;
        public double driveTime = 0.0;
        public double drivePower = 0.0;

        @NonNull
        @Override
        public String toString()
        {
            return String.format(
                Locale.US,
                "delay=%.0f " +
                "alliance=\"%s\" " +
                "strategy=\"%s\" " +
                "driveTime=%.0f " +
                "drivePower=%.1f",
                delay, alliance, strategy, driveTime, drivePower);
        }   //toString

    }   //class AutoChoices

    public static final AutoChoices autoChoices = new AutoChoices();
    private Robot robot;
    private TrcRobot.RobotCommand autoCommand;

    //
    // Implements FtcOpMode abstract method.
    //

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station is pressed.
     */
    @Override
    public void robotInit()
    {
        //
        // Create and initialize robot object.
        //
        robot = new Robot(TrcRobot.getRunMode());
        //
        // Open trace log.
        //
        if (RobotParams.Preferences.useTraceLog)
        {
            Robot.matchInfo = FtcMatchInfo.getMatchInfo();
            String filePrefix = String.format(
                Locale.US, "%s%02d_Auto", Robot.matchInfo.matchType, Robot.matchInfo.matchNumber);
            TrcDbgTrace.openTraceLog(RobotParams.Robot.LOG_FOLDER_PATH, filePrefix);
        }
        //
        // Create and run choice menus.
        //
        doAutoChoicesMenus();
        //
        // Create autonomous command according to chosen strategy.
        //
        switch (autoChoices.strategy)
        {
            case TimedDrive:
                if (robot.robotDrive != null)
                {
                    autoCommand = new CmdTimedDrive(
                        robot.robotDrive.driveBase, autoChoices.delay, autoChoices.driveTime,
                        0.0, autoChoices.drivePower, 0.0);
                }
                break;
            case SampleAuto:
                autoCommand = new SampleAuto(robot, autoChoices);
                //robot.LClaw.setLogicalPosition(0.5);
                //robot.RClaw.setLogicalPosition(0);
                break;
            case SpecimenAuto:
                autoCommand = new SpecimenAuto(robot, autoChoices);
                //robot.LClaw.setLogicalPosition(0.5);
                //robot.RClaw.setLogicalPosition(0);
                break;
            default:
                autoCommand = null;
                break;
        }



        robot.zeroCalibrate();
    }   //robotInit

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    /**
     * This method is called periodically after robotInit() is called but before competition starts. For example,
     * we can put vision code here to detect target before autonomous starts.
     */
    @Override
    public void initPeriodic()
    {
    }   //initPeriodic

    /**
     * This method is called when the competition mode is about to start. In FTC, this is called when the "Play"
     * button on the Driver Station is pressed. Typically, you put code that will prepare the robot for start of
     * competition here such as resetting the encoders/sensors and enabling some sensors to start sampling.
     *
     * @param prevMode specifies the previous RunMode it is coming from (always null for FTC).
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        if (TrcDbgTrace.isTraceLogOpened())
        {
            TrcDbgTrace.setTraceLogEnabled(true);
        }
        robot.globalTracer.traceInfo(
            moduleName, "***** Starting autonomous: " + TrcTimer.getCurrentTimeString() + " *****");
        if (Robot.matchInfo != null)
        {
            robot.globalTracer.logInfo(moduleName, "MatchInfo", Robot.matchInfo.toString());
        }
        robot.globalTracer.logInfo(moduleName, "AutoChoices", autoChoices.toString());
        robot.dashboard.clearDisplay();
        //
        // Tell robot object opmode is about to start so it can do the necessary start initialization for the mode.
        //
        robot.startMode(nextMode);

        if (robot.battery != null)
        {
            robot.battery.setEnabled(true);
        }
    }   //startMode



    /**
     * This method is called when competition mode is about to end. Typically, you put code that will do clean
     * up here such as disabling the sampling of some sensors.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into (always null for FTC).
     */
    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        //
        // Opmode is about to stop, cancel autonomous command in progress if any.
        //
        if (autoCommand != null)
        {
            autoCommand.cancel();
        }
        //
        // Tell robot object opmode is about to stop so it can do the necessary cleanup for the mode.
        //
        robot.stopMode(prevMode);

        if (robot.battery != null)
        {
            robot.battery.setEnabled(false);
        }

        printPerformanceMetrics();
        robot.globalTracer.traceInfo(
            moduleName, "***** Stopping autonomous: " + TrcTimer.getCurrentTimeString() + " *****");

        if (TrcDbgTrace.isTraceLogOpened())
        {
            TrcDbgTrace.closeTraceLog();
        }
    }   //stopMode

    /**
     * This method is called periodically on the main robot thread. Typically, you put TeleOp control code here that
     * doesn't require frequent update For example, TeleOp joystick code or status display code can be put here since
     * human responses are considered slow.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        if (autoCommand != null)
        {
            //
            // Run the autonomous command.
            //
            autoCommand.cmdPeriodic(elapsedTime);
        }
    }   //periodic

    /**
     * This method creates the autonomous menus, displays them and stores the choices.
     */
    private void doAutoChoicesMenus()
    {
        //
        // Construct menus.
        //

        FtcValueMenu delayMenu = new FtcValueMenu("Delay time:", null, 0.0, 30.0, 1.0, 0.0, " %.0f sec");
        FtcChoiceMenu<Alliance> allianceMenu = new FtcChoiceMenu<>("Alliance:", delayMenu);
        FtcChoiceMenu<AutoStrategy> strategyMenu = new FtcChoiceMenu<>("Auto Strategies:", allianceMenu);

        FtcValueMenu driveTimeMenu = new FtcValueMenu(
            "Drive time:", strategyMenu, 0.0, 30.0, 1.0, 5.0, " %.0f sec");
        FtcValueMenu drivePowerMenu = new FtcValueMenu(
            "Drive power:", strategyMenu, -1.0, 1.0, 0.1, 0.5, " %.1f");

        // Link Value Menus to their children.
        delayMenu.setChildMenu(allianceMenu);
        driveTimeMenu.setChildMenu(drivePowerMenu);
        //
        // Populate choice menus.
        //
        allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE, true, delayMenu);
        allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE, false, delayMenu);

        strategyMenu.addChoice("Sample Auto", AutoStrategy.SampleAuto, false);
        strategyMenu.addChoice("Specimen Auto", AutoStrategy.SpecimenAuto, false);

        strategyMenu.addChoice("Timed Drive", AutoStrategy.TimedDrive, false, driveTimeMenu);
        //
        // Traverse menus.
        //
        FtcMenu.walkMenuTree(delayMenu);
        //
        // Fetch choices.
        //
        autoChoices.delay = delayMenu.getCurrentValue();
        autoChoices.alliance = allianceMenu.getCurrentChoiceObject();
        autoChoices.strategy = strategyMenu.getCurrentChoiceObject();
        autoChoices.driveTime = driveTimeMenu.getCurrentValue();
        autoChoices.drivePower = drivePowerMenu.getCurrentValue();
        //
        // Show choices.
        //
        robot.dashboard.displayPrintf(1, "Auto Choices: %s", autoChoices);
    }   //doAutoChoicesMenus

}   //class FtcAuto
