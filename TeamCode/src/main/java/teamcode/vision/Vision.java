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

package teamcode.vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import ftclib.drivebase.FtcRobotDrive;
import ftclib.robotcore.FtcOpMode;
import ftclib.vision.FtcCameraStreamProcessor;
import ftclib.vision.FtcEocvColorBlobProcessor;
import ftclib.vision.FtcLimelightVision;
import ftclib.vision.FtcRawEocvColorBlobPipeline;
import ftclib.vision.FtcRawEocvVision;
import ftclib.vision.FtcVision;
import ftclib.vision.FtcVisionAprilTag;
import ftclib.vision.FtcVisionEocvColorBlob;
import teamcode.Robot;
import teamcode.subsystems.LEDIndicator;
import teamcode.RobotParams;
import trclib.dataprocessor.TrcUtil;
import trclib.pathdrive.TrcPose2D;
import trclib.pathdrive.TrcPose3D;
import trclib.robotcore.TrcDbgTrace;
import trclib.vision.TrcHomographyMapper;
import trclib.vision.TrcOpenCvColorBlobPipeline;
import trclib.vision.TrcOpenCvDetector;
import trclib.vision.TrcVisionTargetInfo;

/**
 * This class implements AprilTag/Eocv/Limelight Vision for the game season. It creates and initializes all the vision
 * target info as well as providing info for the robot, camera and the field. It also provides methods to get the
 * location of the robot and detected targets.
 */
public class Vision
{
    private final String moduleName = getClass().getSimpleName();

    /**
     * This class contains the parameters of the webcam for detecting samples.
     */
    public static class SampleCamParams extends FtcRobotDrive.VisionInfo
    {
        public SampleCamParams()
        {
            camName = "Webcam 1";
            camImageWidth = 640;
            camImageHeight = 480;
            camXOffset = -0.5;              // Inches to the right from robot center
            camYOffset = 25.5;                   // Inches forward from robot center
            camZOffset = 2.375;                // Inches up from the floor
            camYaw = 0.0;                      // degrees clockwise from robot forward
            camPitch = -98.5;           // degrees up from horizontal
            camRoll = 0.0;
            camPose = new TrcPose3D(camXOffset, camYOffset, camZOffset, camYaw, camPitch, camRoll);
            camOrientation = OpenCvCameraRotation.UPRIGHT;
            // Homography: cameraRect in pixels, worldRect in inches
            cameraRect = new TrcHomographyMapper.Rectangle(
                    14.0, 28.0,                     // Camera Top Left
                    612.0, 33.0,                    // Camera Top Right
                    56.0, 448.0,                    // Camera Bottom Left
                    581.0, 430.5);                  // Camera Bottom Right
            worldRect = new TrcHomographyMapper.Rectangle(
                    -19.0, 37.5,                    // World Top Left
                    24.0, 37.5,                     // World Top Right
                    -4.75, 9.0,                     // World Bottom Left
                    6.25, 9.0);                     // World Bottom Right
        }   //SampleCamParams
    }   //class SampleCamParams

    /**
     * This class contains the parameters of the Limelight vision processor.
     */
    public static class LimelightParams extends FtcRobotDrive.VisionInfo
    {
        public LimelightParams()
        {
            camName = "Limelight3a";
            camImageWidth = 640;
            camImageHeight = 480;
//            camHFov = 80.0;                             // in degrees
//            camVFov = 56.0;                             // in degrees
            camXOffset = 135.47*TrcUtil.INCHES_PER_MM;  // Inches to the right from robot center
            camYOffset = 2.073;                         // Inches forward from robot center
            camZOffset = 10.758;                        // Inches up from the floor
            camYaw = -3.438;                            // degrees clockwise from robot front
            camPitch = 0.0;                             // degrees down from horizontal
            camRoll = 0.0;
            camPose = new TrcPose3D(camXOffset, camYOffset, camZOffset, camYaw, camPitch, camRoll);
        }   //LimelightParams
    }   //class LimelightParams

    public enum SampleType
    {
        Specimen,
        RedSample,
        BlueSample,
        YellowSample,
        RedAllianceSamples,
        BlueAllianceSamples,
        AnySample
    }   //enum SampleType

    // Warning: EOCV converts camera stream to RGBA whereas Desktop OpenCV converts it to BGRA. Therefore, the correct
    // color conversion must be RGBA (or RGB) to whatever color space you want to convert.
    //
    // YCrCb Color Space.
    private static final int colorConversion = Imgproc.COLOR_RGB2YCrCb;
    private static final double[] redSampleColorThresholds = {10.0, 180.0, 170.0, 240.0, 80.0, 120.0};
    private static final double[] blueSampleColorThresholds = {0.0, 180.0, 80.0, 150.0, 150.0, 200.0};
    private static final double[] yellowSampleColorThresholds = {100.0, 250.0, 120.0, 200.0, 30.0, 80.0};
    private static final TrcOpenCvColorBlobPipeline.FilterContourParams sampleFilterContourParams =
            new TrcOpenCvColorBlobPipeline.FilterContourParams()
                    .setMinArea(500.0)
                    .setMinPerimeter(100.0)
                    .setWidthRange(10.0, 1000.0)
                    .setHeightRange(10.0, 1000.0)
                    .setSolidityRange(0.0, 100.0)
                    .setVerticesRange(0.0, 1000.0)
                    .setAspectRatioRange(0.5, 2.5);
    private static final TrcOpenCvColorBlobPipeline.FilterContourParams tuneFilterContourParams =
            new TrcOpenCvColorBlobPipeline.FilterContourParams()
                    .setMinArea(10.0)
                    .setMinPerimeter(12.0)
                    .setWidthRange(0.0, 1000.0)
                    .setHeightRange(0.0, 1000.0)
                    .setSolidityRange(0.0, 100.0)
                    .setVerticesRange(0.0, 1000.0)
                    .setAspectRatioRange(0.5, 2.5);

    private final TrcDbgTrace tracer;
    private final Robot robot;
    private FtcRawEocvColorBlobPipeline rawColorBlobPipeline;
    public FtcRawEocvVision rawColorBlobVision;
    public FtcLimelightVision limelightVision;
    private FtcCameraStreamProcessor cameraStreamProcessor;
    public FtcVisionAprilTag aprilTagVision;
    private AprilTagProcessor aprilTagProcessor;
    public FtcVisionEocvColorBlob redSampleVision;
    private FtcEocvColorBlobProcessor redSampleProcessor;
    public FtcVisionEocvColorBlob blueSampleVision;
    private FtcEocvColorBlobProcessor blueSampleProcessor;
    public FtcVisionEocvColorBlob yellowSampleVision;
    private FtcEocvColorBlobProcessor yellowSampleProcessor;
    public FtcVision vision;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     */
    public Vision(Robot robot)
    {
        FtcOpMode opMode = FtcOpMode.getInstance();
        WebcamName webcam1;

        if (robot.robotInfo.webCam1 == null &&
                (RobotParams.Preferences.useWebCam || RobotParams.Preferences.tuneColorBlobVision))
        {
            throw new IllegalArgumentException("Must provide valid WebCam 1 info.");
        }

        this.tracer = new TrcDbgTrace();
        this.robot = robot;
        webcam1 = robot.robotInfo.webCam1 != null?
                opMode.hardwareMap.get(WebcamName.class, robot.robotInfo.webCam1.camName): null;
        if (RobotParams.Preferences.tuneColorBlobVision && webcam1 != null)
        {
            OpenCvCamera openCvCamera;

            if (RobotParams.Preferences.showVisionView)
            {
                int cameraViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                        "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
                openCvCamera = OpenCvCameraFactory.getInstance().createWebcam(webcam1, cameraViewId);
            }
            else
            {
                openCvCamera = OpenCvCameraFactory.getInstance().createWebcam(webcam1);
            }

//            if (RobotParams.Preferences.useCameraStreamProcessor)
//            {
//                FtcDashboard.getInstance().startCameraStream(openCvCamera, 0);
//            }

            tracer.traceInfo(moduleName, "Starting RawEocvColorBlobVision...");
            rawColorBlobPipeline = new FtcRawEocvColorBlobPipeline(
                    "rawColorBlobPipeline", colorConversion, redSampleColorThresholds, tuneFilterContourParams, true);
            // By default, display original Mat.
            rawColorBlobPipeline.setVideoOutput(0);
            rawColorBlobPipeline.setAnnotateEnabled(true);
            rawColorBlobVision = new FtcRawEocvVision(
                    "rawColorBlobVision", robot.robotInfo.webCam1.camImageWidth, robot.robotInfo.webCam1.camImageHeight,
                    null, null,
                    openCvCamera, robot.robotInfo.webCam1.camOrientation);
            rawColorBlobVision.setFpsMeterEnabled(RobotParams.Preferences.showVisionStat);
            setRawColorBlobVisionEnabled(false);
        }
        else
        {
            if (RobotParams.Preferences.useLimelightVision && robot.robotInfo.limelight != null)
            {
                limelightVision = new FtcLimelightVision(
                        robot.robotInfo.limelight.camName, robot.robotInfo.limelight.camPose, this::getTargetGroundOffset);
                limelightVision.setPipeline(0);
            }
            // Creating Vision Processors for VisionPortal.
            ArrayList<VisionProcessor> visionProcessorsList = new ArrayList<>();

            if (RobotParams.Preferences.useCameraStreamProcessor)
            {
                cameraStreamProcessor = new FtcCameraStreamProcessor();
                visionProcessorsList.add(cameraStreamProcessor);
//                FtcDashboard.getInstance().startCameraStream(cameraStreamProcessor, 0);
            }

            if (RobotParams.Preferences.useAprilTagVision)
            {
                tracer.traceInfo(moduleName, "Starting Webcam AprilTagVision...");
                FtcVisionAprilTag.Parameters aprilTagParams = new FtcVisionAprilTag.Parameters()
                        .setDrawTagIdEnabled(true)
                        .setDrawTagOutlineEnabled(true)
                        .setDrawAxesEnabled(false)
                        .setDrawCubeProjectionEnabled(false)
                        .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES);
                aprilTagVision = new FtcVisionAprilTag(aprilTagParams, AprilTagProcessor.TagFamily.TAG_36h11);
                aprilTagProcessor = aprilTagVision.getVisionProcessor();
                visionProcessorsList.add(aprilTagProcessor);
            }

            if (RobotParams.Preferences.useColorBlobVision && robot.robotInfo.webCam1 != null)
            {
                tracer.traceInfo(moduleName, "Starting Webcam SampleVision...");

                redSampleVision = new FtcVisionEocvColorBlob(
                        LEDIndicator.RED_SAMPLE, colorConversion, redSampleColorThresholds, sampleFilterContourParams,
                        true, robot.robotInfo.webCam1.cameraRect, robot.robotInfo.webCam1.worldRect, true);
                redSampleProcessor = redSampleVision.getVisionProcessor();
                visionProcessorsList.add(redSampleProcessor);

                blueSampleVision = new FtcVisionEocvColorBlob(
                        LEDIndicator.BLUE_SAMPLE, colorConversion, blueSampleColorThresholds, sampleFilterContourParams,
                        true, robot.robotInfo.webCam1.cameraRect, robot.robotInfo.webCam1.worldRect, true);
                blueSampleProcessor = blueSampleVision.getVisionProcessor();
                visionProcessorsList.add(blueSampleProcessor);

                yellowSampleVision = new FtcVisionEocvColorBlob(
                        LEDIndicator.YELLOW_SAMPLE, colorConversion, yellowSampleColorThresholds, sampleFilterContourParams,
                        true, robot.robotInfo.webCam1.cameraRect, robot.robotInfo.webCam1.worldRect, true);
                yellowSampleProcessor = yellowSampleVision.getVisionProcessor();
                visionProcessorsList.add(yellowSampleProcessor);
            }

            if (!visionProcessorsList.isEmpty())
            {
                VisionProcessor[] visionProcessors = new VisionProcessor[visionProcessorsList.size()];
                visionProcessorsList.toArray(visionProcessors);
                if (RobotParams.Preferences.useWebCam)
                {
                    // Use USB webcams.
                    vision = new FtcVision(
                            webcam1, robot.robotInfo.webCam1.camImageWidth, robot.robotInfo.webCam1.camImageHeight,
                            RobotParams.Preferences.showVisionView, RobotParams.Preferences.showVisionStat,
                            visionProcessors);
                }
                else
                {
                    // Use phone camera.
                    vision = new FtcVision(
                            RobotParams.Preferences.useBuiltinCamBack?
                                    BuiltinCameraDirection.BACK: BuiltinCameraDirection.FRONT,
                            robot.robotInfo.webCam1.camImageWidth, robot.robotInfo.webCam1.camImageHeight,
                            RobotParams.Preferences.showVisionView, RobotParams.Preferences.showVisionStat,
                            visionProcessors);
                }
                // Disable all vision until they are needed.
                for (VisionProcessor processor: visionProcessors)
                {
                    vision.setProcessorEnabled(processor, false);
                }
            }
        }
    }   //Vision

    /**
     * This method closes the vision portal and is normally called at the end of an opmode.
     */
    public void close()
    {
        if (vision != null)
        {
            vision.close();
        }
    }   //close

    /**
     * This method enables/disables FPS meter on the viewport.
     *
     * @param enabled specifies true to enable FPS meter, false to disable.
     */
    public void setFpsMeterEnabled(boolean enabled)
    {
        if (rawColorBlobVision != null)
        {
            rawColorBlobVision.setFpsMeterEnabled(enabled);
        }
        else if (vision != null)
        {
            vision.setFpsMeterEnabled(enabled);
        }
    }   //setFpsMeterEnabled

    /**
     * This method displays the exposure settings on the dashboard. This helps tuning camera exposure.
     *
     * @param lineNum specifies the dashboard line number to display the info.
     */
    public void displayExposureSettings(int lineNum)
    {
        long[] exposureSetting = vision.getExposureSetting();
        long currExposure = vision.getCurrentExposure();
        int[] gainSetting = vision.getGainSetting();
        int currGain = vision.getCurrentGain();

        if (exposureSetting != null && gainSetting != null)
        {
            robot.dashboard.displayPrintf(
                    lineNum, "Exp: %d (%d:%d), Gain: %d (%d:%d)",
                    currExposure, exposureSetting[0], exposureSetting[1], currGain, gainSetting[0], gainSetting[1]);
        }
    }   //displayExposureSettings

    /**
     * This method returns the color threshold values of rawColorBlobVision.
     *
     * @return array of color threshold values.
     */
    public double[] getRawColorBlobThresholds()
    {
        return rawColorBlobPipeline != null? rawColorBlobPipeline.getColorThresholds(): null;
    }   //getRawColorBlobThresholds

    /**
     * This method sets the color threshold values of rawColorBlobVision.
     *
     * @param colorThresholds specifies an array of color threshold values.
     */
    public void setRawColorBlobThresholds(double... colorThresholds)
    {
        if (rawColorBlobPipeline != null)
        {
            rawColorBlobPipeline.setColorThresholds(colorThresholds);
        }
    }   //setRawColorBlobThresholds

    /**
     * This method enables/disables raw ColorBlob vision.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public void setRawColorBlobVisionEnabled(boolean enabled)
    {
        if (rawColorBlobVision != null)
        {
            rawColorBlobVision.setPipeline(enabled? rawColorBlobPipeline: null);
        }
    }   //setRawColorBlobVisionEnabled

    /**
     * This method checks if raw ColorBlob vision is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    public boolean isRawColorBlobVisionEnabled()
    {
        return rawColorBlobVision != null && rawColorBlobVision.getPipeline() != null;
    }   //isRawColorBlobVisionEnabled

    /**
     * This method calls RawColorBlob vision to detect the color blob for color threshold tuning.
     *
     * @param lineNum specifies the dashboard line number to display the detected object info, -1 to disable printing.
     * @return detected raw color blob object info.
     */
    public TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> getDetectedRawColorBlob(int lineNum)
    {
        TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> colorBlobInfo =
                rawColorBlobVision != null? rawColorBlobVision.getBestDetectedTargetInfo(null, null, 0.0, 0.0): null;

        if (cameraStreamProcessor != null && colorBlobInfo != null)
        {
            cameraStreamProcessor.addRectInfo(
                    colorBlobInfo.detectedObj.label, colorBlobInfo.detectedObj.getRotatedRectVertices());
        }

        if (lineNum != -1)
        {
            robot.dashboard.displayPrintf(
                    lineNum, "RawColorBlob: %s, heading=%.3f",
                    colorBlobInfo != null? colorBlobInfo: "Not found.",
                    robot.robotDrive != null? robot.robotDrive.driveBase.getHeading(): 0.0);
        }

        return colorBlobInfo;
    }   //getDetectedRawColorBlob

    /**
     * This method enables/disables Limelight vision for the specified pipeline.
     *
     * @param pipelineIndex specifies the limelight pipeline index to be selected, ignore if disabled.
     * @param enabled specifies true to enable, false to disable.
     */
    public void setLimelightVisionEnabled(int pipelineIndex, boolean enabled)
    {
        if (limelightVision != null)
        {
            if (enabled)
            {
                limelightVision.setPipeline(pipelineIndex);
            }
            limelightVision.setVisionEnabled(enabled);
            tracer.traceInfo(moduleName, "Pipeline %d is %s: running=%s",
                    pipelineIndex, enabled? "enabled": "disabled", limelightVision.limelight.isRunning());
        }
    }   //setLimelightVisionEnabled

    /**
     * This method checks if Limelight vision is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    public boolean isLimelightVisionEnabled()
    {
        return limelightVision != null && limelightVision.isVisionEnabled();
    }   //isLimelightVisionEnabled

    /**
     * This method calls Limelight vision to detect the object.
     *
     * @param resultType specifies the result type to look for.
     * @param label specifies the detected object label, can be null to match any label.
     * @param lineNum specifies the dashboard line number to display the detected object info, -1 to disable printing.
     * @return detected Limelight object info.
     */
    public TrcVisionTargetInfo<FtcLimelightVision.DetectedObject> getLimelightDetectedObject(
            FtcLimelightVision.ResultType resultType, String label, int lineNum)
    {
        TrcVisionTargetInfo<FtcLimelightVision.DetectedObject> limelightInfo = null;

        if (limelightVision != null)
        {
            String objectName = null;
            int pipelineIndex = -1;
            Double robotHeading = robot.robotDrive != null? robot.robotDrive.driveBase.getHeading(): null;

            limelightInfo = null;
            if (limelightInfo != null)
            {
                pipelineIndex = limelightVision.getPipeline();
                switch (pipelineIndex)
                {
                    case 0:
                        objectName = LEDIndicator.APRIL_TAG;
                        break;

                    case 1:
                        objectName = LEDIndicator.RED_SAMPLE;
                        break;

                    case 2:
                        objectName = LEDIndicator.BLUE_SAMPLE;
                        break;

                    case 3:
                        objectName = LEDIndicator.YELLOW_SAMPLE;
                        break;

                    default:
                        break;
                }
            }

            if (objectName != null && robot.ledIndicator != null)
            {
                robot.ledIndicator.setDetectedPattern(objectName);
            }

            if (lineNum != -1)
            {
                robot.dashboard.displayPrintf(
                        lineNum, "%s(%d): %s",
                        objectName, pipelineIndex, limelightInfo != null? limelightInfo: "Not found.");
            }
        }

        return limelightInfo;
    }   //getLimelightDetectedObject

    /**
     * This method enables/disables the Vision Processor.
     *
     * @param processor specifies the vision processor to enable/disable.
     * @param enabled specifies true to enable, false to disable.
     */
    public void setVisionProcessorEnabled(VisionProcessor processor, boolean enabled)
    {
        if (processor != null)
        {
            vision.setProcessorEnabled(processor, enabled);
        }
    }   //setVisionProcessorEnabled

    /**
     * This method checks if the Vision Processor is enabled.
     *
     * @param processor specifies the vision processor to enable/disable.
     * @return true if enabled, false if disabled.
     */
    public boolean isVisionProcessorEnabled(VisionProcessor processor)
    {
        return processor != null && vision.isVisionProcessorEnabled(processor);
    }   //isVisionProcessorEnabled

    /**
     * This method enables/disables the CameraStream processor.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public void setCameraStreamEnabled(boolean enabled)
    {
        if (vision != null && cameraStreamProcessor != null)
        {
            cameraStreamProcessor.setCameraStreamEnabled(vision, enabled);
        }
    }   //setCameraStreamEnabled

    /**
     * This method checks if the CameraStream processor is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    public boolean isCameraStreamEnabled()
    {
        return cameraStreamProcessor != null && cameraStreamProcessor.isCameraStreamEnabled();
    }   //isAprilTagVisionEnabled

    /**
     * This method enables/disables AprilTag vision.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public void setAprilTagVisionEnabled(boolean enabled)
    {
        setVisionProcessorEnabled(aprilTagProcessor, enabled);
    }   //setAprilTagVisionEnabled

    /**
     * This method checks if AprilTag vision is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    public boolean isAprilTagVisionEnabled()
    {
        return isVisionProcessorEnabled(aprilTagProcessor);
    }   //isAprilTagVisionEnabled

    /**
     * This method calls AprilTag vision to detect the AprilTag object.
     *
     * @param id specifies the AprilTag ID to look for, null if match to any ID.
     * @param lineNum specifies the dashboard line number to display the detected object info, -1 to disable printing.
     * @return detected AprilTag object info.
     */
    public TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> getDetectedAprilTag(Integer id, int lineNum)
    {
        TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> aprilTagInfo =
                aprilTagVision.getBestDetectedTargetInfo(id, null);

        if (cameraStreamProcessor != null && aprilTagInfo != null)
        {
            cameraStreamProcessor.addRectInfo(
                    Integer.toString(aprilTagInfo.detectedObj.aprilTagDetection.id),
                    aprilTagInfo.detectedObj.getRotatedRectVertices());
        }

        if (aprilTagInfo != null && robot.ledIndicator != null)
        {
            robot.ledIndicator.setDetectedPattern(LEDIndicator.APRIL_TAG);
        }

        if (lineNum != -1)
        {
            robot.dashboard.displayPrintf(
                    lineNum, "%s: %s", LEDIndicator.APRIL_TAG, aprilTagInfo != null? aprilTagInfo : "Not found.");
        }

        return aprilTagInfo;
    }   //getDetectedAprilTag

    /**
     * This method calculates the robot's absolute field location with the detected AprilTagInfo.
     *
     * @param aprilTagInfo specifies the detected AprilTag info.
     * @return robot field location.
     */
    public TrcPose2D getRobotFieldPose(TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> aprilTagInfo)
    {
        TrcPose2D robotPose = null;

        if (aprilTagInfo != null)
        {
            TrcPose2D aprilTagPose =
                    RobotParams.Game.APRILTAG_POSES[aprilTagInfo.detectedObj.aprilTagDetection.id - 1];
            TrcPose2D cameraPose = aprilTagPose.subtractRelativePose(aprilTagInfo.objPose);
            robotPose = cameraPose.subtractRelativePose(
                    new TrcPose2D(robot.robotInfo.webCam1.camXOffset, robot.robotInfo.webCam1.camYOffset,
                            robot.robotInfo.webCam1.camYaw));
            tracer.traceInfo(
                    moduleName,
                    "AprilTagId=" + aprilTagInfo.detectedObj.aprilTagDetection.id +
                            ", aprilTagFieldPose=" + aprilTagPose +
                            ", aprilTagPoseFromCamera=" + aprilTagInfo.objPose +
                            ", cameraPose=" + cameraPose +
                            ", robotPose=%s" + robotPose);
        }

        return robotPose;
    }   //getRobotFieldPose

    /**
     * This method uses vision to find an AprilTag and uses the AprilTag's absolute field location and its relative
     * position from the camera to calculate the robot's absolute field location.
     *
     * @return robot field location.
     */
    public TrcPose2D getRobotFieldPose()
    {
        TrcPose2D robotPose = null;

        if (isLimelightVisionEnabled())
        {
            TrcVisionTargetInfo<FtcLimelightVision.DetectedObject> aprilTagInfo =
                    getLimelightDetectedObject(FtcLimelightVision.ResultType.Fiducial, null, -1);

            if (aprilTagInfo != null)
            {
                robotPose = aprilTagInfo.detectedObj.robotPose;
            }
        }
        else if (isAprilTagVisionEnabled())
        {
            // Find any AprilTag in view.
            TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> aprilTagInfo = getDetectedAprilTag(null, -1);

            if (aprilTagInfo != null)
            {
                robotPose = getRobotFieldPose(aprilTagInfo);
            }
        }

        return robotPose;
    }   //getRobotFieldPose

    /**
     * This method enables/disables vision for the specified sample type.
     *
     * @param sampleType specifies the sample type to be detected.
     * @param enabled specifies true to enable, false to disable.
     */
    public void setSampleVisionEnabled(SampleType sampleType, boolean enabled)
    {
        switch (sampleType)
        {
            case RedSample:
                setVisionProcessorEnabled(redSampleProcessor, enabled);
                break;

            case BlueSample:
                setVisionProcessorEnabled(blueSampleProcessor, enabled);
                break;

            case YellowSample:
                setVisionProcessorEnabled(yellowSampleProcessor, enabled);
                break;

            case RedAllianceSamples:
            case BlueAllianceSamples:
            case AnySample:
                if (sampleType != SampleType.BlueAllianceSamples)
                {
                    setVisionProcessorEnabled(redSampleProcessor, enabled);
                }
                if (sampleType != SampleType.RedAllianceSamples)
                {
                    setVisionProcessorEnabled(blueSampleProcessor, enabled);
                }
                setVisionProcessorEnabled(yellowSampleProcessor, enabled);
                break;
        }
    }   //setSampleVisionEnabled

    /**
     * This method checks if vision is enabled for the specified sample type.
     *
     * @param sampleType specifies the sample type to be detected.
     * @return true if enabled, false if disabled.
     */
    public boolean isSampleVisionEnabled(SampleType sampleType)
    {
        boolean enabled = false;

        switch (sampleType)
        {
            case RedSample:
                enabled = isVisionProcessorEnabled(redSampleProcessor);
                break;

            case BlueSample:
                enabled = isVisionProcessorEnabled(blueSampleProcessor);
                break;

            case YellowSample:
                enabled = isVisionProcessorEnabled(yellowSampleProcessor);
                break;

            case RedAllianceSamples:
            case BlueAllianceSamples:
            case AnySample:
                enabled =
                        sampleType != SampleType.BlueAllianceSamples && isVisionProcessorEnabled(redSampleProcessor) ||
                                sampleType != SampleType.RedAllianceSamples && isVisionProcessorEnabled(blueSampleProcessor) ||
                                isVisionProcessorEnabled(yellowSampleProcessor);
                break;
        }

        return enabled;
    }   //isSampleVisionEnabled

    /**
     * This method calls ColorBlob vision to detect the specified Sample object.
     *
     * @param sampleType specifies the sample type to be detected.
     * @param groundOffset specifies the ground offset of the detected sample.
     * @param lineNum specifies the dashboard line number to display the detected object info, -1 to disable printing.
     * @return detected Sample object info.
     */
    public TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> getDetectedSample(
            SampleType sampleType, double groundOffset, int lineNum)
    {
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> sampleInfo = null;

        switch (sampleType)
        {
            case RedSample:
                sampleInfo = redSampleVision != null? redSampleVision.getBestDetectedTargetInfo(
                        null, this::compareDistance, groundOffset, robot.robotInfo.webCam1.camZOffset): null;
                break;

            case BlueSample:
                sampleInfo = blueSampleVision != null? blueSampleVision.getBestDetectedTargetInfo(
                        null, this::compareDistance, groundOffset, robot.robotInfo.webCam1.camZOffset): null;
                break;

            case YellowSample:
                sampleInfo = yellowSampleVision != null? yellowSampleVision.getBestDetectedTargetInfo(
                        null, this::compareDistance, groundOffset, robot.robotInfo.webCam1.camZOffset): null;
                break;

            case RedAllianceSamples:
            case BlueAllianceSamples:
            case AnySample:
                ArrayList<TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject>> sampleList =
                        new ArrayList<>();

                if (sampleType != SampleType.BlueAllianceSamples)
                {
                    sampleInfo = redSampleVision != null ? redSampleVision.getBestDetectedTargetInfo(
                            null, this::compareDistance, groundOffset, robot.robotInfo.webCam1.camZOffset) : null;
                    if (sampleInfo != null)
                    {
                        sampleList.add(sampleInfo);
                    }
                }

                if (sampleType != SampleType.RedAllianceSamples)
                {
                    sampleInfo = blueSampleVision != null ? blueSampleVision.getBestDetectedTargetInfo(
                            null, this::compareDistance, groundOffset, robot.robotInfo.webCam1.camZOffset) : null;
                    if (sampleInfo != null)
                    {
                        sampleList.add(sampleInfo);
                    }
                }

                sampleInfo = yellowSampleVision != null? yellowSampleVision.getBestDetectedTargetInfo(
                        null, this::compareDistance, groundOffset, robot.robotInfo.webCam1.camZOffset): null;
                if (sampleInfo != null)
                {
                    sampleList.add(sampleInfo);
                }

                if (!sampleList.isEmpty())
                {
                    if (sampleList.size() > 1)
                    {
                        sampleList.sort(this::compareDistance);
                    }
                    sampleInfo = sampleList.get(0);
                }
                break;
        }

        if (cameraStreamProcessor != null && sampleInfo != null)
        {
            cameraStreamProcessor.addRectInfo(
                    sampleInfo.detectedObj.label, sampleInfo.detectedObj.getRotatedRectVertices());
        }

        if (lineNum != -1)
        {
            if (sampleInfo != null)
            {
                robot.dashboard.displayPrintf(
                        lineNum, "%s: %s (rotatedAngle=%.1f)",
                        sampleInfo.detectedObj.label, sampleInfo, sampleInfo.detectedObj.rotatedRect.angle);
            }
            else
            {
                robot.dashboard.displayPrintf(lineNum, "No sample found.");
            }
        }

        return sampleInfo;
    }   //getDetectedSample

    /**
     * This method returns the target Z offset from ground.
     *
     * @param resultType specifies the detected object result type.
     * @return target ground offset.
     */
    private double getTargetGroundOffset(FtcLimelightVision.ResultType resultType)
    {
        double offset = 0.0;

        if (resultType == FtcLimelightVision.ResultType.Fiducial)
        {
            offset = 5.75;
        }
        /*else if (resultType == FtcLimelightVision.ResultType.Python)
        {
            offset = 10.0;
        }*/

        return offset;
    }   //getTargetGroundOffset

    /**
     * This method is called by the Arrays.sort to sort the target object by increasing distance.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has closer distance than b, 0 if a and b have equal distances, positive value
     *         if a has higher distance than b.
     */
    private int compareDistance(
            TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> a,
            TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> b)
    {
        return (int)((a.objPose.y - b.objPose.y)*100);
    }   //compareDistance

}   //class Vision
