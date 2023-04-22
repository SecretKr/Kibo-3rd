package jp.jaxa.iss.kibo.rpc.sampleapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Mat;
import android.util.Log;
//import org.scijava.nativelib.NativeLoader;

//import clojure.lang.Var;

import org.opencv.aruco.DetectorParameters;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
//import org.opencv.highgui.HighGui;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Size;
//import org.opencv.core.FeatureDetector;
//import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.FastFeatureDetector;
import org.opencv.features2d.FastFeatureDetector;
/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    // ------------------ Setting up variable --------------------
    final int LOOP_MAX = 5;
    double[] cameraMatrixArray = {
            523.105750, 0.000000, 635.434258,
            0.000000, 534.765913, 500.335102,
            0.000000, 0.000000, 1.000000
    };
    double[] distCoeffsArray = { -0.164787, 0.020375, -0.001572, -0.000369, 0.000000 };

    Point[] positionTarget1 = new Point[]{ new Point(10.71f, -7.75f, 4.48f)};
    Quaternion quaternionTarget1 = new Quaternion(-0.02735f, 0.69021f, 0.02864f, 0.72252f);

    Point[] positionTarget2 = new Point[] {
            new Point(11.39f, -9.39f, 4.48f),
            new Point(11.2746f, -10.04f, 5.41f)
    };
    Point[] positionOptional = new Point[] { new Point(11.2746f, -9.6f, 5.41f)};
    Point[] positionEndPoint = new Point[] {
            new Point(10.71f, -9.7f, 5.2f),
            new Point(10.71f, -7.75f, 5.2f),
            new Point(11.2746f, -7.89178f, 4.96538f)
    };

    Quaternion quaternionTarget2 = new Quaternion(0f, 0f, -0.707f, 0.707f);
    Quaternion quaternionEndPoint = new Quaternion(0f, 0f, -0.707f, 0.707f);

    @Override
    protected void runPlan1(){

        // Mission start
        api.startMission();

        // ------------------ Target 1 --------------------
        moveToPos(positionTarget1, quaternionTarget1);
        delay(5);
        api.reportPoint1Arrival();
        Mat img = api.getMatNavCam();
        api.saveMatImage(img, "Target1.png");

        api.laserControl(true);
        img = api.getMatNavCam();
        api.saveMatImage(img, "Target1Snap.png");
        api.takeTarget1Snapshot();

        // ------------------ Target 2 ------------------
        moveToPos(positionTarget2, quaternionTarget2);
        delay(6);
        img = api.getMatNavCam();
        api.saveMatImage(img, "Target2.png");

        aim(img, positionTarget2[positionTarget2.length - 1]);
        delay(5);

        api.laserControl(true);
        img = api.getMatNavCam();
        api.saveMatImage(img, "Target2Snap.png");
        api.takeTarget2Snapshot();

        // ------------------ End Point ------------------
        moveToPos(positionEndPoint, quaternionEndPoint);

        // send mission completion
        api.reportMissionCompletion();
    }

    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }

    // You can add your method
    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w){

        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                (float)qua_z, (float)qua_w);

        api.moveTo(point, quaternion, true);
    }

    private void relativeMoveToWrapper(double pos_x, double pos_y, double pos_z,
                                       double qua_x, double qua_y, double qua_z,
                                       double qua_w) {

        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);

        api.relativeMoveTo(point, quaternion, true);
    }


    private void aim(Mat img, Point position){
        System.out.println("Start aim");
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);
        int row = 0, col = 0;
        cameraMatrix.put(row, col, cameraMatrixArray);
        distCoeffs.put(row, col, distCoeffsArray);
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> arucoFound = new ArrayList<>();
        Mat markerIds = new Mat();
        Mat rotationMatrix = new Mat(), translationVectors = new Mat();
        DetectorParameters parameters = DetectorParameters.create();
        Aruco.detectMarkers(img, dictionary, arucoFound, markerIds, parameters);
        Aruco.estimatePoseSingleMarkers(arucoFound, 0.05f, cameraMatrix, distCoeffs, rotationMatrix, translationVectors);

        int size2 = (int) (translationVectors.total() * translationVectors.channels());
        double[] disCam2TargetArray = new double[size2];
        translationVectors.get(0, 0, disCam2TargetArray);

        double avg = 0f;
        double ppm;

        if (arucoFound.size() > 0) {
            for (Mat corner : arucoFound) {
                corner.convertTo(corner, CvType.CV_64FC3);
                int size = (int) (corner.total() * corner.channels());
                double[] temp = new double[size]; // use double[] instead of byte[]
                corner.get(0, 0, temp);
                ppm = Math.sqrt(Math.pow(temp[2] - temp[0], 2) + Math.pow(temp[3] - temp[1], 2)) / 0.05f;
                avg += ppm;
            }

            ppm = avg / arucoFound.size();

            double[] fr = pyrToQ(0f, 0f, -80f);

            double avgDisCam2TargetArray = 0f;
            for(int i = 2;i < disCam2TargetArray.length; i+=3){
                avgDisCam2TargetArray += disCam2TargetArray[i];
            }
            avgDisCam2TargetArray = avgDisCam2TargetArray / (disCam2TargetArray.length/3);
            //double[] cr = calRotation(img, ppm, disCam2TargetArray[disCam2TargetArray.length - 1]);
            double[] cr = calRotation(img, ppm, avgDisCam2TargetArray);

            Quaternion quaterniontF = new Quaternion((float) fr[0], (float) fr[1], (float) fr[2], (float) fr[3]);
            Quaternion quaterniontC = new Quaternion((float) cr[0], (float) cr[1], (float) cr[2], (float) cr[3]);

            System.out.print("Q1 ");
            System.out.print(cr[0]);
            System.out.print(" ");
            System.out.print(cr[1]);
            System.out.print(" ");
            System.out.print(cr[2]);
            System.out.print(" ");
            System.out.println(cr[3]);

            api.moveTo(position, quaterniontF, true);
            api.moveTo(position, quaterniontF, true);

            api.moveTo(position, quaterniontC, true);
            api.moveTo(position, quaterniontC, true);
        }
        else {
            moveToPos(positionOptional, quaternionTarget2);
            Mat imgO = api.getMatNavCam();
            api.saveMatImage(img, "Target2Optional.png");
            aim(imgO, positionOptional[positionOptional.length - 1]);
        }
    }

    static double[] calRotation(Mat img, double ppm, double A) {
        double cameraCenterx = img.width()/2; // camera width/2 (pixel)
        double cameraCentery = img.height()/2; // camera height/2 (pixel)
        double[] t2c = getTarget2Center(img);
        double targetPosx = t2c[0];
        double targetPosy = img.height()-t2c[1];

        double offsetX = -0.005f;
        double offsetY = 0.005f;

        double L = ( targetPosx - cameraCenterx ) / ppm + offsetX;
        double M = ( targetPosy - cameraCentery ) / ppm + offsetY;

        double lowestDiff = 1e9;
        double diff;
        double yaw = 0f;
        double pitch = 0f;
        double roll = 0f;

        for(double y = -45.00f;y <= 45.00f;y += 0.01f){
            diff = Math.abs(Math.tan(Math.toRadians(y)) + ((0.0994f-L)-(0.0572f-Math.cos(Math.toRadians(y))*0.0572f))/((A+0.1177f)+(Math.sin(Math.toRadians(y))*0.0572f)));
            if(diff < lowestDiff){
                lowestDiff = diff;
                yaw = y;
            }
        }

        lowestDiff = 1e9;
        for(double p = -45.00f;p <= 45.00f;p += 0.01f){
            diff = Math.abs(Math.tan(Math.toRadians(p)) + ((0.0285f-M)-(0.1111f-Math.cos(Math.toRadians(p))*0.1111f))/((A+0.1177f)+(Math.sin(Math.toRadians(p))*0.1111f)));
            if(diff < lowestDiff){
                lowestDiff = diff;
                pitch = p;
            }
        }

        //// debug
        System.out.print("ppm ");
        System.out.println(ppm);
        System.out.print("Cam x ");
        System.out.println(cameraCenterx);
        System.out.print("Cam y ");
        System.out.println(cameraCentery);
        System.out.print("Target x ");
        System.out.println(targetPosx);
        System.out.print("Target y ");
        System.out.println(targetPosy);
        System.out.print("A ");
        System.out.println(A);
        System.out.print("L ");
        System.out.println(L);
        System.out.print("M ");
        System.out.println(M);
        System.out.print("Yaw offset ");
        System.out.println(yaw);
        System.out.print("Pitch offset ");
        System.out.println(pitch);

        yaw = 270+yaw;

        System.out.print("Yaw ");
        System.out.println(yaw);
        System.out.print("Pitch ");
        System.out.println(pitch);
        System.out.print("Roll ");
        System.out.println(roll);

        return pyrToQ(pitch, roll, yaw);
    }


    static double[] getTarget2Center(Mat img) {
        Imgproc.blur(img, img, new Size(5, 5));
        Mat edges = new Mat();
        Imgproc.Canny(img, edges, 100, 200);

        Mat circles = new Mat();

        Imgproc.HoughCircles(edges, circles, Imgproc.CV_HOUGH_GRADIENT, 1, 100, 100, 30, 10, 40 );
        if(circles.empty()) return new double[] {img.width()/2, img.height()/2};
        double[] temp = circles.get(0, 0);

        return temp;
    }

    private void delay(int seconds){
        try {
            Thread.sleep(seconds*1000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        };
    }

    private void moveToPos(Point[] position, Quaternion quaternion){
        int loopCounter = 0;
        Result result;
        for (Point p : position) {
            result = api.moveTo(p, quaternion, true);
            loopCounter = 0;
            while(!result.hasSucceeded() && loopCounter < LOOP_MAX){
                result = api.moveTo(p, quaternion, true);
                ++loopCounter;
            }
        }
    }

    static double[] pyrToQ(double pitch, double roll, double yaw){
        double cy = Math.cos(Math.toRadians(yaw * 0.5));
        double sy = Math.sin(Math.toRadians(yaw * 0.5));
        double cp = Math.cos(Math.toRadians(pitch * 0.5));
        double sp = Math.sin(Math.toRadians(pitch * 0.5));
        double cr = Math.cos(Math.toRadians(roll * 0.5));
        double sr = Math.sin(Math.toRadians(roll * 0.5));

        double[] result = {sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy};
        return result;
    }
}