package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import static android.graphics.Color.red;
import static android.graphics.Color.green;
import static android.graphics.Color.blue;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.Parameters;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
import java.util.Collections;
import java.util.concurrent.BlockingQueue;
/*
public class RingDetect {
    private AutoMaster auto = null;

    public VuforiaLocalizer vuforia;

    private static final String VUFORIA_KEY =
            "AQswYz7/////AAABmcNWVtOSYEEXpQufuBKrdNMSKO5UAESlpwf1GyWOEzZMytfVdfY2BsxJop+3JkhqYQEby7j5SJHbcw6kSDuMe40rGeec5vJtb9m+qxy8jqy8EuBZ8n9IAldRtolwfIBkMI+d9+EkoqSBiZwhSWDzT0EVw83o3H+WzzMmj91dURhqRNzdHjEz0lUUgwDNrfNuW3oGPn1A1alADdHYnnAo++SiO9m4hHPVkdomVSxNjxu3I6whv16zWlQTLdK97POf2t37U+rS/2hZ5GSNG054PtWDppXH+ec8XNrDfys6+OmeG/m6MFvNjoUAyUgV7bsqMM+QUM3eTI3/FENR6PZ3VND47T3Dm74Hxkor++lEZHEi";


    private final int RED_THRESHOLD = 160;
    private final int GREEN_THRESHOLD = 115;
    private final int BLUE_THRESHOLD = 75;



    public static String wobblePosition = "notFound";


    public void Init(AutoMaster autoMaster) {
        auto = autoMaster;
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
     /*   VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        //     boolean useWebacm = true;

        //   if (useWebacm) {
        parameters.cameraName = auto.hardwareMap.get(WebcamName.class, "Webcam 1");
        //  } else {
        //   parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //     parameters.cameraDirection = CameraDirection.BACK;
        // }

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(4); //tells VuforiaLocalizer to only store one frame at a time
        vuforia.enableConvertFrameToBitmap();
        auto.idle();
     /*   VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters();
        params.vuforiaLicenseKey = "AQvLCbX/////AAABmTGnnsC2rUXvp1TAuiOSac0ZMvc3GKI93tFoRn4jPzB3uSMiwj75PNfUU6MaVsNZWczJYOep8LvDeM/3hf1+zO/3w31n1qJTtB2VHle8+MHWNVbNzXKLqfGSdvXK/wYAanXG2PBSKpgO1Fv5Yg27eZfIR7QOh7+J1zT1iKW/VmlsVSSaAzUSzYpfLufQDdE2wWQYrs8ObLq2kC37CeUlJ786gywyHts3Mv12fWCSdTH5oclkaEXsVC/8LxD1m+gpbRc2KC0BXnlwqwA2VqPSFU91vD8eCcD6t2WDbn0oJas31PcooBYWM6UgGm9I2plWazlIok72QG/kOYDh4yXOT4YXp1eYh864e8B7mhM3VclQ";
        params.cameraName = auto.hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(params);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time

      */
        // init servo
/*    }

    public void Update(boolean grab) {
        // Grab amount
    }
*/

/*
    public android.graphics.Bitmap getBitmap() throws InterruptedException {

        VuforiaLocalizer.CloseableFrame picture = vuforia.getFrameQueue().take();
        Image rgb = picture.getImage(1);

        long numImages = picture.getNumImages();

        auto.telemetry.addData("Num images", numImages);

        for (int i = 0; i < numImages; i++) {

            int format = picture.getImage(i).getFormat();
            auto.telemetry.addData("format:", format);
            if (format == PIXEL_FORMAT.RGB565) {
                rgb = picture.getImage(i);
                break;
            } else {
                auto.telemetry.addLine("Didn't find correct RGB format");


            }
        }

        android.graphics.Bitmap imageBitmap = android.graphics.Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), android.graphics.Bitmap.Config.RGB_565);
        imageBitmap.copyPixelsFromBuffer(rgb.getPixels());

        auto.telemetry.addData("Image width", imageBitmap.getWidth());
        auto.telemetry.addData("Image height", imageBitmap.getHeight());


        auto.sleep(500);

        picture.close();

        return imageBitmap;
    }


/*    //True for red
    public String Wobble(boolean red) throws InterruptedException {
        try {
            while (auto.opModeIsActive() && !auto.isStopRequested()) {
                double avgX = 0;
                double avgY = 0;
                double medX = 0;
                double medY = 0;
                android.graphics.Bitmap bitmap = getBitmap();
                int skystonePixelCount = 0;
                ArrayList<Integer> xValues = new ArrayList<>();
                ArrayList<Integer> yValues = new ArrayList<>();
                if (!red) {
                    for (int y = 0; y < bitmap.getHeight()/2; y++) {
                        for (int x = 320; x < bitmap.getWidth(); x++) {
                            int pixel = bitmap.getPixel(x, y);
                            if (red(pixel) <= RED_THRESHOLD && blue(pixel) <= BLUE_THRESHOLD && green(pixel) <= GREEN_THRESHOLD) {
                                xValues.add(x);
                                yValues.add(y);
                            }
                        }
                    }
                } else {
                    for (int y = 0; y < bitmap.getHeight()/2; y++) {
                        for (int x = 213; x < bitmap.getWidth(); x++) {
                            int pixel = bitmap.getPixel(x, y);
                            if (red(pixel) <= RED_THRESHOLD && blue(pixel) <= BLUE_THRESHOLD && green(pixel) <= GREEN_THRESHOLD) {
                                xValues.add(x);
                                yValues.add(y);
                            }
                        }
                    }
                }

                for (int xCoor : xValues) {
                    avgX += xCoor;
                }
                for (int yCoor : yValues) {
                    avgY += yCoor;
                }
                Collections.sort(xValues);
                Collections.sort(yValues);
                medX = xValues.get(xValues.size() / 2);
                auto.telemetry.addData("medY", medY);
                medY = yValues.get(yValues.size() / 2);
                avgX /= xValues.size();
                avgY /= yValues.size();
                int pixelcount = xValues.get(xValues.size());
                if (red) {
                    if (medX < bitmap.getWidth() * .33) {    //0,0 starts at left top
                        wobblePosition = "1 & 4";
                        auto.telemetry.addData("skystonePosition: ", wobblePosition);
                    } else if (medX < bitmap.getWidth() * 0.66 && medX > bitmap.getWidth() * 0.34) {
                        wobblePosition = "2 & 5";
                        auto.telemetry.addData("skystonePosition: ", wobblePosition);
                    } else {
                        wobblePosition = "3 & 6";
                        auto.telemetry.addData("skystonePosition: ", wobblePosition);
                    }

                } else {
                    if (pixelcount <= 50) {    //0,0 starts at left top
                        wobblePosition = "A";
                        auto.telemetry.addData("wobblePosition: ", wobblePosition);
                    } else if (medY < bitmap.getHeight() * .45 && medY > bitmap.getHeight() * 0.25) {
                        wobblePosition = "B";
                        auto.telemetry.addData("skystonePosition: ", wobblePosition);
                    } else {
                        wobblePosition = "C";
                        auto.telemetry.addData("skystonePosition: ", wobblePosition);
                    }
                }
                break;
            }
            return wobblePosition;
        } catch (IndexOutOfBoundsException e) {
            wobblePosition = "default";
            auto.telemetry.addData("wobblePosition:", wobblePosition);
            return wobblePosition;
        }
    }
*/
    public class RingDetect {

        LinearOpMode opMode;
        String wobblePos = "A";

        private VuforiaLocalizer vuforia;
        private Parameters parameters;
        private VuforiaLocalizer.CameraDirection CAMERA_CHOICE = CameraDirection.BACK;
        private static final String VUFORIA_KEY = "AQswYz7/////AAABmcNWVtOSYEEXpQufuBKrdNMSKO5UAESlpwf1GyWOEzZMytfVdfY2BsxJop+3JkhqYQEby7j5SJHbcw6kSDuMe40rGeec5vJtb9m+qxy8jqy8EuBZ8n9IAldRtolwfIBkMI+d9+EkoqSBiZwhSWDzT0EVw83o3H+WzzMmj91dURhqRNzdHjEz0lUUgwDNrfNuW3oGPn1A1alADdHYnnAo++SiO9m4hHPVkdomVSxNjxu3I6whv16zWlQTLdK97POf2t37U+rS/2hZ5GSNG054PtWDppXH+ec8XNrDfys6+OmeG/m6MFvNjoUAyUgV7bsqMM+QUM3eTI3/FENR6PZ3VND47T3Dm74Hxkor++lEZHEi";

      /*  private final int RED_THRESHOLD = 140;
        private final int GREEN_THRESHOLD = 89;
        private final int BLUE_THRESHOLD = 105;
*/
    private final int RED_THRESHOLD = 160;
    private final int GREEN_THRESHOLD = 105;
    private final int BLUE_THRESHOLD = 105;

        public RingDetect (LinearOpMode opMode){

            this.opMode = opMode;

            int cameraMonitorViewId = this.opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.opMode.hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters params = new Parameters(cameraMonitorViewId);

            params.vuforiaLicenseKey = VUFORIA_KEY;
            params.cameraDirection = CAMERA_CHOICE;
            params.cameraName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
            vuforia = ClassFactory.getInstance().createVuforia(params);

            Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
            vuforia.setFrameQueueCapacity(4);
            vuforia.enableConvertFrameToBitmap();

        }

        public Bitmap getBitmap() throws InterruptedException {

            VuforiaLocalizer.CloseableFrame picture;
            picture = vuforia.getFrameQueue().take();
            Image rgb = picture.getImage(1);

            long numImages = picture.getNumImages();

            opMode.telemetry.addData("Num images", numImages);
            opMode.telemetry.update();

            for (int i = 0; i < numImages; i++) {

                int format = picture.getImage(1).getFormat();
                opMode.telemetry.addData("format:", format);
                if (format == PIXEL_FORMAT.RGB565) {
                    rgb = picture.getImage(1);
                    break;
                } else {
                    opMode.telemetry.addLine("Didn't find correct RGB format");
                    opMode.telemetry.update();
                    opMode.sleep(5000);

                }
            }

            Bitmap imageBitmap = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
            imageBitmap.copyPixelsFromBuffer(rgb.getPixels());

            opMode.sleep(200);

            picture.close();


            return imageBitmap;
        }

        public double getImageHeight() throws InterruptedException {
            Bitmap bitmap = getBitmap();
            return bitmap.getHeight();
        }

        public double getImageWidth() throws InterruptedException {
            Bitmap bitmap = getBitmap();
            return bitmap.getWidth();
        }

        public String getWobble() throws InterruptedException {
            Bitmap bitmap = getBitmap();
            int yellowPixelCount = 0;
            ArrayList<Integer> xValues = new ArrayList<>();
            ArrayList<Integer> yValues = new ArrayList<>();
            //idth = 640
            //height = 480
            for (int y = 0; y < (bitmap.getHeight() / 2); y++) {
                for (int x = 320; x < bitmap.getWidth(); x++) {
                    int pixel = bitmap.getPixel(x, y);
                    if (red(pixel) >= RED_THRESHOLD && blue(pixel) <= BLUE_THRESHOLD && green(pixel) >= GREEN_THRESHOLD) {
                        xValues.add(x);
                        yValues.add(y);
                        yellowPixelCount++;
                    }
                }
            }

            if (yellowPixelCount > 4200){
                wobblePos = "C";
            }
            else if (yellowPixelCount > 1400){
                wobblePos = "B";
            }
            else{
                wobblePos = "A";
            }
            opMode.telemetry.addData("yellowPixCount: ", yellowPixelCount);
            opMode.telemetry.addData("wobblePos: ", wobblePos);
            opMode.telemetry.update();
            return wobblePos;
        }
    }
