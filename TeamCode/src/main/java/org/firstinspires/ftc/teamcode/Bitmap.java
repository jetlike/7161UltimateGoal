package org.firstinspires.ftc.teamcode;

import com.vuforia.CameraDevice;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.ArrayList;
import java.util.Collections;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;

public class Bitmap {
    private AutoMaster auto = null;

    private final int RED_THRESHOLD = 40;
    private final int GREEN_THRESHOLD = 35;
    private final int BLUE_THRESHOLD = 35;


    CameraDevice camera;

    public static String skystonePosition = "notFound";


    public void Init(AutoMaster autoMaster) {
        auto = autoMaster;
        // init servo
    }

    public void Update(boolean grab) {
        // Grab amount
    }

    public android.graphics.Bitmap getBitmap() throws InterruptedException {

        VuforiaLocalizer.CloseableFrame picture;
        picture = auto.vision.vuforia.getFrameQueue().take();
        Image rgb = picture.getImage(1);

        long numImages = picture.getNumImages();

        auto.telemetry.addData("Num images", numImages);

        for (int i = 0; i < numImages; i++) {

            int format = picture.getImage(i).getFormat();
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

    public double getImageHeight() throws InterruptedException {
        android.graphics.Bitmap bitmap = getBitmap();
        return bitmap.getHeight();
    }

    public double getImageWidth() throws InterruptedException {
        android.graphics.Bitmap bitmap = getBitmap();
        return bitmap.getWidth();
    }

    //True for red
    public String Skystone(boolean red) throws InterruptedException {
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
                    for (int y = 224; y < bitmap.getHeight(); y++) {
                        for (int x = 0; x < bitmap.getWidth(); x++) {
                            int pixel = bitmap.getPixel(x, y);
                            if (red(pixel) <= RED_THRESHOLD && blue(pixel) <= BLUE_THRESHOLD && green(pixel) <= GREEN_THRESHOLD) {
                                xValues.add(x);
                                yValues.add(y);
                            }
                        }
                    }
                } else {
                    for (int y = 224; y < bitmap.getHeight(); y++) {
                        for (int x = 0; x < bitmap.getWidth(); x++) {
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
                auto.telemetry.addData("medX", medX);
                medY = yValues.get(yValues.size() / 2);
                avgX /= xValues.size();
                avgY /= yValues.size();
                if (red) {
                    if (medX < bitmap.getWidth() * .33) {    //0,0 starts at left top
                        skystonePosition = "1 & 4";
                        auto.telemetry.addData("skystonePosition: ", skystonePosition);
                    } else if (medX < bitmap.getWidth() * 0.66 && medX > bitmap.getWidth() * 0.34) {
                        skystonePosition = "2 & 5";
                        auto.telemetry.addData("skystonePosition: ", skystonePosition);
                    } else {
                        skystonePosition = "3 & 6";
                        auto.telemetry.addData("skystonePosition: ", skystonePosition);
                    }

                } else {
                    if (medX < bitmap.getWidth() * .33) {    //0,0 starts at left top
                        skystonePosition = "3 & 6";
                        auto.telemetry.addData("skystonePosition: ", skystonePosition);
                    } else if (medX < bitmap.getWidth() * 0.66 && medX > bitmap.getWidth() * 0.34) {
                        skystonePosition = "2 & 5";
                        auto.telemetry.addData("skystonePosition: ", skystonePosition);
                    } else {
                        skystonePosition = "1 & 4";
                        auto.telemetry.addData("skystonePosition: ", skystonePosition);
                    }
                }
                break;
            }
            return skystonePosition;
        } catch (IndexOutOfBoundsException e) {
            skystonePosition = "default";
            auto.telemetry.addData("skystonePosition:", skystonePosition);
            return skystonePosition;
        }
    }
}
