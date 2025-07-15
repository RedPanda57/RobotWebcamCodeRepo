//package org.firstinspires.ftc.teamcode; // Zorg dat dit overeenkomt met je teamcode package
//
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.MatOfPoint;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvPipeline; // Belangrijk: EasyOpenCV import
//
//import java.util.ArrayList;
//import java.util.List;
//
//public class VisionPipeline extends OpenCvPipeline {
//
//    // Dit zijn Mat objecten die we zullen hergebruiken voor verwerking.
//    // Het is efficiënter om Mat objecten te hergebruiken dan steeds nieuwe te maken
//    // voor elk frame. Dit voorkomt geheugenproblemen.
//    Mat hsv = new Mat();
//    Mat mask = new Mat();
//    Mat hierarchy = new Mat(); // Voor contouren, indien nodig
//
//    /**
//     * Deze methode wordt automatisch en continu aangeroepen voor elk frame van de camera.
//     * Hier schrijf je al je beeldverwerkingslogica.
//     *
//     * @param input Het originele camerabeeld (een Mat object), meestal in RGBA formaat.
//     * @return Een Mat object dat het verwerkte beeld vertegenwoordigt. Dit beeld wordt getoond op het Driver Station.
//     */
//    @Override
//    public Mat processFrame(Mat input) {
//        // 'input' is het originele camerabeeld dat binnenkomt (RGBA formaat)
//
//        // Voorbeeld: Converteer het beeld naar de HSV kleurruimte.
//        // HSV (Hue, Saturation, Value) is beter voor kleurdetectie dan RGB.
//        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
//
//        // Voorbeeld: Detecteer een specifieke kleur (hier: rood)
//        // 'Scalar' definieert de lage en hoge drempelwaarden voor de kleur in HSV.
//        // Je zult deze waarden moeten afstemmen op de specifieke kleur die je wilt detecteren
//        // onder de lichtomstandigheden van de competitie.
//        Scalar lowerRed = new Scalar(0, 100, 100);  // Lage HSV-waarden voor rood
//        Scalar upperRed = new Scalar(10, 255, 255); // Hoge HSV-waarden voor rood
//        Core.inRange(hsv, lowerRed, upperRed, mask); // Creëert een binair masker (wit waar rood is, zwart elders)
//
//        // Optioneel: Vind contouren (om de omtrek van gedetecteerde objecten te vinden)
//        List<MatOfPoint> contours = new ArrayList<>();
//        // Imgproc.findContours zoekt naar aaneengesloten witte gebieden in het 'mask'.
//        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//
//        // Optioneel: Teken de gevonden contouren op het originele input frame.
//        // Dit helpt je te visualiseren wat de robot detecteert.
//        // Hier tekenen we groene contouren met een dikte van 2 pixels.
//        Imgproc.drawContours(input, contours, -1, new Scalar(0, 255, 0), 2); // -1 betekent alle contouren
//
//        // Belangrijk: Geef de Mat terug die je wilt tonen op het Driver Station.
//        // - Als je het originele beeld wilt tonen met getekende contouren: return input;
//        // - Als je het binaire masker (wit/zwart) wilt tonen: return mask;
//        // - Als je het HSV-beeld wilt tonen: return hsv;
//        // Zorg ervoor dat je alle Mat objecten die je NIET teruggeeft, 'released' (vrijgeeft)
//        // om geheugenlekken te voorkomen. Als je ze niet released, zal het geheugen vol raken!
//        hsv.release(); // Geef het geheugen van de HSV Mat vrij
//        hierarchy.release(); // Geef het geheugen van de hierarchy Mat vrij
//        // Als je 'mask' niet retourneert, release deze dan ook: mask.release();
//
//        return input; // Retourneer het frame met de getekende contouren
//    }
//
//    /**
//     * Deze methode wordt aangeroepen wanneer de camera-viewport op het Driver Station wordt aangetikt.
//     * Je kunt dit gebruiken voor debug-functionaliteit, zoals het wisselen van weergavemodi.
//     */
//    @Override
//    public void onViewportTapped() {
//        // Optioneel: Voeg hier logica toe voor wanneer de viewport op het Driver Station wordt aangetikt.
//        // Bijvoorbeeld: wissel tussen debug-modi.
//    }
//}
