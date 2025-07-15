//package org.firstinspires.ftc.teamcode; // Zorg dat dit overeenkomt met je teamcode package
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp; // Belangrijk: Importeer TeleOp
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous; // Optioneel: Importeer Autonomous
//import com.qualcomm.robotcore.eventloop.opmode.Disabled; // Optioneel: Importeer Disabled
//
////import com.qualcomm.robotcore.hardware.WebcamName; // JUISTE IMPORT: Importeer de klasse WebcamName
//
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvWebcam; // Voor USB-webcams
//
//// Dit zijn de annotaties die je DIRECT BOVEN DE KLASSEDIFINITIE plaatst.
//// Kies EEN van de volgende: @TeleOp of @Autonomous.
//// Je kunt ook @Disabled toevoegen als je de OpMode tijdelijk niet wilt zien.
//
//@TeleOp(name="OpenCV Camera Test", group="Robot") // Deze OpMode verschijnt op het Driver Station
//// @Autonomous(name="OpenCV Autonomous Test", group="Robot") // Voor een autonome OpMode
//// @Disabled // Gebruik deze om de OpMode tijdelijk te verbergen op het Driver Station
//public class OpenCVCameraTestOpMode extends LinearOpMode {
//
//    OpenCvWebcam webcam; // Declareer je webcam object
//
//    @Override
//    public void runOpMode() {
//        // 1. Verkrijg de ID van de camera-monitor (voor het tonen van het beeld op het Driver Station)
//        // Dit is een standaard ID die de FTC SDK gebruikt om de camera-feed te tonen.
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//        // 2. Initialiseer de webcam
//        // Gebruik de hardwareMap om de USB-webcam te vinden.
//        // "Webcam 1" moet overeenkomen met de naam die je in de Control Hub configuratie hebt ingesteld.
//        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        // 3. Stel je pipeline in
//        // Hier koppel je je custom MyVisionPipeline aan de webcam.
//        webcam.setPipeline(new VisionPipeline());
//
//        // 4. Start de camera
//        // Dit opent de camera-verbinding asynchroon (op de achtergrond).
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                // Camera is succesvol geopend, start de stream
//                // Kies de resolutie die het beste past bij je behoeften.
//                // Lagere resoluties (bijv. 320x240) zijn sneller te verwerken.
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//                telemetry.addData("Camera Status", "Stream gestart");
//                telemetry.update();
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                // Er is een fout opgetreden bij het openen van de camera
//                telemetry.addData("Camera Fout", "Code: " + errorCode);
//                telemetry.update();
//            }
//        });
//
//        telemetry.addData("Status", "Wacht op start");
//        telemetry.update();
//
//        waitForStart(); // Wacht tot de startknop op het Driver Station wordt ingedrukt
//
//        while (opModeIsActive()) {
//            // Dit is de hoofdloop van je OpMode.
//            // Je kunt hier telemetry-data toevoegen of robotbewegingen aansturen.
//            // De OpenCV-pipeline draait op de achtergrond.
//
//            // Voorbeeld: Toon de huidige framerate van de camera
//            telemetry.addData("FPS", webcam.getFps());
//            telemetry.update();
//
//            sleep(50); // Korte pauze om de Control Hub niet te overbelasten
//        }
//
//        // Stop de camera wanneer de OpMode eindigt
//        webcam.stopStreaming();
//        webcam.closeCameraDevice();
//    }
//}
//
