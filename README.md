# ROS2 Waveshare Servo Driver Node

Dieses Repository enthält den Code für einen ROS2 Node, der die bidirektionale Kommunikation mit Waveshare Servo Modulen ermöglicht. Der Node sendet Steuerbefehle an die Servos und empfängt gleichzeitig Feedback (z. B. Joint States).

## Inhalt und Zielsetzung

- **Hardware-Kommunikation:** Ansteuerung und Überwachung von Servomotoren.
- **Bidirektionale Datenübertragung:** Senden von Befehlen und Empfangen von Sensordaten.
- **Lehrinhalt:** Grundlagen der Integration von Hardware und Software in ROS2.

## Hintergrundwissen

- **Servo Driver:** Wie Servomodule angesteuert werden und welche Rückmeldungen (Feedback) möglich sind.
- **ROS2 Nachrichten:** Nutzung von Standard-Nachrichtentypen (z. B. `sensor_msgs/JointState`).
- **Kommunikationsprotokolle:** Grundlagen der seriellen bzw. I²C/SPI-Kommunikation (je nach Hardware).

## Voraussetzungen

- Verständnis der Hardware-Ansteuerung und Sensorik.
- Grundkenntnisse in ROS2 und den entsprechenden Nachrichtentypen.
- Interesse an praktischer Hardware-Integration.

## Aufbau des Codes

- **Ausgabe:** Senden von Joint Commands an die Servos.
- **Eingabe:** Empfang von Feedback-Daten (Joint States) zur Überwachung.
- **Synchronisation:** Weitergabe der Daten an andere Nodes wie den State Estimator.

## Wie baue und starte ich den Container?

1. Repository klonen:
   ```bash
   git clone https://github.com/wggRobotic/ros2_waveshare_servo_driver_node.git
   cd ros2_waveshare_servo_driver_node
2. Docker-Image bauen:
   ```bash
   docker build -t ros2_waveshare_servo_driver_node .

3. Container starten:
   ```bash
   docker run --rm   --network host   --device /dev/board_front   --device /dev/board_back ros2_waveshare_servo_driver_node