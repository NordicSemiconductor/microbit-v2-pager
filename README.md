# micro:bit v2 Pager

This is a sample nRF Connect SDK application for the micro:bit v2.
The one-way Bluetooth LE pager allows you to send text messages to the 5x5 LED matrix on the micro:bit v2 using your smartphone. 
The pager sample utilizes Nordic UART Service (NUS) available in nRF Connect SDK to receive messages through Bluetooth LE and then forward them to the 5x5 LED matrix on the micro:bit v2.

How to use:

1.  Build&flash this sample on your micro:bit v2.

2.  Using nRF Connect for Mobile:

    2.1 Scan for devices.

    2.2 Connect to micro:bit v2 Pager.

    2.3 Type your message in the Rx Characteristic box and click send.

3.  You will first hear a beep for 1.5 seconds followed by your message displayed on the micro:bit v2 5x5 LED matrix.

(Optional) micro:bit v2 interface:
You could re-display the recently received message(if one exists) by pressing button B. Pressing button A will delete the recently received message. 
