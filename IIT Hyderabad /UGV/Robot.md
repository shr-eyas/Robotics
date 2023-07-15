Path Following Robot 

The idea of this project is to make a path following robot which will trace the exact path given to it as an input with the help
of a feedback control system. A mobile phone camera will feed the live video of environment to a server on the phone. This feed
will be received on a system connected to the same Wi-Fi network using OpenCV for processing. After processing, a server on the
system will further send commands to a onboard microcontroller, ESP32 which is again connected to the same Wi-Fi network.
This microcontroller board will command the motors to move such that the given path is traced.
