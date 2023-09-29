# tinyht

tinyht is head tracking software written in [TinyGo](https://github.com/tinygo-org/tinygo) that captures raw data from an inertial measurement unit,
runs it through an AHRS filtering algorithm, transfers the rotation data to a host dongle via bluetooth LE, and
sends the data as joystick events through USB.  

The joystick axes can then be used as rotation input for [OpenTrack](https://github.com/opentrack/opentrack), allowing to use the software as a
head tracker similar to TrackIR.  

This project is a work in progress, it generally works but requires modifying the source code and re-flashing for some changes (e.g. calibration).


License: MIT
