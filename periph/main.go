// The peripheral is a device that will be mounted to glasses or headphones
// and its responsibilities include querying accelerometer, gyroscope and
// magnetometer readings, performing attitude estimation and relaying the
// estimated data over to the dongle device.
package main

import (
	"encoding/binary"
	"fmt"
	"math"
	"time"

	"tinyht/pkg/ahrs"
)

const (
	deviceAddress         = "E1:81:D2:59:12:48"
	mdegToRad             = math.Pi / 180.0 / 1000000.0
	radToDeg              = 180.0 / math.Pi
	bleConnectionInterval = 7500 * time.Microsecond
	sensorPollFreq        = 476
	sensorPollInterval    = 2100840 * time.Nanosecond
)

func log(msg string, additional ...any) {
	if len(additional) > 0 {
		print(msg, ": ", fmt.Sprint(additional[0]), "\n")
	} else {
		println(msg)
	}
}

func main() {
	err := initLeds()
	if err != nil {
		return
	}

	err = initI2C()
	if err != nil {
		return
	}

	err = initIMU()
	if err != nil {
		return
	}

	err = initBLE()
	if err != nil {
		return
	}

	madg := ahrs.NewMadgwick(1)
	go loop(madg)

	for {
		sendEvents(madg)
	}
}

func sendEvents(madg *ahrs.Madgwick) {
	setLed(blue, true)

	dev, err := connect()
	if err != nil {
		return
	}
	defer func() {
		_ = dev.Disconnect()
	}()

	char, err := getCharacteristic(dev)
	if err != nil {
		return
	}

	setLed(blue, false)

	rot := make([]byte, 4*3)
	i := 0
	fails := 0

	for {
		start := time.Now()

		quat := madg.Quaternions
		roll, pitch, yaw := FromQuaternion(quat[0], quat[1], quat[2], quat[3])
		yaw = yaw * radToDeg
		pitch = pitch * radToDeg
		roll = roll * radToDeg

		binary.LittleEndian.PutUint32(rot[0:4], math.Float32bits(float32(yaw)))
		binary.LittleEndian.PutUint32(rot[4:8], math.Float32bits(float32(pitch)))
		binary.LittleEndian.PutUint32(rot[8:12], math.Float32bits(float32(roll)))

		_, err = char.WriteWithoutResponse(rot)
		if err != nil {
			setLed(blue, false)
			fails++
			if fails > 133 {
				// 1 second of failures? we probably lost connection
				break
			}
		} else {
			fails = 0

			i = (i + 1) % 24
			if i == 0 {
				setLed(blue, false)
			} else if i > 12 {
				setLed(blue, true)
			}
		}

		dt := time.Since(start)
		if dt < bleConnectionInterval {
			time.Sleep(bleConnectionInterval - dt)
		}
	}
}

func loop(madg *ahrs.Madgwick) {
	// no soft iron, on floor sensor
	//>+6.434198e+006 +1.079803e+005 +1.093925e+006
	ogx := int32(6434198)
	ogy := int32(107980)
	ogz := int32(1093925)

	omx := 16466.0
	omy := 2289.0
	omz := -36624.0

	i := 0

	for {
		start := time.Now()

		i++
		if i == sensorPollFreq {
			madg.Beta = 0.1
		}

		gx, gy, gz, err := imu.ReadRotation()
		if err != nil {
			continue
		}

		ax, ay, az, err := imu.ReadAcceleration()
		if err != nil {
			continue
		}

		mx, my, mz, err := imu.ReadMagneticField()
		if err != nil {
			continue
		}

		madg.Update9D(
			-float64(gx-ogx)*mdegToRad,
			float64(gy-ogy)*mdegToRad,
			float64(gz-ogz)*mdegToRad,
			-float64(ax),
			float64(ay),
			float64(az),
			float64(mx)-omx,
			float64(my)-omy,
			float64(mz)-omz,
			time.Since(start).Seconds(),
		)

		dt := time.Since(start)
		if dt < sensorPollInterval {
			time.Sleep(sensorPollInterval - dt)
		}
	}
}

func FromQuaternion(q0, q1, q2, q3 float64) (phi float64, theta float64, psi float64) {
	phi = math.Atan2(2*(q0*q1+q2*q3), q0*q0-q1*q1-q2*q2+q3*q3)

	v := -2 * (q0*q2 - q3*q1) / (q0*q0 + q1*q1 + q2*q2 + q3*q3)
	if v >= 1 {
		theta = math.Pi / 2
	} else if v <= -1 {
		theta = -math.Pi / 2
	} else {
		theta = math.Asin(v)
	}

	psi = math.Pi/2 - math.Atan2(2*(q0*q3+q1*q2), q0*q0+q1*q1-q2*q2-q3*q3)
	if psi < 0 {
		psi += 2 * math.Pi
	}

	return
}
