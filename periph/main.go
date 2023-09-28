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

	"tinygo.org/x/bluetooth"

	"edht/pkg/ahrs"
)

const deviceAddress = "E1:81:D2:59:12:48"

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
		run(madg)
	}
}

func run(madg *ahrs.Madgwick) {
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

	sendEvents(madg, char)
}

func sendEvents(madg *ahrs.Madgwick, char bluetooth.DeviceCharacteristic) {
	var yaw, pitch, roll float64
	var yaw32, pitch32, roll32 float32
	rot := make([]byte, 4*3)

	intvl := 7500 * time.Microsecond
	var start time.Time
	var dt time.Duration

	var err error
	var fails int

	var quat [4]float64

	for {
		start = time.Now()

		quat = madg.Quaternions
		roll, pitch, yaw = FromQuaternion(quat[0], quat[1], quat[2], quat[3])
		yaw = yaw * radToDeg
		pitch = pitch * radToDeg
		roll = roll * radToDeg

		yaw32 = float32(yaw)
		pitch32 = float32(pitch)
		roll32 = float32(roll)

		binary.LittleEndian.PutUint32(rot[0:4], math.Float32bits(yaw32))
		binary.LittleEndian.PutUint32(rot[4:8], math.Float32bits(pitch32))
		binary.LittleEndian.PutUint32(rot[8:12], math.Float32bits(roll32))

		_, err = char.WriteWithoutResponse(rot)
		if err != nil {
			fails++
			if fails > 133 {
				break
			}
		} else {
			fails = 0
		}

		dt = time.Since(start)
		if dt < intvl {
			time.Sleep(intvl - dt)
		}
	}
}

func loop(madg *ahrs.Madgwick) {
	ogx := int32(6743242)
	ogy := int32(489364)
	ogz := int32(547766)

	ogx = int32(812490)
	ogy = int32(7370154)
	ogz = int32(1029393)

	// no soft iron, on floor sensor
	//>+6.434198e+006 +1.079803e+005 +1.093925e+006
	ogx = int32(6434198)
	ogy = int32(107980)
	ogz = int32(1093925)

	omx := 2625.0
	omy := -10801.0
	omz := -32697.0

	omx = 16466.0
	omy = 2289.0
	omz = -36624.0

	var gx, gy, gz int32
	var ax, ay, az int32
	var mx, my, mz int32

	var err error

	mdegToRad := (math.Pi / 180.0 / 1000000.0)

	var start = time.Now()
	var dt = time.Duration(0)

	ival := 1e9 / 476.0
	var intvl = time.Duration(ival) * time.Nanosecond
	println("ival", intvl)

	start = time.Now()
	var startall = time.Now()
	var dtloop time.Duration
	var it = 0
	for {
		it++
		if it == 500 {
			madg.Beta = 0.1
			print(">>>")
		}

		gx, gy, gz, err = imu.ReadRotation()
		if err != nil {
			continue
		}

		ax, ay, az, err = imu.ReadAcceleration()
		if err != nil {
			continue
		}

		mx, my, mz, err = imu.ReadMagneticField()
		if err != nil {
			continue
		}

		dt = time.Since(start)

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
			dt.Seconds(),
		)

		dtloop = time.Since(startall)
		start = time.Now()
		if dtloop < 2101*time.Microsecond {
			time.Sleep(2101*time.Microsecond - dtloop)
		}
		startall = time.Now()
	}
}

var radToDeg = 180.0 / math.Pi

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
	psi = math.Pi/2 - math.Atan2(2*(q0*q3+q1*q2), (q0*q0+q1*q1-q2*q2-q3*q3))
	if psi < 0 {
		psi += 2 * math.Pi
	}
	return
}
