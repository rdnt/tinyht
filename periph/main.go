// The peripheral is a device that will be mounted to glasses or headphones
// and its responsibilities include querying accelerometer, gyroscope and
// magnetometer readings, performing attitude estimation and relaying the
// estimated data over to the dongle device.
package main

import (
	"encoding/binary"
	"errors"
	"machine"
	"math"
	"sync"
	"time"

	"github.com/google/uuid"
	"tinygo.org/x/bluetooth"
	"tinygo.org/x/drivers/lsm9ds1"

	"edht/pkg/ahrs"
)

var adapter = bluetooth.DefaultAdapter

const deviceAddress = "E1:81:D2:59:12:48"

func log(context string, err ...error) {
	if len(err) > 0 {
		println(context, err[0].Error())
	} else {
		println(context)
	}
}

func main() {
	//time.Sleep(1 * time.Second)
	println("initializing")

	err := initLeds()
	if err != nil {
		return
	}

	println("i2c configure")

	err = machine.I2C0.Configure(machine.I2CConfig{
		Frequency: 400 * machine.KHz,
		SDA:       machine.SDA0_PIN,
		SCL:       machine.SCL0_PIN,
	})
	if err != nil {
		log("i2c configure failed", err)
		return
	}

	//time.Sleep(10 * time.Millisecond)

	println("lsm9ds1 configure")

	imu := lsm9ds1.New(machine.I2C0)
	err = imu.Configure(lsm9ds1.Configuration{
		AccelRange:      lsm9ds1.ACCEL_2G,
		AccelSampleRate: lsm9ds1.ACCEL_SR_476,
		GyroRange:       lsm9ds1.GYRO_250DPS,
		GyroSampleRate:  lsm9ds1.GYRO_SR_476,
		MagRange:        lsm9ds1.MAG_4G,
		MagSampleRate:   lsm9ds1.MAG_SR_80,
	})
	if err != nil {
		log("lsm9ds1 configure failed", err)
		return
	}

	println("adapter configure")

	err = adapter.Enable()
	if err != nil {
		log("ble enable failed", err)
		return
	}

	for {
		connect(blue, imu)
	}
}

var q = [4]float64{}
var rot = make([]byte, 4*3)

func loop(char bluetooth.DeviceCharacteristic, imu *lsm9ds1.Device) {
	madg := ahrs.NewMadgwick(2)
	madgref := &madg

	go func() {

		var yaw, pitch, roll float64
		var yaw32, pitch32, roll32 float32

		//q := [4]float64{}

		//declination := 5.32329
		//yawOffset := 140.0 - 82

		var intvl = 7500 * time.Microsecond
		var dt time.Duration
		var err error
		var start time.Time

		//var dti int64
		for {
			start = time.Now()

			//q = madgref.Quaternions

			roll, pitch, yaw = FromQuaternion(q[0], q[1], q[2], q[3])
			yaw = yaw * radToDeg
			pitch = pitch * radToDeg
			roll = roll * radToDeg
			//yaw = getYaw(q)
			//pitch = getPitch(q)
			//roll = getRoll(q)

			//yaw = yaw - declination
			//yaw = math.Mod(yaw+180+yawOffset, 360) - 180
			//if yaw < 0 {
			//	yaw += 360.0
			//}
			//if yaw >= 360.0 {
			//	yaw -= 360.0
			//}

			//yaw, pitch, roll = FromQuaternion(quat[0], quat[1], quat[2], quat[3])

			yaw32 = float32(yaw)
			pitch32 = float32(pitch)
			roll32 = float32(roll)

			//fmt.Printf("%.3f, %.3f, %.3f\n", yaw, pitch, roll)

			binary.LittleEndian.PutUint32(rot[0:4], math.Float32bits(yaw32))
			binary.LittleEndian.PutUint32(rot[4:8], math.Float32bits(pitch32))
			binary.LittleEndian.PutUint32(rot[8:12], math.Float32bits(roll32))

			_, err = char.WriteWithoutResponse(rot)

			dt = time.Since(start)
			if err != nil {
				print("!")
			}
			//if err != nil {
			//	fails++
			//	if fails > 133 {
			//		break
			//	}
			//} else {
			//	fails = 0
			//}

			//dti = dt.Microseconds()
			//println(dti)

			if dt < intvl {
				time.Sleep(intvl - dt)
			} else {
				print("[")
			}
		}

	}()

	updateIMU2(
		imu,
		madgref,
		char,
	)
	//sendEvents(char, madg)
}

var device *bluetooth.Device

var fifo = make(chan [9]int32, 10)
var fifolen int
var quat = [4]float64{}
var quatMux sync.Mutex

func connect(blue uint8, imu *lsm9ds1.Device) {
	setLed(blue, true)

	ch := make(chan bluetooth.ScanResult, 1)

	println("scan")

	err := adapter.Scan(func(adapter *bluetooth.Adapter, result bluetooth.ScanResult) {
		println("device", result.Address.String())
		if result.Address.String() == deviceAddress {
			err := adapter.StopScan()
			if err != nil {
				log("ble stop scan failed", err)
			}
			ch <- result
		}
	})
	if err != nil {
		log("ble scan failed", err)
		return
	}

	select {
	case result := <-ch:
		device, err = adapter.Connect(result.Address, bluetooth.ConnectionParams{
			ConnectionTimeout: bluetooth.NewDuration(4 * time.Second),
			MinInterval:       bluetooth.NewDuration(7500 * time.Microsecond),
			MaxInterval:       bluetooth.NewDuration(7500 * time.Microsecond),
		})
		if err != nil {
			log("ble connect failed", err)
			return
		}
	}
	defer func() {
		_ = device.Disconnect()
	}()

	srvcs, err := device.DiscoverServices([]bluetooth.UUID{bluetooth.NewUUID(uuid.MustParse("e43e5547-afad-4309-81a4-1453c8fde090"))})
	if err != nil {
		log("service discovery failed", err)
		return
	}

	if len(srvcs) == 0 {
		err := errors.New("could not find service")
		log("could not find service", err)
		return
	}

	srvc := srvcs[0]

	chars, err := srvc.DiscoverCharacteristics([]bluetooth.UUID{
		bluetooth.NewUUID(uuid.MustParse("464e8563-de1d-4ff2-8e32-7761ce0620b6")),
	})
	if err != nil {
		log("could not find characteristics", err)
		return
	}

	if len(chars) == 0 {
		err := errors.New("could not find characteristic")
		log("could not find characteristics", err)
		return
	}

	accChar := chars[0]

	setLed(blue, false)

	loop(accChar, imu)

	return
}

func updateIMU2(imu *lsm9ds1.Device, madg *ahrs.Madgwick, char bluetooth.DeviceCharacteristic) {
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
	var dtall = time.Duration(0)
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

		start = time.Now()

		q = madg.Update9D(
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

		dtall = time.Since(startall)
		if dtall < 2101*time.Microsecond {
			time.Sleep(2101*time.Microsecond - dtall)
		} else {
			println("@")
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
