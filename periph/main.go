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
	time.Sleep(1 * time.Second)
	println("initializing")

	pwm := machine.PWM0

	blue, err := pwm.Channel(machine.LED_BLUE)
	if err != nil {
		log("get pwm channel failed", err)
		return
	}
	pwm.SetInverting(blue, true)
	pwm.Set(blue, 0)

	err = pwm.Configure(machine.PWMConfig{
		Period: 1e9 / 1000,
	})
	if err != nil {
		log("pwm configure failed", err)
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

	time.Sleep(10 * time.Millisecond)

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
		err := connect(pwm, blue, imu)
		if err != nil {
			continue
		}
	}
}

// var mux sync.Mutex
var q = [4]float64{}
var rot = make([]byte, 4*3)

func loop(char bluetooth.DeviceCharacteristic, imu *lsm9ds1.Device) {
	madg := ahrs.NewMadgwick(2)
	madgref := &madg

	//go doMadgwick(ctx, madg)

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

//func doMadgwick(ctx context.Context, madg *ahrs.Madgwick) {
//	//ticker := time.NewTicker(time.Duration(int64(f)) * time.Nanosecond)
//	//defer ticker.Stop()
//
//	ogx := int32(6743242)
//	ogy := int32(489364)
//	ogz := int32(547766)
//
//	//ogx = int32(812490)
//	//ogy = int32(7370154)
//	//ogz = int32(1029393)
//
//	//812490.744, 7370154.308, 1029393.488
//
//	omx := 2625.0
//	omy := -10801.0
//	omz := -32697.0
//
//	omx = 16466.0
//	omy = 2289.0
//	omz = -36624.0
//
//	//omx = 0
//	//omy = 0
//	//omz = 0
//	//minx -34034 maxx 66976 miny -62398 maxy 25690 minz -79618 maxz 6370
//
//	//var ax, ay, az int32
//	//var gx, gy, gz int32
//	//var mx, my, mz int32
//
//	mdegToRad := (math.Pi / 180.0 / 1000000.0)
//
//	var pak [9]int32
//
//	q := [4]float64{}
//
//	for {
//		select {
//		case pak = <-fifo:
//			//fifolen--
//			//print("-", fifolen)
//
//			//println(fmt.Sprint(pak))
//			q = madg.Update9D(
//				-float64(pak[0]-ogx)*mdegToRad,
//				float64(pak[1]-ogy)*mdegToRad,
//				float64(pak[2]-ogz)*mdegToRad,
//				-float64(pak[3]), // *101971.62129779
//				float64(pak[4]),
//				float64(pak[5]),
//				float64(pak[6])-omx,
//				float64(pak[7])-omy,
//				float64(pak[8])-omz,
//			)
//
//			quatMux.Lock()
//			quat = q
//			quatMux.Unlock()
//		case <-ctx.Done():
//			return
//		}
//	}
//}

var device *bluetooth.Device

var fifo = make(chan [9]int32, 10)
var fifolen int
var quat = [4]float64{}
var quatMux sync.Mutex

func connect(pwm *machine.PWM, blue uint8, imu *lsm9ds1.Device) error {
	pwm.Set(blue, pwm.Top()*5/255)

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
		return err
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
			return err
		}
	}
	defer func() {
		_ = device.Disconnect()
	}()

	srvcs, err := device.DiscoverServices([]bluetooth.UUID{bluetooth.NewUUID(uuid.MustParse("e43e5547-afad-4309-81a4-1453c8fde090"))})
	if err != nil {
		log("service discovery failed", err)
		return err
	}

	if len(srvcs) == 0 {
		err := errors.New("could not find service")
		log("could not find service", err)
		return err
	}

	srvc := srvcs[0]

	chars, err := srvc.DiscoverCharacteristics([]bluetooth.UUID{
		bluetooth.NewUUID(uuid.MustParse("464e8563-de1d-4ff2-8e32-7761ce0620b6")),
	})
	if err != nil {
		log("could not find characteristics", err)
		return err
	}

	if len(chars) == 0 {
		err := errors.New("could not find characteristic")
		log("could not find characteristics", err)
		return err
	}

	accChar := chars[0]

	pwm.Set(blue, 0)

	loop(accChar, imu)

	return nil
}

func updateIMU2(imu *lsm9ds1.Device, madg *ahrs.Madgwick, char bluetooth.DeviceCharacteristic) {
	//f := 1000000000.0 / 476.0
	//ticker := time.NewTicker(time.Duration(int64(f)) * time.Nanosecond)
	//defer ticker.Stop()

	//ticker := time.NewTicker(7500 * time.Microsecond)
	//defer ticker.Stop()

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

	//var gxcal, gycal, gzcal float64
	//var msms float64

	//812490.744, 7370154.308, 1029393.488

	omx := 2625.0
	omy := -10801.0
	omz := -32697.0

	omx = 16466.0
	omy = 2289.0
	omz = -36624.0
	//minx -34034 maxx 66976 miny -62398 maxy 25690 minz -79618 maxz 6370

	var gx, gy, gz int32
	var ax, ay, az int32
	var mx, my, mz int32

	var err error

	mdegToRad := (math.Pi / 180.0 / 1000000.0)

	//var minx, miny, minz, maxx, maxy, maxz int32
	//
	//var ogxacc, ogyacc, ogzacc float64

	//var measurements float64

	//last := time.Now()
	//var dt float64
	//var start = time.Now()

	var start = time.Now()
	var dt = time.Duration(0)
	//var globdt = time.Duration(0)
	//var dti int64

	//var i int

	//var senses = [3][10]int32{
	//	//gx, gy, gz,
	//	//ax, ay, az,
	//	//mx, my, mz,
	//}

	//var fails int

	//var rotX, rotY, rotZ = make([]byte, 4), make([]byte, 4), make([]byte, 4)
	//var yaw, pitch, roll float64
	//var yaw32, pitch32, roll32 float32

	//q := [4]float64{}
	//
	//declination := 5.32329
	//yawOffset := 140.0

	start = time.Now()
	//var start2 = time.Now()

	ival := 1e9 / 476.0
	var intvl = time.Duration(ival) * time.Nanosecond
	println("ival", intvl)

	//var totalstart = time.Now()
	//var totaldur time.Duration
	//var totalit int

	//var globstart = time.Now()
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
		//startall = time.Now()
		//globstart = time.Now()
		//var got int
		//for i := 0; i < 3; i++ {
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
		//println(dt.Microseconds())
		//dti := dt.Microseconds()

		start = time.Now()

		//gxcal += float64(gx)
		//gycal += float64(gy)
		//gzcal += float64(gz)
		//
		//msms++

		//println(gxcal/msms, gycal/msms, gzcal/msms)
		//senses[0] = [10]int32{
		//	gx, gy, gz,
		//	ax, ay, az,
		//	mx, my, mz,
		//	0,
		//	//dt.Milliseconds(),
		//}
		////}
		//
		//senses[0][0] = senses[0][0] + senses[1][0] + senses[2][0]
		//senses[0][1] = senses[0][1] + senses[1][1] + senses[2][1]
		//senses[0][2] = senses[0][2] + senses[1][2] + senses[2][2]
		//
		//senses[0][3] = int32(float64(senses[0][3]+senses[1][3]+senses[2][3]) / 3.0)
		//senses[0][4] = int32(float64(senses[0][4]+senses[1][4]+senses[2][4]) / 3.0)
		//senses[0][5] = int32(float64(senses[0][5]+senses[1][5]+senses[2][5]) / 3.0)
		//
		//senses[0][6] = int32(float64(senses[0][6]+senses[1][6]+senses[2][6]) / 3.0)
		//senses[0][7] = int32(float64(senses[0][7]+senses[1][7]+senses[2][7]) / 3.0)
		//senses[0][8] = int32(float64(senses[0][8]+senses[1][8]+senses[2][8]) / 3.0)

		//mux.Lock()
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

		//q = madg.Update9D(
		//	-float64(gy-ogy)*mdegToRad,
		//	-float64(gx-ogx)*mdegToRad,
		//	float64(gz-ogz)*mdegToRad,
		//	-float64(ay)/1000000,
		//	-float64(ax)/1000000,
		//	float64(az)/1000000,
		//	float64(mx)-omx,
		//	float64(my)-omy,
		//	float64(mz)-omz,
		//	dt.Seconds(),
		//	//dt,
		//)
		//mux.Unlock()

		//q = madg.Update9D(
		//	-float64(gy-ogy)*mdegToRad,
		//	-float64(gx-ogx)*mdegToRad,
		//	float64(gz-ogz)*mdegToRad,
		//	-float64(ay), /**101971.62129779*/
		//	-float64(ax),
		//	float64(az),
		//	(float64(mx) - float64(omx)),
		//	(float64(my) - float64(omy)),
		//	(float64(mz) - float64(omz)),
		//	//dt,
		//)

		//	-float64(gy-ogy)*mdegToRad,
		//	-float64(gx-ogx)*mdegToRad,
		//	float64(gz-ogz)*mdegToRad,
		//	-float64(ay), /**101971.62129779*/
		//	-float64(ax),
		//	float64(az),
		//	(float64(mx) - float64(omx)),
		//	(float64(my) - float64(omy)),
		//	(float64(mz) - float64(omz)),

		// TODO only do these if we need to send packet
		//yaw = -getYaw(q)
		//yaw = yaw - declination
		//yaw = math.Mod(yaw+180+yawOffset, 360) - 180
		//if yaw < 0 {
		//	yaw += 360.0
		//}
		//if yaw >= 360.0 {
		//	yaw -= 360.0
		//}
		//yaw32 = float32(yaw)
		//
		//pitch = getPitch(q)
		//roll = getRoll(q)
		//
		////yaw, pitch, roll = FromQuaternion(quat[0], quat[1], quat[2], quat[3])
		//
		//pitch32 = float32(pitch)
		//roll32 = float32(roll)
		//
		////fmt.Printf("%.3f, %.3f, %.3f\n", yaw, pitch, roll)
		//
		//binary.LittleEndian.PutUint32(rot[0:4], math.Float32bits(yaw32))
		//binary.LittleEndian.PutUint32(rot[4:8], math.Float32bits(pitch32))
		//binary.LittleEndian.PutUint32(rot[8:12], math.Float32bits(roll32))

		//copy(rot[0:4], rotX)
		//copy(rot[4:8], rotY)
		//copy(rot[8:12], rotZ)

		//_, err = char.WriteWithoutResponse(rot)
		//if err != nil {
		//	fails++
		//	if fails > 133 {
		//		break
		//	}
		//} else {
		//	fails = 0
		//}

		//globdt = time.Since(globstart)
		//dti := globdt.Microseconds()
		//if globdt < intvl {
		//	println("sleep for", intvl-globdt)
		//	time.Sleep(intvl - globdt)
		//} else {
		//	print(">")
		//}

		//fifo <- [9]int32{
		//	gx, gy, gz,
		//	ax, ay, az,
		//	mx, my, mz,
		//}
		//dt = time.Since(start)
		//println("@@@@@@@", dt.Seconds()*1000.0, "@@@@@@@")
		//fifolen++
		//print("+", fifolen)

		//dt = time.Since(start)
		//println(dt.Seconds() * 1000.0)
		//start = time.Now()

		//dt = float64(time.Since(start)) / 1000000000.0
		//start = time.Now()
		//
		//if mx < minx {
		//	minx = mx
		//}
		//if mx > maxx {
		//	maxx = mx
		//}
		//
		//if my < miny {
		//	miny = my
		//}
		//if my > maxy {
		//	maxy = my
		//}
		//
		//if mz < minz {
		//	minz = mz
		//}
		//if mz > maxz {
		//	maxz = mz
		//}
		//
		//ogxacc += float64(gy)
		//ogyacc += float64(gx)
		//ogzacc += float64(gz)
		//
		//measurements++

		//fmt.Printf("%.3f, %.3f, %.3f\n", ogxacc/measurements, ogyacc/measurements, ogzacc/measurements)
		//madg.Update9D(
		//	-float64(gx-ogx)*mdegToRad,
		//	float64(gy-ogy)*mdegToRad,
		//	float64(gz-ogz)*mdegToRad,
		//	-float64(ax)*101971.62129779,
		//	float64(ay)*101971.62129779,
		//	float64(az)*101971.62129779,
		//	float64(mx)-omx,
		//	float64(my)-omy,
		//	float64(mz)-omz,
		//	//dt,
		//)

		//println(float64(time.Since(start)) / 1000.0)

		//madg.Update9D(
		//	-float64(gy-ogy)*mdegToRad,
		//	-float64(gx-ogx)*mdegToRad,
		//	float64(gz-ogz)*mdegToRad,
		//	-float64(ay), /**101971.62129779*/
		//	-float64(ax),
		//	float64(az),
		//	(float64(mx) - float64(omx)),
		//	(float64(my) - float64(omy)),
		//	(float64(mz) - float64(omz)),
		//)

		dtall = time.Since(startall)
		if dtall < 2101*time.Microsecond {
			time.Sleep(2101*time.Microsecond - dtall)
		} else {
			println("@")
		}
		startall = time.Now()
	}
}

//func updateIMU(ctx context.Context, imu *lsm9ds1.Device, madg *ahrs.Madgwick) {
//	f := 1000000000.0 / 476.0
//	ticker := time.NewTicker(time.Duration(int64(f)) * time.Nanosecond)
//	defer ticker.Stop()
//
//	fmt.Println(time.Duration(int64(f)) * time.Nanosecond)
//
//	ogx := int32(6743242)
//	ogy := int32(489364)
//	ogz := int32(547766)
//
//	//ogx = int32(812490)
//	//ogy = int32(7370154)
//	//ogz = int32(1029393)
//
//	//812490.744, 7370154.308, 1029393.488
//
//	omx := 2625.0
//	omy := -10801.0
//	omz := -32697.0
//
//	omx = 16466.0
//	omy = 2289.0
//	omz = -36624.0
//	//minx -34034 maxx 66976 miny -62398 maxy 25690 minz -79618 maxz 6370
//
//	var ax, ay, az int32
//	var gx, gy, gz int32
//	var mx, my, mz int32
//
//	var err error
//
//	mdegToRad := (math.Pi / 180.0 / 1000000.0)
//
//	//var minx, miny, minz, maxx, maxy, maxz int32
//	//
//	//var ogxacc, ogyacc, ogzacc float64
//
//	//var measurements float64
//
//	//last := time.Now()
//	//var dt float64
//	//var start = time.Now()
//
//	var start = time.Now()
//
//	for {
//		select {
//		case <-ticker.C:
//			start = time.Now()
//			gx, gy, gz, err = imu.ReadRotation()
//			if err != nil {
//				continue
//			}
//
//			ax, ay, az, err = imu.ReadAcceleration()
//			if err != nil {
//				continue
//			}
//
//			mx, my, mz, err = imu.ReadMagneticField()
//			if err != nil {
//				continue
//			}
//			println(time.Since(start))
//
//			//dt = float64(time.Since(start)) / 1000000000.0
//			//start = time.Now()
//			//
//			//if mx < minx {
//			//	minx = mx
//			//}
//			//if mx > maxx {
//			//	maxx = mx
//			//}
//			//
//			//if my < miny {
//			//	miny = my
//			//}
//			//if my > maxy {
//			//	maxy = my
//			//}
//			//
//			//if mz < minz {
//			//	minz = mz
//			//}
//			//if mz > maxz {
//			//	maxz = mz
//			//}
//			//
//			//ogxacc += float64(gy)
//			//ogyacc += float64(gx)
//			//ogzacc += float64(gz)
//			//
//			//measurements++
//
//			//fmt.Printf("%.3f, %.3f, %.3f\n", ogxacc/measurements, ogyacc/measurements, ogzacc/measurements)
//			madg.Update9D(
//				-float64(gx-ogx)*mdegToRad,
//				float64(gy-ogy)*mdegToRad,
//				float64(gz-ogz)*mdegToRad,
//				-float64(ax)*101971.62129779,
//				float64(ay)*101971.62129779,
//				float64(az)*101971.62129779,
//				float64(mx)-omx,
//				float64(my)-omy,
//				float64(mz)-omz,
//				//dt,
//			)
//
//			//println(float64(time.Since(start)) / 1000.0)
//
//			//madg.Update9D(
//			//	-float64(gy-ogy)*mdegToRad,
//			//	-float64(gx-ogx)*mdegToRad,
//			//	float64(gz-ogz)*mdegToRad,
//			//	-float64(ay), /**101971.62129779*/
//			//	-float64(ax),
//			//	float64(az),
//			//	(float64(mx) - float64(omx)),
//			//	(float64(my) - float64(omy)),
//			//	(float64(mz) - float64(omz)),
//			//)
//		case <-ctx.Done():
//			return
//		}
//	}
//}

//
//func sendEvents(char bluetooth.DeviceCharacteristic, madg *ahrs.Mahony) {
//	var rotX, rotY, rotZ = make([]byte, 4), make([]byte, 4), make([]byte, 4)
//	var yaw, pitch, roll float64
//	var yaw32, pitch32, roll32 float32
//
//	q := [4]float64{}
//
//	declination := 5.32329
//	yawOffset := 140.0
//
//	var err error
//
//	//ticker := time.NewTicker(7500 * time.Microsecond)
//	//defer ticker.Stop()
//
//	var fails int
//
//	for range ticker.C {
//		quatMux.Lock()
//		q = quat
//		quatMux.Unlock()
//
//		yaw = -getYaw(q)
//		yaw = yaw - declination
//		yaw = math.Mod(yaw+180+yawOffset, 360) - 180
//		if yaw < 0 {
//			yaw += 360.0
//		}
//		if yaw >= 360.0 {
//			yaw -= 360.0
//		}
//		yaw32 = float32(yaw)
//
//		pitch = getPitch(q)
//		roll = getRoll(q)
//
//		//yaw, pitch, roll = FromQuaternion(quat[0], quat[1], quat[2], quat[3])
//
//		pitch32 = float32(pitch)
//		roll32 = float32(roll)
//
//		//fmt.Printf("%.3f, %.3f, %.3f\n", yaw, pitch, roll)
//
//		//pi16 := float16.Fromfloat32(pi)
//		//pi16.Bits()
//
//		binary.LittleEndian.PutUint32(rotX, math.Float32bits(yaw32))
//		binary.LittleEndian.PutUint32(rotY, math.Float32bits(pitch32))
//		binary.LittleEndian.PutUint32(rotZ, math.Float32bits(roll32))
//
//		copy(rot[0:4], rotX)
//		copy(rot[4:8], rotY)
//		copy(rot[8:12], rotZ)
//
//		_, err = char.WriteWithoutResponse(rot)
//		if err != nil {
//			fails++
//			if fails > 133 {
//				break
//			}
//		} else {
//			fails = 0
//		}
//	}
//}

var radToDeg = 180.0 / math.Pi

func getYaw(q [4]float64) float64 {
	//https://github.com/westphae/quaternion/blob/master/quaternion.go
	return math.Atan2(2*(q[1]*q[2]+q[0]*q[3]), 1-2*(q[2]*q[2]+q[3]*q[3])) * radToDeg

	//return math.Atan2(q[1]*q[2]+q[0]*q[3], 0.5-q[2]*q[2]-q[3]*q[3]) * radToDeg
}

func getPitch(q [4]float64) float64 {
	return math.Asin(2.0*(q[0]*q[2]-q[1]*q[3])) * radToDeg
}

func getRoll(q [4]float64) float64 {

	return math.Atan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2])) * radToDeg

	//return math.Atan2(q[0]*q[1]+q[2]*q[3], 0.5-q[1]*q[1]-q[2]*q[2]) * radToDeg
}

func FromQuaternion(q0, q1, q2, q3 float64) (phi float64, theta float64, psi float64) {
	phi = math.Atan2(2*(q0*q1+q2*q3), (q0*q0 - q1*q1 - q2*q2 + q3*q3))

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

func FromQuaternionOld(q0, q1, q2, q3 float64) (phi float64, theta float64, psi float64) {
	phi = math.Atan2(2*(q0*q1+q2*q3), (q0*q0 - q1*q1 - q2*q2 + q3*q3))

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
