package main

import (
	"machine"
	"time"

	"github.com/google/uuid"
	"tinygo.org/x/bluetooth"
	"tinygo.org/x/drivers/lsm9ds1"
)

var adapter = bluetooth.DefaultAdapter

const deviceAddress = "E1:81:D2:59:12:48"

var device *bluetooth.Device

func main() {
	time.Sleep(1 * time.Second)

	ch := make(chan bluetooth.ScanResult, 1)

	println("scan")

	pwm := machine.PWM0

	blue, err := pwm.Channel(machine.LED_BLUE)
	if err != nil {
		print(err)
		return
	}
	pwm.SetInverting(blue, true)
	pwm.Set(blue, 0)

	err = pwm.Configure(machine.PWMConfig{
		Period: 1e9 / 1000,
	})
	if err != nil {
		print(err)
		return
	}

	err = machine.I2C0.Configure(machine.I2CConfig{
		Frequency: 100 * machine.KHz,
		SDA:       machine.SDA0_PIN,
		SCL:       machine.SCL0_PIN,
	})
	if err != nil {
		print(err)
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
		print(err)
		return
	}

	err = adapter.Enable()
	if err != nil {
		print(err)
		return
	}

	err = adapter.Scan(func(adapter *bluetooth.Adapter, result bluetooth.ScanResult) {
		println("device", result.Address.String())
		if result.Address.String() == deviceAddress {
			err := adapter.StopScan()
			if err != nil {
				print(err)
			}
			ch <- result
		}
	})
	if err != nil {
		print(err)
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
			print(err)
			return
		}
	}
	defer func() {
		_ = device.Disconnect()
	}()

	srvcs, err := device.DiscoverServices([]bluetooth.UUID{bluetooth.NewUUID(uuid.MustParse("e43e5547-afad-4309-81a4-1453c8fde090"))})
	if err != nil {
		print(err)
		return
	}

	if len(srvcs) == 0 {
		print("no svc")
		return
	}

	srvc := srvcs[0]

	chars, err := srvc.DiscoverCharacteristics([]bluetooth.UUID{
		bluetooth.NewUUID(uuid.MustParse("464e8563-de1d-4ff2-8e32-7761ce0620b6")),
	})
	if err != nil {
		print(err)
		return
	}

	if len(chars) == 0 {
		print("no chars")
		return
	}

	accChar := chars[0]

	for {
		funcWithTicker(imu, accChar)
		time.Sleep(1 * time.Second)
	}
}

var rot = make([]byte, 4*3)

func funcWithTicker(imu *lsm9ds1.Device, char bluetooth.DeviceCharacteristic) {
	//ticker := time.NewTicker(7500 * time.Microsecond)
	//defer ticker.Stop()

	for {
		_, err := char.WriteWithoutResponse(rot)
		if err != nil {
			print(err)
		} else {
			print(".")
		}
		print(imu)
		time.Sleep(7500 * time.Microsecond)
	}
}
