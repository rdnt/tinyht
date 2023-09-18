package main

import (
	"encoding/binary"
	"machine"
	"machine/usb/hid/joystick"
	"math"
	"time"

	"github.com/google/uuid"
	"tinygo.org/x/bluetooth"
)

var (
	adapter = bluetooth.DefaultAdapter
)

var serviceUUID = bluetooth.NewUUID(uuid.MustParse("e43e5547-afad-4309-81a4-1453c8fde090"))
var rotUUID = bluetooth.NewUUID(uuid.MustParse("464e8563-de1d-4ff2-8e32-7761ce0620b6"))

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

	err = pwm.Configure(machine.PWMConfig{
		Period: 1e9 / 1000,
	})
	if err != nil {
		log("pwm configure failed", err)
		return
	}

	println("i2c configure")

	var js = joystick.Port()

	//log.SetFlags(log.Lmicroseconds)

	println("enabling")

	var rotX, rotY, rotZ float32
	//var rotX, rotY float32

	//update := func() {
	//	js.SetAxis(2, int(rotX*(65535.0/360.0)))
	//	js.SetAxis(4, int(rotY*(65535.0/360.0)))
	//	js.SetAxis(3, int(rotZ*(65535.0/360.0)))
	//	js.SendState()
	//}

	println("starting")
	must("enable BLE stack", adapter.Enable())
	adv := adapter.DefaultAdvertisement()
	must("config adv", adv.Configure(bluetooth.AdvertisementOptions{
		LocalName:    "EDHT",
		ServiceUUIDs: []bluetooth.UUID{serviceUUID},
	}))

	var rotChar bluetooth.Characteristic

	bufRot := make([]byte, 12)

	var changed bool

	must("add serviceUUID", adapter.AddService(&bluetooth.Service{
		UUID: serviceUUID,
		Characteristics: []bluetooth.CharacteristicConfig{
			{
				Handle: &rotChar,
				UUID:   rotUUID,
				Flags:  bluetooth.CharacteristicWritePermission | bluetooth.CharacteristicWriteWithoutResponsePermission,
				WriteEvent: func(client bluetooth.Connection, offset int, buf []byte) {
					if offset != 0 || len(buf) != 12 {
						return
					}

					copy(bufRot, buf)

					changed = true
				},
			},
		},
	}))

	// Start advertising
	must("start adv", adv.Start())

	var pak int

	println("looping")

	for range time.Tick(7500 * time.Microsecond) {
		if !changed {
			pwm.Set(blue, 0)
			continue
		}
		changed = false
		print(".")

		rotX = math.Float32frombits(binary.LittleEndian.Uint32(bufRot[0:4]))
		rotY = math.Float32frombits(binary.LittleEndian.Uint32(bufRot[4:8]))
		rotZ = math.Float32frombits(binary.LittleEndian.Uint32(bufRot[8:12]))

		//fmt.Printf("%.3f, %.3f, %.3f\n", rotX, rotY, rotZ)

		print(".")

		js.SetAxis(2, int(rotX*(65535.0/360.0))) // yaw
		js.SetAxis(4, int(rotY*(65535.0/360.0))) // roll
		js.SetAxis(3, int(rotZ*(65535.0/360.0))) // pitch
		js.SendState()

		if pak == 0 {
			pwm.Set(blue, 0)
		}

		pak = (pak + 1) % 24

		if pak > 12 {
			pwm.Set(blue, pwm.Top()*5/255)
		}
	}
}

func must(action string, err error) {
	if err != nil {
		panic("failed to " + action + ": " + err.Error())
	}
}
