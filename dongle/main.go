package main

import (
	"encoding/binary"
	"fmt"
	"machine/usb/hid/joystick"
	"math"
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

	buf := make([]byte, 12)
	updateCh := make(chan []byte, 1)

	err = initBLE(func(b []byte) {
		copy(buf, b)
		updateCh <- buf
	})
	if err != nil {
		return
	}

	i := 0

	declination := 5.32329
	yawOffset := 140.0

	for b := range updateCh {
		yaw := float64(math.Float32frombits(binary.LittleEndian.Uint32(b[0:4])))
		pitch := float64(math.Float32frombits(binary.LittleEndian.Uint32(b[4:8])))
		roll := float64(math.Float32frombits(binary.LittleEndian.Uint32(b[8:12])))

		yaw = yaw - declination
		yaw = math.Mod(yaw+180+yawOffset, 360) - 180
		if yaw < 0 {
			yaw += 360.0
		}
		if yaw >= 360.0 {
			yaw -= 360.0
		}

		joystick.Joystick.SetAxis(2, int(yaw*(65535.0/360.0)))
		joystick.Joystick.SetAxis(4, int(pitch*(65535.0/360.0))) // roll
		joystick.Joystick.SetAxis(3, int(roll*(65535.0/360.0)))  // pitch
		joystick.Joystick.SendState()

		i = (i + 1) % 24
		if i == 0 {
			setLed(blue, false)
		} else if i > 12 {
			setLed(blue, true)
		}
	}
}
