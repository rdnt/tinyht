package main

import (
	"encoding/binary"
	"encoding/json"
	"fmt"
	"io"
	"log"
	"math"

	"github.com/tarm/serial"
	"github.com/tracktum/go-ahrs"
	"github.com/x448/float16"

	// vjoy
	"github.com/tajtiattila/vjoy"
)

func main() {
	avail := vjoy.Available()
	if !avail {
		log.Println("vJoyInterface.dll could not be found.")
		log.Println("Please add it to your PATH environment variable.")
		return
	}

	d, err := vjoy.Acquire(1)
	if err != nil {
		fmt.Println(err)
		return
	}
	d.Reset()

	axes := []*vjoy.Axis{
		d.Axis(vjoy.AxisX),
		d.Axis(vjoy.AxisY),
		d.Axis(vjoy.AxisZ),
		d.Axis(vjoy.AxisRX),
		d.Axis(vjoy.AxisRY),
		d.Axis(vjoy.AxisRZ),
		d.Axis(vjoy.Slider0),
		d.Axis(vjoy.Slider1),
	}

	//buf := make([]byte, 200000)

	c := &serial.Config{Name: "COM8", Baud: 115200}
	s, err := serial.OpenPort(c)
	if err != nil {
		log.Fatal(err)
	}

	//rr := strings.NewReader("[1,2,3]\n[4,5,6]\n[7,8,9]\n")

	dec := json.NewDecoder(s)
	//for {
	//
	//	fmt.Printf("%#v\n", dest)
	//}

	//var i int
	//var n int

	madg := ahrs.NewMadgwick(0.1, 133.333333333)
	var gx, gy, gz, ax, ay, az, mx, my, mz float64

	for {
		buf := make([]byte, 20)
		if err := dec.Decode(&buf); err == io.EOF {
			break
		} else if err != nil {
			fmt.Println(err)
			continue
		}

		//n, err := s.Read(buf)
		//if err != nil {
		//	log.Fatal(err)
		//}

		//if n != 20 {
		//	continue
		//}

		//fmt.Printf("%x\n", buf)

		//gx = float64(int16(binary.LittleEndian.Uint16(buf[0:2]))) * 65536
		//gy = float64(int16(binary.LittleEndian.Uint16(buf[2:4]))) * 65536
		//gz = float64(int16(binary.LittleEndian.Uint16(buf[4:6]))) * 65536
		//ax = float64(int16(binary.LittleEndian.Uint16(buf[6:8]))) * 65536
		//ay = float64(int16(binary.LittleEndian.Uint16(buf[8:10]))) * 65536
		//az = float64(int16(binary.LittleEndian.Uint16(buf[10:12]))) * 65536
		//mx = float64(int16(binary.LittleEndian.Uint16(buf[12:14]))) * 65536
		//my = float64(int16(binary.LittleEndian.Uint16(buf[14:16]))) * 65536
		//mz = float64(int16(binary.LittleEndian.Uint16(buf[16:18]))) * 65536
		//dt = float64(int16(binary.LittleEndian.Uint16(buf[18:20]))) * 32768

		gx = float64(float16.Frombits(binary.LittleEndian.Uint16(buf[0:2])).Float32()) * 65536
		gy = float64(float16.Frombits(binary.LittleEndian.Uint16(buf[2:4])).Float32()) * 65536
		gz = float64(float16.Frombits(binary.LittleEndian.Uint16(buf[4:6])).Float32()) * 65536
		ax = float64(float16.Frombits(binary.LittleEndian.Uint16(buf[6:8])).Float32()) * 65536
		ay = float64(float16.Frombits(binary.LittleEndian.Uint16(buf[8:10])).Float32()) * 65536
		az = float64(float16.Frombits(binary.LittleEndian.Uint16(buf[10:12])).Float32()) * 65536
		mx = float64(float16.Frombits(binary.LittleEndian.Uint16(buf[12:14])).Float32()) * 65536
		my = float64(float16.Frombits(binary.LittleEndian.Uint16(buf[14:16])).Float32()) * 65536
		mz = float64(float16.Frombits(binary.LittleEndian.Uint16(buf[16:18])).Float32()) * 65536
		//dt = float64(float16.Frombits(binary.LittleEndian.Uint16(buf[18:20])).Float32())

		//  7035000.000000, 577500.000000, 1146250.000000, -23546.000000, 16531.000000, 1016931.000000, 33026.000000, -39578.000000, -213458.000000
		//  286676256.000000, -286720000.000000, -286720000.000000, 260653.000000, -1998604.000000, 88511.000000, 38346.000000, -18830.000000, -60046.000000

		//fmt.Printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", gx, gy, gz, ax, ay, az, mx, my, mz)

		ogx := float64(6743242)
		ogy := float64(489364)
		ogz := float64(547766)

		//ogx = int32(812490)
		//ogy = int32(7370154)
		//ogz = int32(1029393)

		omx := 2625.0
		omy := -10801.0
		omz := -32697.0

		omx = 16466.0
		omy = 2289.0
		omz = -36624.0

		mdegToRad := (math.Pi / 180.0 / 1000000.0)

		q := madg.Update9D(
			-float64(gx-ogx)*mdegToRad,
			float64(gy-ogy)*mdegToRad,
			float64(gz-ogz)*mdegToRad,
			-float64(ax)*101971.62129779,
			float64(ay)*101971.62129779,
			float64(az)*101971.62129779,
			float64(mx)-omx,
			float64(my)-omy,
			float64(mz)-omz,
		)

		yaw := -getYaw(q)
		//yaw = math.Mod(yaw+180, 360) - 180
		if yaw < 0 {
			yaw += 360.0
		}
		if yaw >= 360.0 {
			yaw -= 360.0
		}

		pitch := getPitch(q)
		roll := getRoll(q)

		pitch += 180
		roll += 180

		fmt.Printf("%.3f\t\t%.3f\t\t%.3f\n", yaw, pitch, roll)

		axes[0].Setuf(MarshalAngle(yaw))
		axes[1].Setuf(MarshalAngle(pitch))
		axes[2].Setuf(MarshalAngle(roll))

		err = d.Update()
		if err != nil {
			fmt.Print("!")
		}

		//js.SetAxis(2, int(rotX*(65535.0/360.0)))
		//js.SetAxis(4, int(rotY*(65535.0/360.0)))
		//js.SetAxis(3, int(rotZ*(65535.0/360.0)))
		//js.SendState()

		//err = d.Update()
		//if err != nil {
		//	fmt.Println(err)
		//}

		//fmt.Println(gx, "\t", gy, "\t", gz, "\t", ax, "\t", ay, "\t", az, "\t", mx, "\t", my, "\t", mz, "\t", dt)

		//
		//c, err := machine.Serial.ReadByte()
		//if err == nil {
		//	if c < 32 {
		//		// Convert nonprintable control characters to
		//		// ^A, ^B, etc.
		//		//machine.Serial.WriteByte('^')
		//		//machine.Serial.WriteByte(c + '@')
		//	} else if c >= 127 {
		//		// Anything equal or above ASCII 127, print ^?.
		//		//machine.Serial.WriteByte('^')
		//		//machine.Serial.WriteByte('?')
		//	} else {
		//		// Echo the printable character back to the
		//		// host computer.
		//		//machine.Serial.WriteByte(c)
		//
		//		buf[i] = c
		//		i++
		//
		//		if i == 20 {
		//			fmt.Printf("%x\n", buf)
		//			i = 0
		//		}
		//
		//	}
		//}

		// This assumes that the input is coming from a keyboard
		// so checking 120 times per second is sufficient. But if
		// the data comes from another processor, the port can
		// theoretically receive as much as 11000 bytes/second
		// (115200 baud). This delay can be removed and the
		// Serial.Read() method can be used to retrieve
		// multiple bytes from the receive buffer for each
		// iteration.
		//time.Sleep(time.Millisecond * 8)
	}
}

var radToDeg = 180.0 / math.Pi

func getYaw(q [4]float64) float64 {
	return math.Atan2(q[1]*q[2]+q[0]*q[3], 0.5-q[2]*q[2]-q[3]*q[3]) * radToDeg
}

func getRoll(q [4]float64) float64 {
	return math.Atan2(q[0]*q[1]+q[2]*q[3], 0.5-q[1]*q[1]-q[2]*q[2]) * radToDeg
}

func getPitch(q [4]float64) float64 {
	return math.Asin(2.0*(q[0]*q[2]-q[1]*q[3])) * radToDeg
}

// MarshalAngle prepares the angle to be sent to vJoy
func MarshalAngle(angle float64) float32 {
	return float32(angle) / 360
}
