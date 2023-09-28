package main

import (
	"machine"

	"tinygo.org/x/drivers/lsm9ds1"
)

var imu *lsm9ds1.Device

func initI2C() error {
	err := machine.I2C0.Configure(machine.I2CConfig{
		Frequency: 400 * machine.KHz,
		SDA:       machine.SDA0_PIN,
		SCL:       machine.SCL0_PIN,
	})
	if err != nil {
		log("i2c configure failed", err)
		return err
	}

	return nil
}

func initIMU() error {
	imu = lsm9ds1.New(machine.I2C0)
	err := imu.Configure(lsm9ds1.Configuration{
		AccelRange:      lsm9ds1.ACCEL_2G,
		AccelSampleRate: lsm9ds1.ACCEL_SR_476,
		GyroRange:       lsm9ds1.GYRO_250DPS,
		GyroSampleRate:  lsm9ds1.GYRO_SR_476,
		MagRange:        lsm9ds1.MAG_4G,
		MagSampleRate:   lsm9ds1.MAG_SR_80,
	})
	if err != nil {
		log("imu configure failed", err)
		return err
	}

	return nil
}
