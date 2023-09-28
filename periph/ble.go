package main

import (
	"time"

	"github.com/google/uuid"
	"tinygo.org/x/bluetooth"
)

func initBLE() error {
	err := bluetooth.DefaultAdapter.Enable()
	if err != nil {
		log("ble enable failed", err)
		return err
	}

	return nil
}

func connect() (*bluetooth.Device, error) {
	ch := make(chan bluetooth.ScanResult, 1)
	err := bluetooth.DefaultAdapter.Scan(func(adapter *bluetooth.Adapter, result bluetooth.ScanResult) {
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
		return nil, err
	}

	var dev *bluetooth.Device
	select {
	case result := <-ch:
		dev, err = bluetooth.DefaultAdapter.Connect(result.Address, bluetooth.ConnectionParams{
			ConnectionTimeout: bluetooth.NewDuration(4 * time.Second),
			MinInterval:       bluetooth.NewDuration(7500 * time.Microsecond),
			MaxInterval:       bluetooth.NewDuration(7500 * time.Microsecond),
		})
		if err != nil {
			log("ble connect failed", err)
			return nil, err
		}
	}

	return dev, nil
}

func getCharacteristic(dev *bluetooth.Device) (bluetooth.DeviceCharacteristic, error) {
	svcs, err := dev.DiscoverServices([]bluetooth.UUID{bluetooth.NewUUID(uuid.MustParse("e43e5547-afad-4309-81a4-1453c8fde090"))})
	if err != nil || len(svcs) == 0 {
		log("service discovery failed", err)
		return bluetooth.DeviceCharacteristic{}, err
	}

	chars, err := svcs[0].DiscoverCharacteristics([]bluetooth.UUID{
		bluetooth.NewUUID(uuid.MustParse("464e8563-de1d-4ff2-8e32-7761ce0620b6")),
	})
	if err != nil || len(chars) == 0 {
		log("characteristic discovery failed", err)
		return bluetooth.DeviceCharacteristic{}, err
	}

	return chars[0], nil
}
