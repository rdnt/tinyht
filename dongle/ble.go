package main

import (
	"github.com/google/uuid"
	"tinygo.org/x/bluetooth"
)

var serviceUUID = bluetooth.NewUUID(uuid.MustParse("e43e5547-afad-4309-81a4-1453c8fde090"))
var rotUUID = bluetooth.NewUUID(uuid.MustParse("464e8563-de1d-4ff2-8e32-7761ce0620b6"))

func initBLE(onUpdate func([]byte)) error {
	err := bluetooth.DefaultAdapter.Enable()
	if err != nil {
		log("ble enable failed", err)
		return err
	}

	adv := bluetooth.DefaultAdapter.DefaultAdvertisement()
	err = adv.Configure(bluetooth.AdvertisementOptions{
		LocalName:    "EDHT",
		ServiceUUIDs: []bluetooth.UUID{serviceUUID},
	})
	if err != nil {
		log("adv configure failed", err)
		return err
	}

	err = bluetooth.DefaultAdapter.AddService(&bluetooth.Service{
		UUID: serviceUUID,
		Characteristics: []bluetooth.CharacteristicConfig{
			{
				Handle: &bluetooth.Characteristic{},
				UUID:   rotUUID,
				Flags:  bluetooth.CharacteristicWritePermission | bluetooth.CharacteristicWriteWithoutResponsePermission,
				WriteEvent: func(client bluetooth.Connection, offset int, buf []byte) {
					if offset != 0 || len(buf) != 12 {
						return
					}

					onUpdate(buf)
				},
			},
		},
	})
	if err != nil {
		log("add service failed", err)
		return err
	}

	err = adv.Start()
	if err != nil {
		log("start adv failed", err)
		return err
	}

	return err
}
