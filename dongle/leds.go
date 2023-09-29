package main

import "machine"

var (
	led   uint8
	red   uint8
	green uint8
	blue  uint8
)

func initLeds() error {
	var err error
	led, err = initLed(machine.LED)
	if err != nil {
		log("init led failed", err)
		return err
	}

	red, err = initLed(machine.LED_RED)
	if err != nil {
		log("init red led failed", err)
		return err
	}

	green, err = initLed(machine.LED_GREEN)
	if err != nil {
		log("init green led failed", err)
		return err
	}

	blue, err = initLed(machine.LED_BLUE)
	if err != nil {
		log("init blue led failed", err)
		return err
	}

	err = machine.PWM0.Configure(machine.PWMConfig{
		Period: 1e6,
	})
	if err != nil {
		log("pwm configure failed", err)
		return err
	}
	machine.PWM0.Set(led, 0)
	machine.PWM0.Set(red, 0)
	machine.PWM0.Set(green, 0)
	machine.PWM0.Set(blue, 0)

	return nil
}

func initLed(led machine.Pin) (uint8, error) {
	ledCh, err := machine.PWM0.Channel(led)
	if err != nil {
		log("get pwm channel failed", err)
		return 0, err
	}
	machine.PWM0.SetInverting(ledCh, true)
	machine.PWM0.Set(ledCh, 0)

	return ledCh, nil
}

func setLed(led uint8, enable bool) {
	if enable {
		machine.PWM0.Set(led, machine.PWM0.Top()*5/255)
	} else {
		machine.PWM0.Set(led, 0)
	}
}
