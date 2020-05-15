package main

import (
	"flag"
	"fmt"
	"log"
	"os"
	"regexp"
	"strings"
	"strconv"
)

const (
	DevClass_NONE = iota
	DevClass_SERIAL
	DevClass_TCP
	DevClass_UDP
)

type DevDescription struct {
	klass  int
	name   string
	param  int
	name1  string
	param1 int
}

var (
	baud   = flag.Int("b", 115200, "Baud rate")
	device = flag.String("d", "", "Serial Device")
	arm    = flag.Bool("a", false, "Arm (take care now) [with iNav versions supporting stick arming]")
	sarm   = flag.Int("A", 0, "Arm Switch, (5-8), assumes 2000us will arm")
	usev2  = flag.Bool("2", false, "Use MSPv2")
	cmap   = flag.String("m", "AERT", "channel map")
)

func check_device() DevDescription {
	devdesc := parse_device()
	if devdesc.name == "" {
		for _, v := range []string{"/dev/ttyACM0", "/dev/ttyUSB0"} {
			if _, err := os.Stat(v); err == nil {
				devdesc.klass = DevClass_SERIAL
				devdesc.name = v
				devdesc.param = *baud
				break
			}
		}
	}
	if devdesc.name == "" && devdesc.param == 0 {
		log.Fatalln("No device given\n")
	} else {
		log.Printf("Using device %s\n", devdesc.name)
	}
	return devdesc
}

func parse_device() DevDescription {
	dd := DevDescription{klass: DevClass_NONE}
	r := regexp.MustCompile(`^(tcp|udp)://([\[\]:A-Za-z\-\.0-9]*):(\d+)/{0,1}([A-Za-z\-\.0-9]*):{0,1}(\d*)`)
	m := r.FindAllStringSubmatch(*device, -1)
	if len(m) > 0 {
		if m[0][1] == "tcp" {
			dd.klass = DevClass_TCP
		} else {
			dd.klass = DevClass_UDP
		}
		dd.name = m[0][2]
		dd.param, _ = strconv.Atoi(m[0][3])
		// These are only used for ESP8266 UDP
		dd.name1 = m[0][4]
		dd.param1, _ = strconv.Atoi(m[0][5])
	} else {
		ss := strings.Split(*device, "@")
		dd.klass = DevClass_SERIAL
		dd.name = ss[0]
		if len(ss) > 1 {
			dd.param, _ = strconv.Atoi(ss[1])
		} else {
			dd.param = *baud
		}
	}
	return dd
}

func main() {
	flag.Usage = func() {
		fmt.Fprintf(os.Stderr, "Usage of msp_set_rx [options]\n")
		flag.PrintDefaults()
	}
	flag.Parse()

	fmt.Printf("Map is %s\n", *cmap)
	devdesc := check_device()
	s := MSPInit(devdesc, *usev2)
	s.set_map(*cmap)
	s.test_rx(*arm, *sarm)
}
