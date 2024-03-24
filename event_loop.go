package main

import (
	"encoding/binary"
	"fmt"
	"log"
	"os"
	"os/signal"
	"strings"
	"syscall"
	"time"

	"github.com/mattn/go-tty"
)

const (
	ASTATE_Unknown = iota
	ASTATE_Ready
	ASTATE_Armed
	ASTATE_Disarmed
)

const max_stick = 300

// Virtual RC settings
type vRCset struct {
	thr   int // Throttle
	roll  int
	pitch int
	yaw   int
	fs    bool // Failsafe
}

func get_status(v SChan) (status uint64, armflags uint32) {
	if v.cmd == msp2_INAV_STATUS {
		status = binary.LittleEndian.Uint64(v.data[13:21])
	} else {
		status = uint64(binary.LittleEndian.Uint32(v.data[6:10]))
	}

	if v.cmd == msp_STATUS_EX {
		armflags = uint32(binary.LittleEndian.Uint16(v.data[13:15]))
	} else {
		armflags = binary.LittleEndian.Uint32(v.data[9:13])
	}
	return status, armflags
}

func (m *MSPSerial) find_status_cmd() (stscmd uint16) {
	// MSP STatus inquiry, INAV version dependent
	if m.vcapi > 0x200 {
		if m.fcvers >= 0x010801 {
			stscmd = msp2_INAV_STATUS
		} else {
			stscmd = msp_STATUS_EX
		}
	} else {
		stscmd = msp_STATUS
	}
	return stscmd
}

func (m *MSPSerial) main_rx_loop(setthr int, verbose bool, autoarm bool) {
	phase := PHASE_Quiescent
	stscmd := m.find_status_cmd()
	xboxflags := uint64(0)
	xarmflags := uint32(0)
	dpending := false

	vrc := vRCset{ // Virtual RC
		thr: setthr,
		fs:  false,
	}

	tty, err := tty.Open()
	if err != nil {
		log.Fatal(err)
	}
	defer tty.Close()

	evchan := make(chan rune)
	go func() {
		for {
			r, err := tty.ReadRune()
			if err != nil {
				log.Panic(err)
			}
			evchan <- r
		}
	}()

	cc := make(chan os.Signal, 1)
	signal.Notify(cc, os.Interrupt, syscall.SIGINT, syscall.SIGTERM)

	fmt.Println("Keypresses: 'p'/'P': toggle arming, 'L': quit, 'F': quit to failsafe")
	fmt.Println("            '+'/'-' raise / lower throttle by 25µs")
	fmt.Println("            'c'/'C' Center sticks")
	fmt.Println("            'a'<=>'d' Roll")
	fmt.Println("            'w'<=>'s' Pitch")
	fmt.Println("            'q'<=>'e' Yaw")
	log.Printf("Start TX loop")

	ticker := time.NewTicker(100 * time.Millisecond)

	for done := false; !done; {
		select {
		case <-ticker.C:
			tdata := m.serialise_rx(phase, vrc)
			m.Send_msp(msp_SET_RAW_RC, tdata)
			if verbose {
				txdata := deserialise_rx(tdata)
				log.Printf("Tx: %v\n", txdata)
			}
		case v := <-m.c0:
			if v.ok {
				switch v.cmd {
				case msp_SET_RAW_RC:
					if verbose {
						m.Send_msp(msp_RC, nil)
					} else {
						m.Send_msp(stscmd, nil)
					}
				case msp_RC:
					rxdata := deserialise_rx(v.data)
					log.Printf("Rx: %v\n", rxdata)
					m.Send_msp(stscmd, nil)

				case msp2_INAV_STATUS, msp_STATUS_EX, msp_STATUS:
					boxflags, armflags := get_status(v)
					if boxflags != xboxflags || xarmflags != armflags {
						log.Printf("Box: %s (%x) Arm: %s\n", m.format_box(boxflags), boxflags, arm_status(armflags))
						vrc.fs = ((boxflags & m.fail_mask) == m.fail_mask)
						if boxflags&m.arm_mask == 0 { // not armed
							if armflags < 0x80 { // ready to arm
								if autoarm {
									phase = PHASE_Arming
									autoarm = false
								} else {
									phase = PHASE_Quiescent
									done = dpending
								}
							}
						} else { // Armed
							phase = PHASE_LowThrottle
						}
						xboxflags = boxflags
						xarmflags = armflags
					}
				default:
				}
			} else {
				log.Printf("MSP %d (%x) failed\n", v.cmd, v.cmd)
				done = true
			}

		case ev := <-evchan:
			switch ev {
			case 'p', 'P':
				switch phase {
				case PHASE_Quiescent:
					log.Println("Arming commanded")
					phase = PHASE_Arming
				case PHASE_LowThrottle:
					log.Println("Disarming commanded")
					phase = PHASE_Disarming
				default:
				}
			case 'F':
				log.Println("Exit to Fail Safe commanded")
				done = true
			case 'L':
				log.Println("Quit commanded")
				phase, done, dpending = safe_quit(phase)
			case 'v', 'V':
				verbose = !verbose
			case '5':

			case '+', '=':
				vrc.thr += 25
				if vrc.thr > 2000 {
					vrc.thr = 2000
				}
			case '-':
				vrc.thr -= 25
				if vrc.thr < 1000 {
					vrc.thr = 1000
				}
			case 'c', 'C', '`':
				vrc.roll, vrc.pitch, vrc.yaw = 0, 0, 0
				log.Println("Centering the sticks")
			case 'd':
				vrc.roll += 25
				if vrc.roll > max_stick {
					vrc.roll = max_stick
				}
			case 'a':
				vrc.roll -= 25
				if vrc.roll < -max_stick {
					vrc.roll = -max_stick
				}
			case 's':
				vrc.pitch += 25
				if vrc.pitch > max_stick {
					vrc.pitch = max_stick
				}
			case 'w':
				vrc.pitch -= 25
				if vrc.pitch < -max_stick {
					vrc.pitch = -max_stick
				}
			case 'e':
				vrc.yaw += 25
				if vrc.yaw > max_stick {
					vrc.yaw = max_stick
				}
			case 'q':
				vrc.yaw -= 25
				if vrc.yaw < -max_stick {
					vrc.yaw = -max_stick
				}
			}
		case <-cc:
			log.Println("Interrupt")
			phase, done, dpending = safe_quit(phase)
		}
		fmt.Printf("\r")
		fmt.Printf("[R:%d, P:%d, Y:%d, T:%d]",
			vrc.roll, vrc.pitch, vrc.yaw, vrc.thr)
	}
}

func safe_quit(phase int) (int, bool, bool) {
	dpending := false
	done := false
	if phase == PHASE_LowThrottle || phase == PHASE_Disarming {
		dpending = true
		phase = PHASE_Disarming
	} else {
		done = true
	}
	return phase, done, dpending
}

func arm_status(status uint32) string {
	armfails := [...]string{
		"",           /*      1 */
		"",           /*      2 */
		"Armed",      /*      4 */
		"Ever armed", /*      8 */
		"",           /*     10 */ // HITL
		"",           /*     20 */ // SITL
		"",           /*     40 */
		"F/S",        /*     80 */
		"Level",      /*    100 */
		"Calibrate",  /*    200 */
		"Overload",   /*    400 */
		"NavUnsafe", "MagCal", "AccCal", "ArmSwitch", "H/WFail",
		"BoxF/S", "BoxKill", "RCLink", "Throttle", "CLI",
		"CMS", "OSD", "Roll/Pitch", "Autotrim", "OOM",
		"Settings", "PWM Out", "PreArm", "DSHOTBeep", "Land", "Other",
	}

	var sarry []string
	if status < 0x80 {
		if status&(1<<2) != 0 {
			sarry = append(sarry, armfails[2])
		}
		if len(sarry) == 0 {
			sarry = append(sarry, "Ready to arm")
		}
	} else {
		for i := 0; i < len(armfails); i++ {
			if ((status & (1 << i)) != 0) && armfails[i] != "" {
				sarry = append(sarry, armfails[i])
			}
		}
	}
	sarry = append(sarry, fmt.Sprintf("(0x%x)", status))
	return strings.Join(sarry, " ")
}
