
# improvements, compared to the osmocom sources


## versions

comparing osmocom's git git://git.osmocom.org/rtl-sdr.git dated from 2020-08-19
against librtlsdr's development branch (same date)


## "Driver" Library Features

* added support for special USB (vendor) VID 0x1209 (product) PID 0x2832: "Generic RTL2832U":
  * A special USB vendor/product id got reserved at http://pid.codes/1209/2832/
  * for such devices the linux kernel's DVB modules are not loaded automatically, thus can be used without blacklisting dvb_usb_rtl28xxu below /etc/modprobe.d/
  * this allows to use a second RTL dongle for use with DVB in parallel
  * the IDs can be programmed with 'rtl_eeprom -n' or 'rtl_eeprom -g realtek_sdr'
  * see https://www.rtl-sdr.com/watching-dvb-t-tv-and-using-sdr-mode-at-the-same-time-with-two-rtl-sdrs/

* added support for using RTLSDR-Dongle from remote - see rtl_rpcd and README.rtlsdr_rpc

* improvements for R820T/2 tuner
  also see https://codingspirit.de/librtlsdr-driver.pdf
  * added better bandwidth support
    * added smaller bandwidths, improving selectivity:
      290, 375, 420, 470, 600, 860, 950, 1100, 1300, 1500, 1600, 1750, 1950 kHz.
      These are coarse measured values .. which might get adjusted in future.
    * bandwidth filters utilize tuner's low- and highpass filters at IF
  * added spectrum flipping (inside tuner) - and back in RTL2832
    * the band edges (low/high-pass) have different steepness;
      the steeper edge can be selected with the mixer sideband (rtlsdr_set_tuner_sideband()),
      to achieve better attenuation depending on signal scenario
  * added (automatic) control over VGA (variable gain amplifier)
    * VGA gain (besides LNA and Mixer) can be utilized and set to automatic,
      letting it controlled from RTL2832U.
      Having all automatic (AGC) including activation of digital AGC in RTL2832
      (rtlsdr_set_agc_mode()), oversteering effects got reduced (a lot).
    * gain range now up to 100 dB
  * deactivated "Filter extension under weak signal" for a stable filter characteristic
  * added shifting of IF-center, to receive away from DC. See rtlsdr_set_tuner_band_center()

* probably some more: it's highly probable, that this list is incomplete


## "Driver" Library API

* added rtlsdr_set_and_get_tuner_bandwidth(), which also delivers the bandwidth.
 [ with rtlsdr_set_tuner_bandwidth() does not deliver the bandwidth ]

* added rtlsdr_set_tuner_band_center(),  to set center of the filtered tuner band

* added rtlsdr_set_tuner_sideband(), to set mixer sideband

* added rtlsdr_set_tuner_gain_ext(), special for R820T/2 tuner

* added rtlsdr_set_tuner_if_mode(), sets AGC modes in detail

* added rtlsdr_set_ds_mode() including threshold frequency

* added rtlsdr_ir_query()

* added rtlsdr_set_opt_string() and rtlsdr_get_opt_help()
 for configuration of 'driver' - especially from command line

* added rtlsdr_set_tuner_i2c_register(), rtlsdr_get_tuner_i2c_register()
 and rtlsdr_set_tuner_i2c_override()
 exposing hacking of tuner-specific I2C registers

* added rtlsdr_get_ver_id(),
 to allow discrimination between osmocom library - or this fork

* added rtlsdr_get_version()


## Added Tools

* added rtl_ir:
 display received IR signals.
  * requires the IR diode of an RTL-SDR - which might not exist!

* added rtl_rpcd:
 a Remote Procedure Call server for RTL-SDR dongles.
  * for use, set environment variable "RTLSDR_RPC_IS_ENABLED"
  * optionally set environment varibales "RTLSDR_RPC_SERV_ADDR"
    and "RTLSDR_RPC_SERV_PORT". These default to "127.0.0.1" and "40000".
  * requires cmake option WITH_RPC

* added rtl_raw2wav:
 save rtl_sdr or rtl_fm's output (pipe) into a wave file,
 including some meta information like timestamp and frequency

* added rtl_udp:
 same as rtl_tcp - just using UDP instead of TCP

* added rtl_wavestat:
 display wave file meta information

* added rtl_wavestream:
 stream raw data (in specified format)


## Improved Tools

* rtl_fm:
  * added command file option '-C', which can trigger actions depending on signal.
    have a look at README.rtlfm_cmdfile.
  * added command line interface option '-E rdc', to enable dc blocking on raw I/Q data at capture rate
  * added CLI option '-E rtlagc', to enable rtl2832's digital agc
  * added CLI option '-E bclo', to use tuner bandwidths low  corner as band center
  * added CLI option '-E bchi', to use tuner bandwidths high corner as band center
  * added CLI option '-O', to set RTL driver options seperated with ':', e.g. -O 'bc=30000:agc=0'
  * added CLI option '-R', to specify number of seconds to run
  * added CLI option '-H', to write wave Header to file, producing a wave file with meta information,
    compatible with several SDR programs
  * added CLI option '-o', to request oversampling (4 recommended) for processing gain

* not just rtl_fm, but many tools have more options.
 compare all the details by starting with command line option '-h'.


## "Driver" Library's UDP-Server - only on Windows

* enabled by cmake option PROVIDE_UDP_SERVER for tests. OFF by default

* activated by rtlsdr_set_opt_string(): "port=1" or "port=<udp_port>",
 default port number: 32323

* purpose is to allow configuration at runtime with a simple text protocol, e.g. with netcat

* for detailed protocol, see comment section in parse() of librtlsdr.c.
 or look for sections with '#ifdef WITH_UDP_SERVER'



## RTL_TCP TCP-PROTOCOL

* allows non-GPL programs, e.g. QIRX, to utilize the RTLSDR stuff in a license compliant way

* added several control functions in rtl_tcp, not existing in osmocom release:
 UDP_ESTABLISH, UDP_TERMINATE, SET_I2C_TUNER_REGISTER, SET_I2C_TUNER_OVERRIDE,
 SET_TUNER_BW_IF_CENTER, SET_TUNER_IF_MODE, SET_SIDEBAND, REPORT_I2C_REGS

* control functions documented in rtl_tcp.h

* (by default) control port number 1234, configurable via command-line-interface (CLI)

* response(s) at +1 of control port: 1235, configurable via CLI

* protocol details in protocol_rtl_tcp.txt

