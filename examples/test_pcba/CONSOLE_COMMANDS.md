# test_pcba Console Commands

This document is compiled from `examples/test_pcba/main/main.c` and the actual command registration source code. It covers the commands directly available in the test_pcba example console, including parameter descriptions and usage examples.

## Notes

- The console prompt is `cmd>`.
- Enter `help` first to list the commands currently registered on the console.
- Some commands run continuously; you usually need to press `Ctrl+C` to exit.
- A few parameters in the `wifi-cmd` component are affected by the IDF version or SoC capabilities; this document annotates them where applicable.
- The main list below includes only commands that are unconditionally registered in the example source code, or that can be confirmed as available under this project's configuration.

## Command Overview

| Category | Commands |
| --- | --- |
| Built-in | `help` |
| Wi-Fi / Network | `wifi`, `wifi_count`, `wifi_mode`, `wifi_protocol`, `wifi_bandwidth`, `wifi_ps`, `wifi_country`, `ap_set`, `ap`, `ap_query`, `sta_connect`, `sta`, `sta_disconnect`, `disconnect`, `sta_scan`, `wifi_scan`, `scan`, `wifi_txpower`, `ping` |
| LoRa / BLE / GNSS | `lora_cw`, `lora_tx`, `lora_fcc_fhss`, `lora_rx`, `gfsk_tx`, `lr_fhss_tx`, `lora_fcc_fhss_2`, `gfsk_fcc_fhss_3`, `subg_tx_flrc`, `subg_rx_flrc`, `ble`, `gnss`, `gnss_flash` |
| GPIO / I2C / IO Expander | `gpio_set`, `gpio_get`, `gpio_config`, `i2c_scan`, `io_exp_set`, `io_exp_get`, `io_exp_config`, `io_exp_print` |
| Sensors / Battery | `sensor_id`, `sensor_time_get`, `sensor_time_set`, `sht4x_serial`, `sht4x_read`, `bmm350_id`, `bmm350_read`, `bmm350_reset`, `battery_get`, `lsm6dsox_id`, `lsm6dsox_read`, `spa06_id`, `spa06_read` |
| Audio | `speaker_play`, `speaker_tone`, `microphone_read`, `audio_loopback` |
| LCD / SD | `lcd_test`, `lcd_fb_demo`, `lcd_ui_demo`, `lcd_line_demo`, `sd_rw_test`, `lcd_fps_test` |
| Power Control | `power_gnss_off`, `power_sd_off`, `power_lcd_off`, `power_sen_audio_off` |

## Built-in Commands

### help

- Function: Display the list of commands currently registered on the console along with their help text.
- Parameters: none.
- Example:

```bash
help
```

## Wi-Fi / Network Commands

### wifi

- Function: Basic Wi-Fi operations.
- Syntax: `wifi <action> [--espnow_enc <int>] [--storage <flash|ram>]`
- Parameters:
  - `<action>`: Required. Valid values are `init`, `deinit`, `start`, `stop`, `restart`, `status`.
  - `--espnow_enc <int>`: Optional. Used only with `init` and `restart`.
  - `--storage <flash|ram>`: Optional. Sets the Wi-Fi storage location during initialization.
- Example:

```bash
wifi init --storage ram
wifi start
wifi status
wifi restart --espnow_enc 6
```

### wifi_count

- Function: Query or clear the Wi-Fi counters.
- Syntax: `wifi_count [query|clear]`
- Parameters:
  - `<action>`: Optional, defaults to `query`.
- Example:

```bash
wifi_count
wifi_count clear
```

### wifi_mode

- Function: Set the Wi-Fi operating mode.
- Syntax: `wifi_mode <ap|sta|apsta>`
- Parameters:
  - `<mode>`: Required. One of `ap`, `sta`, `apsta`.
- Example:

```bash
wifi_mode sta
wifi_mode apsta
```

### wifi_protocol

- Function: Set or query the Wi-Fi protocol.
- Syntax:
  - `wifi_protocol`
  - `wifi_protocol <protocol> [-i <ap|sta>]`
  - `wifi_protocol --2g <proto> [--5g <proto>] [-i <ap|sta>]`
- Parameters:
  - `<protocol>`: Optional. Base interface protocol configuration, e.g. `b`, `b/g`, `b/g/n`.
  - `--2g <proto>`: Optional. 2.4 GHz protocol configuration; accepts `lr`, `b`, `g`, `n`, `ax` and combinations such as `lr/b`.
  - `--5g <proto>`: Optional. 5 GHz protocol configuration; only meaningful on supported interfaces.
  - `-i, --interface <ap|sta>`: Optional, defaults to `sta`.
- Note: Without any arguments, it queries the current protocol configuration.
- Example:

```bash
wifi_protocol
wifi_protocol b/g/n -i sta
wifi_protocol --2g b/g/n --5g ax -i ap
```

### wifi_bandwidth

- Function: Set or query the Wi-Fi bandwidth.
- Syntax:
  - `wifi_bandwidth`
  - `wifi_bandwidth <20|40> [-i <ap|sta>]`
  - `wifi_bandwidth --2g <20|40> [--5g <20|40>] [-i <ap|sta>]`
- Parameters:
  - `<cbw>`: Optional. Base bandwidth setting, `20` or `40`.
  - `--2g <20|40>`: Optional. 2.4 GHz bandwidth.
  - `--5g <20|40>`: Optional. 5 GHz bandwidth; only meaningful on supported interfaces.
  - `-i, --interface <ap|sta>`: Optional, defaults to `sta`.
- Note: Without any arguments, it queries the current bandwidth.
- Example:

```bash
wifi_bandwidth
wifi_bandwidth 20 -i sta
wifi_bandwidth --2g 40 -i ap
```

### wifi_ps

- Function: Set the Wi-Fi power-save mode.
- Syntax: `wifi_ps <type>`
- Parameters:
  - `<type>`: Required. `0` = `WIFI_PS_NONE`, `1` = `WIFI_PS_MIN_MODEM`, `2` = `WIFI_PS_MAX_MODEM`.
- Example:

```bash
wifi_ps 0
wifi_ps 2
```

### wifi_country

- Function: Query or set the country code and channel range.
- Syntax:
  - `wifi_country`
  - `wifi_country <code> [-s <schan>] [-n <nchan>] [-p <auto|manual>]`
- Parameters:
  - `<code>`: Optional. Country code, e.g. `CN`, `US`.
  - `-s, --schan <int>`: Optional. Starting channel.
  - `-n, --nchan <int>`: Optional. Number of channels.
  - `-p, --policy <auto|manual>`: Optional, defaults to `auto`.
  - `--5g-chan-mask <int>`, `--5g-chan <list>`: Available only in 5 GHz-capable configurations.
- Note: Without arguments, it queries the current country configuration.
- Example:

```bash
wifi_country
wifi_country CN
wifi_country US -s 1 -n 11 -p manual
```

### ap_set

- Function: Configure the SoftAP.
- Syntax: `ap_set <ssid> [pass] [-a <authmode>] [-n <channel>] [-m <max_conn>]`
- Parameters:
  - `<ssid>`: Required. AP name.
  - `<pass>`: Optional. AP password.
  - `-a, --authmode <authmode>`: Optional, e.g. `open`, `wep`, `wpa2`, `wpa2_enterprise`.
  - `-n, --channel <channel>`: Optional. AP channel.
  - `-m, --max_conn <max_conn>`: Optional. Maximum number of connected stations, defaults to `2`.
- Example:

```bash
ap_set MeshPager_AP 12345678 -a wpa2 -n 6 -m 4
ap_set MeshPager_AP
```

### ap

- Function: Compatibility alias for `ap_set`.
- Parameters: Same as `ap_set`.
- Example:

```bash
ap MeshPager_AP 12345678 -n 1
```

### ap_query

- Function: Query the current AP configuration and status.
- Parameters: none.
- Example:

```bash
ap_query
```

### sta_connect

- Function: Connect to a specified AP in STA mode.
- Syntax: `sta_connect <ssid> [pass] [options]`
- Parameters:
  - `<ssid>`: Required.
  - `<pass>`: Optional.
  - `-b, --bssid <bssid>`: Optional. Specify the BSSID.
  - `-n, --channel <channel>`: Optional. Specify the channel.
  - `--no-disconnect`: Optional. Do not actively disconnect before connecting.
  - `--no-reconnect`: Optional. Disable auto-reconnect.
  - `--full-scan`: Optional. Perform a full-channel scan before connecting.
  - `--5g-offset <int>`: Optional. Present only under specific IDF versions/configurations.
  - `--failure_retry <int>`: Optional. Present only under specific IDF versions.
- Example:

```bash
sta_connect MySSID MyPass
sta_connect MySSID MyPass -n 6 --full-scan
sta_connect MySSID -b aa:bb:cc:dd:ee:ff --no-reconnect
```

### sta

- Function: Compatibility alias for `sta_connect`.
- Parameters: Same as `sta_connect`.
- Example:

```bash
sta MySSID MyPass
```

### sta_disconnect

- Function: Disconnect the current STA link or stop reconnecting.
- Parameters: none.
- Example:

```bash
sta_disconnect
```

### disconnect

- Function: Compatibility alias for `sta_disconnect`.
- Parameters: none.
- Example:

```bash
disconnect
```

### sta_scan

- Function: Scan nearby APs.
- Syntax: `sta_scan [options]`
- Parameters:
  - `<ssid>`: Optional. Filter by SSID.
  - `-b, --bssid <bssid>`: Optional. Filter by BSSID.
  - `-n, --channel <int>`: Optional. Filter by channel.
  - `-h, --show-hidden`: Optional. Show hidden networks.
  - `--max <int>`: Optional. Maximum active-scan duration.
  - `--min <int>`: Optional. Minimum active-scan duration.
  - `--passive`: Optional. Use passive scanning.
  - `--passive-time <int>`: Optional. Passive-scan duration.
  - `--dwell <int>`: Optional. Present only under specific IDF versions; home-channel dwell time.
  - `-2, --bitmap-2g <int/hex>`: Optional. Present only under specific IDF versions.
  - `-5, --bitmap-5g <int/hex>`: Optional. Present only under specific IDF versions.
  - `--count-only`: Optional. Count APs only.
- Example:

```bash
sta_scan
sta_scan --show-hidden
sta_scan MySSID -n 6
sta_scan --passive-time 120 --count-only
```

### wifi_scan

- Function: Compatibility alias for `sta_scan`.
- Parameters: Same as `sta_scan`.
- Example:

```bash
wifi_scan --show-hidden
```

### scan

- Function: Compatibility alias for `sta_scan`.
- Parameters: Same as `sta_scan`.
- Example:

```bash
scan
```

### wifi_txpower

- Function: Set or read the maximum Wi-Fi transmit power.
- Syntax:
  - `wifi_txpower`
  - `wifi_txpower -u <unit>`
  - `wifi_txpower --reset`
- Parameters:
  - `-u, --unit <unit>`: Optional. Transmit power unit, each unit is `0.25 dBm`, e.g. `8 = 2 dBm`.
  - `--reset`: Optional. Restore the sdkconfig default value.
- Note: Without arguments, it reads the current power.
- Example:

```bash
wifi_txpower
wifi_txpower -u 68
wifi_txpower --reset
```

### ping

- Function: Send ICMP ECHO requests to test network connectivity.
- Syntax: `ping <host> [options]`
- Parameters:
  - `<host>`: Required. Domain name or IP address.
  - `-W, --timeout <timeout>`: Optional. Time to wait for a response.
  - `-i, --interval <interval>`: Optional. Send interval, in seconds.
  - `-s, --packetsize <size>`: Optional. Number of data bytes.
  - `-c, --count <count>`: Optional. Number of sends, defaults to `5`.
  - `-Q, --tos <n>`: Optional. Set the ToS.
  - `--abort`: Optional. Abort the current ping.
- Example:

```bash
ping 192.168.1.1
ping www.example.com -c 4 -i 1 -W 2
ping 192.168.1.1 --abort
```

## LoRa / BLE / GNSS Commands

### lora_cw

- Function: Start a LoRa continuous-wave (CW) transmit test.
- Syntax: `lora_cw [-f <freq>] [-p <power>] [-o <ocp>]`
- Parameters:
  - `-f, --freq <f>`: Optional. Frequency, range `415000000 ~ 940000000`, default `868000000`.
  - `-p, --power`: Optional. Transmit power, default `10`.
  - `-o, --ocp`: Optional. Over-current protection level, range `0 ~ 63`, default unset; the log steps in `2.5 mA` increments.
- Example:

```bash
lora_cw
lora_cw -f 868000000 -p 14 -o 24
```

### lora_tx

- Function: LoRa packet transmit test, supporting a fixed packet count or continuous transmission.
- Syntax: `lora_tx [options]`
- Parameters:
  - `-f, --freq <f>`: Frequency, range `150000000 ~ 2500000000` Hz, default `868000000`.
  - `-s, --sf <5~12>`: Spreading factor, range `5~12`; out of range is reset to `7`, default `7`.
  - `-b, --bw <0~9>`: Bandwidth index, `0=31kHz`, `1=41kHz`, `2=62kHz`, `3=125kHz`, `4=200kHz`, `5=250kHz`, `6=400kHz`, `7=500kHz`, `8=800kHz`, `9=1000kHz`; values above `9` are clamped to `9`, default `3` (125 kHz).
  - `-c, --cr <1~7>`: Coding rate, `1=CR_4_5`, `2=CR_4_6`, `3=CR_4_7`, `4=CR_4_8`, `5=CR_LI_4_5`, `6=CR_LI_4_6`, `7=CR_LI_4_8`, default `1`.
  - `-p, --power`: Transmit power. LF band range `-10 ~ +22 dB`, HF band range `-17 ~ +12 dB`, default `10`.
  - `--crc <0|1>`: CRC switch, `0=DISABLE`, `1=ENABLE`, default `1`.
  - `--iq <0|1>`: IQ mode, `0=STANDARD`, `1=INVERTED`, default `0`.
  - `--net <0|1>`: Network type, `0=Private Network(0x12)`, `1=Public Network(0x34)`, default `1`.
  - `-i, --interval <t>`: Send interval, in milliseconds, default `0`.
  - `-d, --txt <d>`: Transmit text, default `hello`.
  - `-n, --num <n>`: Number of packets to send, `0` means continuous. The help text says the default is `1`, but the code's actual default is `10`.
  - `-hp, --half_power`: PA half power, 0.5 dB steps. LF range `-19 ~ 44`, HF range `-39 ~ 24` (see datasheet Table 7-15/16/17/18/20), default `0`.
  - `-dc, --duty_cycle`: PA duty cycle, range `0 ~ 31`, default `0`.
  - `-lf, --lf_slices`: PA lf_slices, range `0 ~ 15`, default `0`.
- Note: When any of `-hp` / `-dc` / `-lf` is passed, `usr_lr20xx_set_tx_cfg` is called for custom PA configuration. During continuous transmission, press `Ctrl+C` to exit.
- Example:

```bash
lora_tx
lora_tx -f 868000000 -s 7 -b 3 -c 1 -p 14 -d hello -n 10
lora_tx -d test_packet -i 1000 -n 0
```

### lora_fcc_fhss

- Function: LoRa FCC FHSS transmit test.
- Syntax: `lora_fcc_fhss [options]`
- Parameters:
  - `-m, --mode <0|1>`: FHSS mode, `0=FHSS_125K_MODE`, `1=FHSS_500K_MODE`, default `0`.
  - `-s, --sf <6~12>`: Default `10`.
  - `-c, --cr <1|2|3|4>`: Default `1`.
  - `-p, --power`: Default `14`.
  - `--crc <0|1>`: Default `1`.
  - `--iq <0|1>`: Default `0`.
  - `--net <0|1>`: Default `0`.
  - `-i, --interval <t>`: Send interval in ms, default `0`.
  - `-d, --txt <d>`: Transmit text, default `hello seeed! 1234567`.
- Note: The command automatically generates a hopping table and transmits through the list; press `Ctrl+C` to exit early.
- Example:

```bash
lora_fcc_fhss
lora_fcc_fhss -m 0 -s 10 -p 14 -d fhss_test
```

### lora_rx

- Function: LoRa receive test.
- Syntax: `lora_rx [options]`
- Parameters:
  - `-f, --freq <f>`: Frequency, range `150000000 ~ 2500000000` Hz, default `868000000`.
  - `-s, --sf <5~12>`: Spreading factor, range `5~12`; out of range is clamped to the boundary, default `7`.
  - `-b, --bw <0~9>`: Bandwidth index, `0=31kHz`, `1=41kHz`, `2=62kHz`, `3=125kHz`, `4=200kHz`, `5=250kHz`, `6=400kHz`, `7=500kHz`, `8=800kHz`, `9=1000kHz`; values above `9` are clamped to `9`, default `3` (125 kHz).
  - `-c, --cr <1|2|3|4>`: Coding rate, `1=CR_4_5`, `2=CR_4_6`, `3=CR_4_7`, `4=CR_4_8`, default `1`.
  - `--crc <0|1>`: CRC switch, `0=DISABLE`, `1=ENABLE`, default `1`.
  - `--iq <0|1>`: IQ mode, `0=STANDARD`, `1=INVERTED`, default `0`.
  - `--net <0|1>`: Network type, `0=Private Network(0x12)`, `1=Public Network(0x34)`, default `1`.
  - `--boosted <0|1>`: `1=Boosted RX`, `0=Normal RX`, default `1`. The parameter is registered but is not actually used by the current implementation.
- Note: The command receives continuously; press `Ctrl+C` to exit and print receive statistics.
- Example:

```bash
lora_rx
lora_rx -f 868000000 -s 7 -b 3 -c 1 --crc 1
```

### gfsk_tx

- Function: GFSK packet transmit test, supporting a fixed packet count or continuous transmission.
- Syntax: `gfsk_tx [options]`
- Parameters:
  - `-f, --freq <f>`: Frequency, default `868000000`.
  - `-p, --power`: Transmit power, default `10`.
  - `--br <bps>`: GFSK bit rate, default `50000`.
  - `--fdev <hz>`: Frequency deviation, default `25000`.
  - `--bw <hz>`: Dual-sided bandwidth, default `100000`.
  - `-i, --interval <t>`: Send interval, in milliseconds, default `0`.
  - `-d, --txt <d>`: Transmit text, default `hello`.
  - `-n, --num <n>`: Number of packets to send, `0` means continuous, default `1`.
- Note: During continuous transmission, press `Ctrl+C` to exit.
- Example:

```bash
gfsk_tx
gfsk_tx -f 868000000 -p 14 --br 50000 --fdev 25000 --bw 100000 -d gfsk_test -n 10
gfsk_tx -d test_packet -i 1000 -n 0
```

### lr_fhss_tx

- Function: LR-FHSS packet transmit test, transmitting along a random hop sequence.
- Syntax: `lr_fhss_tx [options]`
- Parameters:
  - `-f, --freq <f>`: Center frequency, default `868000000`.
  - `-b, --bw <0-9>`: Bandwidth index. `0=39063`, `1=85938`, `2=136719`, `3=183594`, `4=335938`, `5=386719`, `6=722656`, `7=773438`, `8=1523438`, `9=1574219`, default `9`.
  - `-c, --cr <0-3>`: Coding rate. `0=CR_5_6`, `1=CR_2_3`, `2=CR_1_2`, `3=CR_1_3`, default `0`.
  - `-g, --grid <0|1>`: Frequency grid. `0=25391_HZ`, `1=3906_HZ`, default `0`.
  - `-h, --hop <0|1>`: Whether hopping is enabled, default `1`.
  - `-p, --power`: Transmit power, default `10`.
  - `-i, --interval <t>`: Send interval, in milliseconds, default `0`.
  - `-d, --txt <d>`: Transmit text, default `hello`.
  - `-n, --num <n>`: Number of packets to send, `0` means continuous, default `1`.
- Note: During continuous transmission, press `Ctrl+C` to exit.
- Example:

```bash
lr_fhss_tx
lr_fhss_tx -f 868000000 -b 9 -c 0 -g 0 -h 1 -d lr_fhss_test -n 5
```

### lora_fcc_fhss_2

- Function: LoRa FCC FHSS test (500 kHz bandwidth channel-sequence version).
- Syntax: `lora_fcc_fhss_2 [options]`
- Parameters:
  - `-s, --sf <6~12>`: Default `10`.
  - `-c, --cr <1|2|3|4>`: Default `1`.
  - `-p, --power`: Default `14`.
  - `--crc <0|1>`: Default `1`.
  - `--iq <0|1>`: Default `0`.
  - `--net <0|1>`: Default `0`.
  - `-i, --interval <t>`: Send interval in ms, default `0`.
  - `-d, --txt <d>`: Transmit text, default `hello seeed! 1234567`.
- Note: The command transmits along a predefined FCC channel sequence; press `Ctrl+C` to exit early.
- Example:

```bash
lora_fcc_fhss_2
lora_fcc_fhss_2 -s 10 -p 14 -d fhss2_test
```

### gfsk_fcc_fhss_3

- Function: GFSK FCC FHSS test (100 kHz step, 129-channel random sequence).
- Syntax: `gfsk_fcc_fhss_3 [options]`
- Parameters:
  - `-f, --freq <f>`: Reference frequency, default `868000000` (the test switches to an FCC channel sequence).
  - `-p, --power`: Transmit power, default `10`.
  - `--br <bps>`: GFSK bit rate, default `50000`.
  - `--fdev <hz>`: Frequency deviation, default `25000`.
  - `--bw <hz>`: Dual-sided bandwidth, default `100000`.
  - `-i, --interval <t>`: Send interval in ms, default `0`.
  - `-d, --txt <d>`: Transmit text, default `hello`.
  - `-n, --num <n>`: The parameter is registered, but the current implementation instead loops over the internally generated number of FCC channels.
- Note: The command automatically generates and iterates through the channel sequence; press `Ctrl+C` to exit early.
- Example:

```bash
gfsk_fcc_fhss_3
gfsk_fcc_fhss_3 -p 14 --br 50000 --fdev 25000 --bw 100000 -d gfsk_fhss_test
```

### subg_tx_flrc

- Function: FLRC (Fast LoRa, fast long-range communication) modulation transmit test. Sends a fixed number of packets filled with incrementing bytes (`0,1,2,...,len-1`), with the send interval fixed at `1000 ms`.
- Syntax: `subg_tx_flrc [options]`
- Parameters:
  - `-f, --freq <f>`: Frequency, range `415000000 ~ 940000000`, default `915000000`.
  - `-p, --power`: Transmit power. LPA range `-17 ~ +14 dB`, HPA range `-9 ~ +22 dB`, default `22`.
  - `-b, --br_bw <0-7>`: Bit-rate/bandwidth combination index, default `0`; values above `7` are clamped to `7`.
    - `0=BR_2_600_BW_2_666` (2.6 Mbps / 2.666 MHz)
    - `1=BR_2_080_BW_2_222`
    - `2=BR_1_300_BW_1_333`
    - `3=BR_1_040_BW_1_333`
    - `4=BR_0_650_BW_0_740`
    - `5=BR_0_520_BW_0_571`
    - `6=BR_0_325_BW_0_357`
    - `7=BR_0_260_BW_0_307`
  - `-l, --len <n>`: Single-packet data length (bytes), default `200`; values above `255` are clamped to `255`.
  - `-c, --cnt <n>`: Number of packets to send, default `100`; values above `200` are clamped to `200`.
- Note: During transmission, press `Ctrl+C` to exit early.
- Example:

```bash
subg_tx_flrc
subg_tx_flrc -f 915000000 -p 22 -b 0 -l 200 -c 100
```

### subg_rx_flrc

- Function: FLRC receive test, receives continuously and counts, printing receive results on exit.
- Syntax: `subg_rx_flrc [options]`
- Parameters:
  - `-f, --freq <f>`: Frequency, range `415000000 ~ 940000000`, default `915000000` (the help text says `915000000`, but the current implementation actually uses `915000000`).
  - `-b, --br_bw <0-7>`: Bit-rate/bandwidth combination index, default `0`; values above `7` are clamped to `7`. The meanings are the same as the `-b` of `subg_tx_flrc`.
- Note: The command receives continuously; press `Ctrl+C` to exit and print the three counters `rx_done`, `rx_hdr_err`, and `rx_crc_err`.
- Example:

```bash
subg_rx_flrc
subg_rx_flrc -f 915000000 -b 0
```

### ble

- Function: BLE advertising test.
- Syntax: `ble [-a <0|1>]`
- Parameters:
  - `-a, --adv <0|1>`: `0` stops advertising, `1` starts advertising.
- Note: In the code, the start or stop action is performed only when `-a` is explicitly passed.
- Example:

```bash
ble -a 1
ble -a 0
```

### gnss

- Function: GNSS UART data read or parse test.
- Syntax: `gnss [-m <0|1>] [-b <baud>]`
- Parameters:
  - `-m, --mode <0|1>`: `0` raw NMEA output, `1` parsed output, default `0`.
  - `-b, --baud <baud>`: UART baud rate, e.g. `115200`, `921600`, default `115200`. The UART driver is reinstalled at the requested baud each run (the previous run's driver is torn down by `gnss` on exit), so two invocations with different bauds switch rates cleanly.
- Note: The command runs continuously; press `Ctrl+C` to exit.
- Example:

```bash
gnss
gnss -m 1
gnss -b 921600
gnss -m 1 -b 921600
gnss --baud 460800
```

### gnss_flash

- Function: Switch the GNSS to the firmware-download ready state.
- Parameters: none.
- Example:

```bash
gnss_flash
```

## GPIO / I2C / IO Expander Commands

### gpio_set

- Function: Set a GPIO level.
- Syntax: `gpio_set -g <gpio> -l <0|1>`
- Parameters:
  - `-g, --gpio <n>`: GPIO number.
  - `-l, --level <0|1>`: Output level.
- Example:

```bash
gpio_set -g 4 -l 1
gpio_set -g 4 -l 0
```

### gpio_get

- Function: Read a GPIO level.
- Syntax: `gpio_get -g <gpio>`
- Example:

```bash
gpio_get -g 4
```

### gpio_config

- Function: Configure a GPIO mode and pull resistor.
- Syntax: `gpio_config -g <gpio> -m <0|1|2> [-p <0|1|2|3>]`
- Parameters:
  - `-m, --mode <0|1|2>`: `0=input`, `1=output`, `2=output_od`.
  - `-p, --pull <0|1|2|3>`: `0=floating`, `1=pullup`, `2=pulldown`, `3=both`, default `0`.
- Example:

```bash
gpio_config -g 4 -m 1
gpio_config -g 5 -m 0 -p 1
```

### i2c_scan

- Function: Scan for I2C devices.
- Syntax: `i2c_scan [-b <0|1>]`
- Parameters:
  - `-b, --bus <0|1>`: Optional. If omitted, both I2C0 and I2C1 are scanned.
- Example:

```bash
i2c_scan
i2c_scan -b 0
```

### io_exp_set

- Function: Set an IO expander pin level.
- Syntax: `io_exp_set -i <io> -l <0|1>`
- Parameters:
  - `-i, --io <n>`: IO number, range `0-23`.
  - `-l, --level <0|1>`: Output level.
- Example:

```bash
io_exp_set -i 3 -l 1
```

### io_exp_get

- Function: Read an IO expander pin level.
- Syntax: `io_exp_get -i <io>`
- Example:

```bash
io_exp_get -i 3
```

### io_exp_config

- Function: Configure an IO expander pin direction.
- Syntax: `io_exp_config -i <io> -m <0|1>`
- Parameters:
  - `-m, --mode <0|1>`: `0=input`, `1=output`.
- Example:

```bash
io_exp_config -i 3 -m 1
io_exp_config -i 4 -m 0
```

### io_exp_print

- Function: Print the IO expander status.
- Parameters: none.
- Example:

```bash
io_exp_print
```

## Sensor / Battery Commands

### sensor_id

- Function: Read the YSN8900E device ID.
- Parameters: none.
- Example:

```bash
sensor_id
```

### sensor_time_get

- Function: Read the YSN8900E time.
- Parameters: none.
- Example:

```bash
sensor_time_get
```

### sensor_time_set

- Function: Set the YSN8900E time.
- Syntax: `sensor_time_set -y <year> -M <month> -d <day> -w <week> -H <hour> -m <minute> -s <second>`
- Parameters:
  - `-y, --year <2000-2099>`
  - `-M, --month <1-12>`
  - `-d, --day <1-31>`
  - `-w, --week <1-7>`
  - `-H, --hour <0-23>`
  - `-m, --minute <0-59>`
  - `-s, --second <0-59>`
- Example:

```bash
sensor_time_set -y 2026 -M 5 -d 21 -w 4 -H 14 -m 30 -s 0
```

### sht4x_serial

- Function: Read the SHT4X serial number.
- Parameters: none.
- Example:

```bash
sht4x_serial
```

### sht4x_read

- Function: Read SHT4X temperature and humidity.
- Parameters: none.
- Example:

```bash
sht4x_read
```

### bmm350_id

- Function: Read the BMM350 chip ID.
- Parameters: none.
- Example:

```bash
bmm350_id
```

### bmm350_read

- Function: Read the BMM350 magnetic field and temperature.
- Parameters: none.
- Example:

```bash
bmm350_read
```

### bmm350_reset

- Function: Perform a software reset on the BMM350.
- Parameters: none.
- Note: A soft reset restores all registers to their default values (suspend mode, default ODR, all axes disabled). Therefore, after reset this command automatically reconfigures the power mode / ODR / enables each axis; otherwise subsequent `bmm350_id` / `bmm350_read` calls will fail to read.
- Example:

```bash
bmm350_reset
```

### battery_get

- Function: Read the battery voltage and estimate the state of charge percentage.
- Parameters: none.
- Example:

```bash
battery_get
```

### lsm6dsox_id

- Function: Read the LSM6DSOX (6-axis IMU) device ID, used to verify that sensor communication is working.
- Parameters: none.
- Note: The LSM6DSOX shares the I2C_1 bus with the BMM350 and SHT4X. The first call automatically probes addresses `0x6A` / `0x6B` on that bus (corresponding to the two SA0 wiring options); after finding a device with ID `0x6C`, initialization completes. The normal return value is `0x6C`.
- Example:

```bash
lsm6dsox_id
```

### spa06_id

- Function: Read the SPA06-003 pressure and temperature sensor chip ID.
- Parameters: none.
- Note: The sensor is initialized automatically on the shared I2C_1 bus. The expected chip ID is `0x11`.
- Example:

```bash
spa06_id
```

### spa06_read

- Function: Read the SPA06-003 pressure and temperature.
- Parameters: none.
- Output fields: pressure in `hPa`, temperature in `°C`.
- Note: The sensor is initialized automatically on the shared I2C_1 bus.
- Example:

```bash
spa06_read
```

## Audio Commands

### speaker_play

- Function: Play an audio file from the SD card.
- Syntax: `speaker_play [-p <path>] [-v <volume>]`
- Parameters:
  - `-p, --path </sdcard/file.mp3>`: Optional. Audio path; supports wav/mp3. Defaults to `/sdcard/test.mp3`.
  - `-v, --volume <0-93>`: Optional. Speaker volume. The help text says `0-93`, but the actual code validates a range of `0-100`; it defaults to the current volume.
- Example:

```bash
speaker_play
speaker_play -p /sdcard/test.mp3 -v 60
```

### speaker_tone

- Function: Compatibility alias for `speaker_play`.
- Parameters: Same as `speaker_play`.
- Example:

```bash
speaker_tone -p /sdcard/test.wav
```

### microphone_read

- Function: Record microphone audio to a WAV file on the SD card. The microphone signal is processed through the AFE path (NS + AGC + AEC) into clean mono PCM, rather than raw interleaved ADC samples.
- Syntax: `microphone_read [-o <output>] [-d <duration>] [-g <linear_gain>] [-m <mic_gain>]`
- Parameters:
  - `-o, --output </sdcard/file.wav>`: Optional, defaults to `/sdcard/mic_record.wav`.
  - `-d, --duration <100-60000>`: Optional. Recording duration, in milliseconds, default `10000`.
  - `-g, --linear_gain <1.0-10.0>`: Optional. AFE linear gain (corresponds to `afe_config->afe_linear_gain`), range `1.0~10.0`, default `1.0`. Out-of-range values are rejected.
  - `-m, --mic_gain <20.0-37.5>`: Optional. Microphone PGA gain, in dB, range `20.0~37.5`, default `37.5` (chip maximum). Must be set after opening the recording codec; out-of-range values are rejected.
- Example:

```bash
microphone_read
microphone_read -o /sdcard/mic.wav -d 5000
microphone_read -g 5.0 -m 30.0
```

### audio_loopback

- Function: Play `/sdcard/1.wav` from the SD card using the ES8311 while simultaneously recording 5 seconds of audio with the ES7343E, saving both the raw microphone content and the AFE-processed content separately.
- Syntax: `audio_loopback [-v <0-60>] [-g <linear_gain>]`
- Parameters:
  - `-v, --volume <0-60>`: Optional. Speaker volume during loopback, default `28`. To avoid strong feedback between the onboard microphone and speaker, the loopback default volume is lower than normal playback volume.
  - `-g, --linear_gain <1.0-10.0>`: Optional. AFE linear gain (corresponds to `afe_config->afe_linear_gain`), range `1.0~10.0`, default `1.0`. Out-of-range values are rejected.
- Output:
  - Raw microphone mono content is saved to `/sdcard/1_raw.wav`
  - AFE-processed mono content is saved to `/sdcard/1_afe.wav`
- Note: During the test, the recorded content is not played back to the speaker; only the input file `/sdcard/1.wav` is played.
- Example:

```bash
audio_loopback
audio_loopback -v 20
audio_loopback -v 20 -g 5.0
```

## LCD / SD Commands

### lcd_test

- Function: Display a basic LCD test pattern.
- Syntax: `lcd_test -m <solid|bars|checker> [-c <color>] [-b <brightness>]`
- Parameters:
  - `-m, --mode <solid|bars|checker>`: Required.
  - `-c, --color <name>`: Optional. Valid only in `solid` mode. Supports `red`, `green`, `blue`, `white`, `black`, `yellow`, `cyan`, `magenta`, `gray`, `orange`, etc.
  - `-b, --brightness <1-99>`: Optional. Sets the screen backlight brightness percentage; the actual value is clamped to the `1-99` range (the lower/upper limits of `lcd_bl_set`). If omitted, the current brightness is kept.
- Example:

```bash
lcd_test -m bars
lcd_test -m checker
lcd_test -m solid -c red
lcd_test -m solid -c white -b 50
lcd_test -m bars -b 99
```

### lcd_fb_demo

- Function: Display the framebuffer demo screen.
- Syntax: `lcd_fb_demo -m <gradient|ui|clear>`
- Example:

```bash
lcd_fb_demo -m gradient
lcd_fb_demo -m ui
lcd_fb_demo -m clear
```

### lcd_ui_demo

- Function: Display the UI dashboard example. The dashboard reads live battery
  voltage and charger state, then shows battery percentage, voltage, and the
  charger status (`CHARGING` / `FULL` / `NO USB`).
- Battery percentage mapping: `>= 4.4 V` → 100 %, `<= 3.3 V` → 0 %, linear in
  between.
- Charger status: `BSP_CHARGER_IN` high → USB connected; then `BSP_CHARGER_STAT`
  low → `CHARGING`, high → `FULL`. `BSP_CHARGER_IN` low → `NO USB`.
- Syntax:
  - `lcd_ui_demo -a <start|stop|status>`
  - `lcd_ui_demo -m <dashboard|status>` (legacy one-shot display)
- Note: `-a start` draws the dashboard immediately and then refreshes the battery voltage, percentage, and charger state every 5 seconds. Use `-a stop` to stop the periodic refresh; `-a status` reports whether it is running. Repeating `-a start` does not create a duplicate task.
- Note: `dashboard` and `status` with `-m` retain the original one-shot behavior.
- Example:

```bash
lcd_ui_demo -a start
lcd_ui_demo -a status
lcd_ui_demo -a stop
lcd_ui_demo -m dashboard
```

### lcd_line_demo

- Function: Demonstrate partial refresh using a small line buffer.
- Syntax: `lcd_line_demo -m <gradient|bands>`
- Example:

```bash
lcd_line_demo -m gradient
lcd_line_demo -m bands
```

### sd_rw_test

- Function: Test SD card read/write throughput.
- Syntax: `sd_rw_test -m <write|read|rw> [-c <chunk-kb>] [-t <total-kb>]`
- Parameters:
  - `-m, --mode <write|read|rw>`: Required.
  - `-c, --chunk-kb <kb>`: Optional. Chunk size, default `64`.
  - `-t, --total-kb <kb>`: Optional. Total test size, default `8192`.
- Example:

```bash
sd_rw_test -m rw
sd_rw_test -m write -c 128 -t 4096
sd_rw_test -m read -c 64 -t 2048
```

### lcd_fps_test

- Function: Start, stop, or query the LCD FPS test task. Test frames are read from `/sdcard/rgb565/` (raw RGB565 bare frames only, 240×320, 153600 bytes/frame).
- Syntax: `lcd_fps_test -a <start|stop|status> [-m <sd|psram>] [-i <interval-ms>]`
- Parameters:
  - `-a, --action <start|stop|status>`: Required.
  - `-m, --mode <sd|psram>`: Optional. Effective on `start`, default `sd`.
  - `-i, --interval-ms <ms>`: Optional. Frame-switch interval, default `0`.
- Example:

```bash
lcd_fps_test -a start -m sd
lcd_fps_test -a start -m psram -i 50
lcd_fps_test -a status
lcd_fps_test -a stop
```

## Power Control Commands

Each command is responsible for completely shutting down one peripheral domain: stop the driver → cut the power rail → reset the corresponding data/control pins to a low-leakage state (input, no pull, no interrupts). Commands can be executed repeatedly (when already off, only a warning is printed, no crash). To restore, restart the device.

### power_gnss_off

- Function: Turn off the GNSS power. Stops NMEA scanning and releases UART1, cuts `PWR_EN`, `VRTC_EN`, and `RST`, and sets the `RX/TX/PPS0` pins to a low-leakage state.
- Parameters: none.
- Example:

```bash
power_gnss_off
```

### power_sd_off

- Function: Turn off the SD card power. If mounted, it first unmounts the driver (`bsp_sdcard_unmount`), resets the `D0/CLK/CMD` pins, then cuts the `SD_PWR_EN` power rail.
- Parameters: none.
- Example:

```bash
power_sd_off
```

### power_lcd_off

- Function: Turn off the LCD power. Turns off the backlight, calls `bsp_display_deinit` to release the panel/SPI resources, resets the `CS/SDA/SCK/A0/PWM` pins, then cuts `LCD_PWR_EN` and `LCD_RST`.
- Parameters: none.
- Example:

```bash
power_lcd_off
```

### power_sen_audio_off

- Function: Turn off the sensors and audio (the two share the `SEN_EN` power rail and the I2C_1 bus, so they are combined into one command). Stops the player/codec, cuts `PA_PWR_EN` and `SEN_EN`, releases the I2C_1 bus, and resets the I2S (`MCLK/SCLK/LRLK/SDIN/SDOUT`) and I2C_1 (`SCL/SDA`) pins.
- Parameters: none.
- Example:

```bash
power_sen_audio_off
```

## Notes

- `ap`, `sta`, `disconnect`, `wifi_scan`, `scan`, and `speaker_tone` are compatibility/historical aliases; prefer the new primary command names.
- Commands such as `lora_tx`, `lora_rx`, `lora_fcc_fhss`, `subg_tx_flrc`, `subg_rx_flrc`, and `gnss` are typically long-running tests; make sure your serial terminal can send `Ctrl+C`.
- `lcd_fps_test -a start` reads raw RGB565 bare frames from `/sdcard/rgb565/` (suffixes `.rgb565`/`.raw`/`.bin`/`.rgb`/`.565`, each frame 240×320×2 = 153600 bytes); the `psram` mode depends on a successful preload. The images under `/sdcard/images/` are for the image-display commands only and are independent of the FPS test.
- `speaker_play` and `microphone_read` require the SD card filesystem to be mounted.
- The `wifi-cmd` component also contains a few conditionally-compiled commands such as `wifi_band`, `wifi_stats`, `itwt`, and HE debug commands; because they depend on specific SoCs or configuration switches, they are not included in the main list of this document.
