# ESP8266 Xilinx Virtual Cable

ESP8266 implementation of XVC (Xilinx Virtual Cable) protocol based on xvcd (https://github.com/tmbinc/xvcd)

## Pinout

| ESP8266  | JTAG |
|----------|------|
| D4 | TDO |
| D5 | TMS |
| D6 | TDI |
| D7 | TCK |

## Usage

Start Impact and use xilinx_xvc cable plugin:

```
xilinx_xvc host=192.168.0.1:2542 maxpacketsize=512 disableversioncheck=true
```

## Reference

Xilix Virtual Cable protocol reference:
https://github.com/Xilinx/XilinxVirtualCable
