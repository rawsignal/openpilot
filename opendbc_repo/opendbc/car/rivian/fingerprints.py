""" AUTO-FORMATTED USING opendbc/car/debug/format_fingerprints.py, EDIT STRUCTURE THERE."""
from opendbc.car.structs import CarParams
from opendbc.car.rivian.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.RIVIAN_R1_GEN1: {
    (Ecu.eps, 0x733, None): [
      b'R1TS_v3.4.1(51),3.4.1\x00',
      b'R1TS_v4.4.1(63),4.4.1\x00',
    ],
    # FCM (forward camera module) @ 0x72d (resp 0x76d); UDS F191 = manufacturer ECU hardware number / part ID
    (Ecu.fwdCamera, 0x72d, None): [
      b'\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xffPT00000023-F.0-0000',
    ],
  },
}
