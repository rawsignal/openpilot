""" AUTO-FORMATTED USING opendbc/car/debug/format_fingerprints.py, EDIT STRUCTURE THERE."""
from opendbc.car.structs import CarParams
from opendbc.car.rivian.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.RIVIAN_R1_GEN1: {
    # FCM @ 0x72d (resp 0x76d); UDS F191 = manufacturer ECU hardware number / part ID (camera + ACM harness)
    (Ecu.fwdCamera, 0x72d, None): [
      b'\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xffPT00000023-F.0-0000',
    ],
  },
}
