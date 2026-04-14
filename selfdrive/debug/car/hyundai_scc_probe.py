#!/usr/bin/env python3
import argparse
import time

from opendbc.can import CANPacker
from opendbc.car.crc import CRC8J1850, mk_crc8_fun
from opendbc.car.structs import CarParams
from panda import Panda


hyundai_checksum = mk_crc8_fun(CRC8J1850, init_crc=0xFD, xor_out=0xDF)


def create_scc11(packer: CANPacker, idx: int, enabled: bool, set_speed: int) -> tuple[int, bytes, int]:
  values = {
    "MainMode_ACC": int(enabled),
    "TauGapSet": 4,
    "VSetDis": set_speed if enabled else 0,
    "AliveCounterACC": idx % 0x10,
    "ObjValid": int(enabled),
    "ACC_ObjStatus": int(enabled),
    "ACC_ObjLatPos": 0,
    "ACC_ObjRelSpd": 0,
    "ACC_ObjDist": 1 if enabled else 0,
  }
  return packer.make_can_msg("SCC11", 0, values)


def create_scc12(packer: CANPacker, idx: int, enabled: bool, accel: float) -> tuple[int, bytes, int]:
  values = {
    "ACCMode": int(enabled),
    "aReqRaw": accel + 3.0,
    "aReqValue": accel + 3.0,
    "CR_VSM_Alive": idx % 0xF,
  }
  dat = packer.make_can_msg("SCC12", 0, values)[1]
  values["CR_VSM_ChkSum"] = 0x10 - sum(sum(divmod(b, 16)) for b in dat) % 0x10
  return packer.make_can_msg("SCC12", 0, values)


def create_scc13(packer: CANPacker) -> tuple[int, bytes, int]:
  values = {
    "SCCDrvModeRValue": 2,
    "SCC_Equip": 1,
    "AebDrvSetStatus": 0,
  }
  return packer.make_can_msg("SCC13", 0, values)


def create_scc14(packer: CANPacker, enabled: bool) -> tuple[int, bytes, int]:
  values = {
    "ACCMode": int(enabled),
    "JerkUpperLimit": 3.2 if enabled else 0.0,
    "JerkLowerLimit": 0.1 if enabled else 0.0,
    "SCCMode2": 1 if enabled else 0,
    "ComfortBandUpper": 0.24 if enabled else 0.0,
    "ComfortBandLower": 0.24 if enabled else 0.0,
  }
  return packer.make_can_msg("SCC14", 0, values)


def create_fca12(packer: CANPacker) -> tuple[int, bytes, int]:
  values = {
    "FCA_DrvSetState": 2,
    "FCA_USM": 1,
  }
  return packer.make_can_msg("FCA12", 0, values)


def create_frt_radar11(packer: CANPacker) -> tuple[int, bytes, int]:
  values = {
    "CF_FCA_Equip_Front_Radar": 1,
  }
  return packer.make_can_msg("FRT_RADAR11", 0, values)


def main() -> None:
  parser = argparse.ArgumentParser(description="Send Hyundai SCC probe messages directly via Panda.")
  parser.add_argument("--bus", type=int, default=0, help="CAN bus to transmit on")
  parser.add_argument("--duration", type=float, default=30.0, help="Seconds to run")
  parser.add_argument("--set-speed", type=int, default=60, help="SCC11 VSetDis value")
  parser.add_argument("--accel", type=float, default=0.0, help="Requested accel baseline for SCC12")
  parser.add_argument("--disabled", action="store_true", help="Send disabled-state SCC11/12/14 values")
  parser.add_argument("--skip-fca12", action="store_true", help="Do not send FCA12")
  args = parser.parse_args()

  enabled = not args.disabled
  packer = CANPacker("hyundai_kia_generic")

  print("Connecting to Panda...")
  panda = Panda()
  panda.set_safety_mode(CarParams.SafetyModel.allOutput)

  print("Starting Hyundai SCC probe")
  print(f"bus={args.bus} duration={args.duration}s enabled={enabled} set_speed={args.set_speed} accel={args.accel}")
  print("Sending: SCC11/12/14 @ 50Hz, SCC13/FCA12 @ 5Hz, FRT_RADAR11 @ 2Hz")
  print("Ctrl-C to stop")

  start_t = time.monotonic()
  next_50hz = start_t
  next_5hz = start_t
  next_2hz = start_t
  idx = 0

  try:
    while True:
      now = time.monotonic()
      if now - start_t >= args.duration:
        break

      to_send: list[list[int | bytes]] = []

      if now >= next_50hz:
        to_send.extend([
          list(create_scc11(packer, idx, enabled, args.set_speed)),
          list(create_scc12(packer, idx, enabled, args.accel)),
          list(create_scc14(packer, enabled)),
        ])
        idx += 1
        next_50hz += 0.02

      if now >= next_5hz:
        to_send.append(list(create_scc13(packer)))
        if not args.skip_fca12:
          to_send.append(list(create_fca12(packer)))
        next_5hz += 0.2

      if now >= next_2hz:
        to_send.append(list(create_frt_radar11(packer)))
        next_2hz += 0.5

      if to_send:
        for msg in to_send:
          msg[2] = args.bus
        panda.can_send_many(to_send, timeout=0)

      time.sleep(0.001)
  except KeyboardInterrupt:
    pass

  print("Done.")


if __name__ == "__main__":
  main()
