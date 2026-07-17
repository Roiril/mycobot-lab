"""SO-101: サーボの電圧リミット(EEPROM)と実電圧を突き合わせる診断/修復ツール。

「[RxPacketError] Input voltage error!」で接続できない時の一次切り分け。

このエラーは *電源が異常* な時だけでなく、**サーボ EEPROM の Max_Voltage /
Min_Voltage の設定を実電圧が外れている**時にも出る。後者は電源を挿し直しても
消えない（設定側の問題なので）。実際 2026-07-17 に、フォロワーの id3/4/5 だけ
Max_Voltage=12.0V のまま実測 12.2V となり、恒久的にエラーを吐いて接続不能に
なった。他の id1/2/6 は工場出荷値の 14.0V で無事だった。

    python scripts/so101_check_voltage_limits.py --port COM13
    python scripts/so101_check_voltage_limits.py --port COM13 --set-max 14.0

lerobot 非依存（生プロトコル）なので、どの env からでも実行できる。
"""
from __future__ import annotations

import argparse
import sys
import time

import serial

BAUD = 1_000_000

ADDR_MAX_VOLTAGE = 14      # EEPROM, 0.1V 単位
ADDR_MIN_VOLTAGE = 15      # EEPROM, 0.1V 単位
ADDR_LOCK = 55             # 0=EEPROM書込可 / 1=ロック
ADDR_PRESENT_VOLTAGE = 62  # RAM, 0.1V 単位

ERR_VOLTAGE = 0x01         # ステータスのエラービット: 電圧


def _txn(ser: serial.Serial, sid: int, instr: int, params: bytes = b"") -> bytes:
    body = bytes([sid, 2 + len(params), instr]) + params
    chk = ~sum(body) & 0xFF
    ser.reset_input_buffer()
    ser.write(b"\xFF\xFF" + body + bytes([chk]))
    time.sleep(0.015)
    return ser.read(16)


def read_reg(ser: serial.Serial, sid: int, addr: int):
    """(err_bits, value) を返す。無応答なら (None, None)。"""
    r = _txn(ser, sid, 2, bytes([addr, 1]))
    if len(r) < 6:
        return None, None
    return r[4], r[5]


def write_reg(ser: serial.Serial, sid: int, addr: int, value: int) -> None:
    _txn(ser, sid, 3, bytes([addr, value]))
    time.sleep(0.02)


def set_max_voltage(ser: serial.Serial, sid: int, volts: float) -> float | None:
    """EEPROM のロックを外して Max_Voltage を書き、再ロックして読み戻す。"""
    write_reg(ser, sid, ADDR_LOCK, 0)
    write_reg(ser, sid, ADDR_MAX_VOLTAGE, int(round(volts * 10)))
    time.sleep(0.03)
    write_reg(ser, sid, ADDR_LOCK, 1)
    _, mx = read_reg(ser, sid, ADDR_MAX_VOLTAGE)
    return mx / 10 if mx is not None else None


def survey(ser: serial.Serial, ids: list[int]) -> list[dict]:
    rows = []
    for sid in ids:
        _, mx = read_reg(ser, sid, ADDR_MAX_VOLTAGE)
        _, mn = read_reg(ser, sid, ADDR_MIN_VOLTAGE)
        err, pv = read_reg(ser, sid, ADDR_PRESENT_VOLTAGE)
        rows.append({
            "id": sid,
            "err": err,
            "max_v": mx / 10 if mx is not None else None,
            "min_v": mn / 10 if mn is not None else None,
            "present_v": pv / 10 if pv is not None else None,
        })
    return rows


def diagnose(row: dict) -> str:
    if row["err"] is None:
        return "無応答（配線・ID・ボーレートを確認）"
    if not row["err"] & ERR_VOLTAGE:
        return "OK"
    pv, mx, mn = row["present_v"], row["max_v"], row["min_v"]
    if pv is None or mx is None or mn is None:
        return "電圧エラー（リミット読み取り不可）"
    if pv > mx:
        return f"電圧エラー: 実測 {pv:.1f}V > Max {mx:.1f}V ← リミットが狭すぎる"
    if pv < mn:
        return f"電圧エラー: 実測 {pv:.1f}V < Min {mn:.1f}V ← 給電不足を疑う"
    return f"電圧エラー（実測 {pv:.1f}V はリミット内。電源の瞬断履歴を疑う）"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="COM13", help="フォロワーの COM ポート (既定 COM13)")
    ap.add_argument("--ids", default="1,2,3,4,5,6", help="調べるサーボ ID (既定 1..6)")
    ap.add_argument("--set-max", type=float, metavar="VOLTS",
                    help="実測が Max を超えているサーボの Max_Voltage をこの値に書き換える"
                         "（STS3215 の工場出荷値は 14.0）")
    args = ap.parse_args()

    ids = [int(x) for x in args.ids.split(",") if x.strip()]
    try:
        ser = serial.Serial(args.port, BAUD, timeout=0.15)
    except serial.SerialException as e:
        print(f"{args.port} を開けません: {e}")
        print("（コックピット等がポートを掴んでいる場合は先に止めてください）")
        return 2

    with ser:
        rows = survey(ser, ids)
        print(f"--- {args.port} ---")
        print(" id |  err | Max_V | Min_V | 実測  | 判定")
        for r in rows:
            err = "  -- " if r["err"] is None else f" 0x{r['err']:02X}"
            fmt = lambda v: f"{v:5.1f}" if v is not None else "    ?"
            print(f"  {r['id']} |{err} | {fmt(r['max_v'])} | {fmt(r['min_v'])} |"
                  f" {fmt(r['present_v'])} | {diagnose(r)}")

        over = [r for r in rows
                if r["err"] and r["err"] & ERR_VOLTAGE
                and r["present_v"] is not None and r["max_v"] is not None
                and r["present_v"] > r["max_v"]]

        if not args.set_max:
            if over:
                print(f"\n{len(over)} 個が Max_Voltage 超過。他のサーボと同じ値に揃えるなら:")
                print(f"  python {sys.argv[0]} --port {args.port} --set-max 14.0")
            return 0

        if not over:
            print("\nMax_Voltage 超過のサーボは無し。書き換えません。")
            return 0

        print(f"\nMax_Voltage を {args.set_max:.1f}V に書き換えます: "
              f"id {', '.join(str(r['id']) for r in over)}")
        for r in over:
            got = set_max_voltage(ser, r["id"], args.set_max)
            print(f"  id{r['id']}: Max_Voltage -> {got:.1f}V" if got is not None
                  else f"  id{r['id']}: 読み戻し失敗")

        time.sleep(0.3)
        print("\n--- 書込後 ---")
        for r in survey(ser, ids):
            err = "  -- " if r["err"] is None else f" 0x{r['err']:02X}"
            print(f"  id{r['id']}: err={err.strip()}  {diagnose(r)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
