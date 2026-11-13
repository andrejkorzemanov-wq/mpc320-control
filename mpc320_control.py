#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mpc320_control.py — управление поляризационным контроллером Thorlabs MPC320.

MPC320 — это 3 моторизованных лепестка (аналог трёх вращающихся
фазовых пластин: лямбда/4, лямбда/2, лямбда/4). Каждый лепесток
поворачивается в диапазоне 0–170°. Прибор подключается по USB
и общается по протоколу Thorlabs APT, поэтому для работы скрипта
НЕ требуется установленный Kinesis — нужны только Python и драйвер USB.

ВАЖНО: во время работы скрипта закройте Kinesis / любое другое ПО,
которое держит прибор открытым (порт открывается в эксклюзивном режиме).

Установка зависимостей:
    pip install pnpq pyserial

Примеры использования:
    python mpc320_control.py list                  # найти прибор, показать его серийный номер
    python mpc320_control.py status                # прочитать текущие углы лепестков
    python mpc320_control.py set 45 90 30          # задать углы лепестков 1, 2, 3 (градусы)
    python mpc320_control.py set 45 90 30 --serial 38408784
    python mpc320_control.py home                  # выставить все лепестки в home-позицию
    python mpc320_control.py home --channel 2      # home только лепестка 2
    python mpc320_control.py identify              # мигнуть LED на приборе
    python mpc320_control.py scan 2 0 170 5        # сканировать лепесток 2: от 0° до 170° с шагом 5°

Эмулятор (проверка логики БЕЗ физического прибора):
    python mpc320_control.py --simulate status
    python mpc320_control.py --simulate set 45 90 30
    python mpc320_control.py --simulate scan 2 0 170 5
Эмулятор ведёт себя как настоящий MPC320, но движения происходят
"в компьютере". Реальную связь с железом это не проверяет.

Использование как модуля:
    from pnpq.apt.connection import AptConnection
    from pnpq.apt.protocol import ChanIdent
    from pnpq.devices.polarization_controller_thorlabs_mpc import (
        PolarizationControllerThorlabsMPC320,
    )
    from pnpq.units import pnpq_ureg

    with AptConnection(serial_number="38408784") as conn:
        mpc = PolarizationControllerThorlabsMPC320(connection=conn)
        mpc.move_absolute(ChanIdent.CHANNEL_1, 45 * pnpq_ureg.degree)
        mpc.move_absolute(ChanIdent.CHANNEL_2, 90 * pnpq_ureg.degree)
        mpc.move_absolute(ChanIdent.CHANNEL_3, 30 * pnpq_ureg.degree)
"""

from __future__ import annotations

import argparse
import sys
import time

import serial.tools.list_ports
from pnpq.apt.connection import AptConnection
from pnpq.apt.protocol import ChanIdent
from pnpq.devices.polarization_controller_thorlabs_mpc import (
    PolarizationControllerThorlabsMPC320,
)
from pnpq.units import pnpq_ureg

try:
    from pnpq.devices.polarization_controller_thorlabs_mpc_stub import (
        PolarizationControllerThorlabsMPC320Stub,
    )
    STUB_AVAILABLE = True
except ImportError:  # старые версии pnpq без эмулятора
    STUB_AVAILABLE = False

CHANNELS = (ChanIdent.CHANNEL_1, ChanIdent.CHANNEL_2, ChanIdent.CHANNEL_3)

# Диапазон поворота лепестка по документации Thorlabs (0–170°,
# минимальный шаг ~0.12°). Всё покрытие сферы Пуанкаре — внутри этого диапазона.
ANGLE_MIN = 0.0
ANGLE_MAX = 170.0


def list_devices() -> None:
    """Печатает подключённые USB-COM порты. Ищите строку с Thorlabs."""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("USB-COM порты не найдены. Проверьте подключение прибора.")
        return
    for p in ports:
        print(f"{p.device:<8} SN={p.serial_number}  {p.description}")


def autodetect_serial() -> str:
    """Находит серийный номер прибора Thorlabs автоматически."""
    for p in serial.tools.list_ports.comports():
        text = f"{p.description} {p.manufacturer or ''}".upper()
        if "THORLABS" in text and p.serial_number:
            print(f"Найден прибор: {p.description}, SN={p.serial_number}")
            return str(p.serial_number)
    raise SystemExit(
        "Прибор Thorlabs не найден. Запустите 'python mpc320_control.py list',\n"
        "найдите серийный номер и передайте его через --serial."
    )


def check_angle(angle: float) -> float:
    if not (ANGLE_MIN <= angle <= ANGLE_MAX):
        raise ValueError(f"Угол {angle}° вне диапазона {ANGLE_MIN}–{ANGLE_MAX}°")
    return angle


def set_angles(mpc: PolarizationControllerThorlabsMPC320,
               a1: float, a2: float, a3: float) -> None:
    """Повернуть лепестки 1, 2, 3 в заданные углы (градусы)."""
    for idx, (chan, ang) in enumerate(zip(CHANNELS, (a1, a2, a3)), start=1):
        check_angle(ang)
        print(f"Лепесток {idx}: движение к {ang:.2f}° ...")
        mpc.move_absolute(chan, ang * pnpq_ureg.degree)
    print("Все лепестки на месте.")


def get_angles(mpc: PolarizationControllerThorlabsMPC320) -> list[float]:
    """Возвращает текущие углы лепестков 1–3 в градусах."""
    angles = []
    for chan in CHANNELS:
        status = mpc.get_status(chan)
        deg = (status.position * pnpq_ureg.mpc320_step).to("degree").magnitude
        angles.append(deg)
    return angles


def print_status(mpc: PolarizationControllerThorlabsMPC320) -> None:
    for idx, (chan, deg) in enumerate(zip(CHANNELS, get_angles(mpc)), start=1):
        print(f"Лепесток {idx}: {deg:.2f}°")


def scan(mpc: PolarizationControllerThorlabsMPC320, channel: int,
         start: float, stop: float, step: float, delay: float) -> None:
    """Сканирует угол одного лепестка и печатает каждую точку.

    delay — пауза после каждого шага, секунды (полезно, если вы
    в это время что-то измеряете вручную или из другого скрипта).
    """
    chan = CHANNELS[channel - 1]
    check_angle(start)
    check_angle(stop)
    if step <= 0:
        raise ValueError("Шаг должен быть положительным")
    if start > stop:
        step = -step
    angle = start
    while (angle <= stop + 1e-9) if step > 0 else (angle >= stop - 1e-9):
        mpc.move_absolute(chan, angle * pnpq_ureg.degree)
        print(f"Лепесток {channel}: {angle:.2f}°")
        time.sleep(delay)
        angle += step


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Управление поляризационным контроллером Thorlabs MPC320")
    parser.add_argument("--serial", help="серийный номер прибора (если не задан — автоопределение)")
    parser.add_argument("--simulate", action="store_true",
                        help="работа с эмулятором вместо реального прибора")
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("list", help="найти подключённые приборы и их серийные номера")
    sub.add_parser("status", help="показать текущие углы лепестков")
    sub.add_parser("identify", help="мигнуть LED на приборе (проверка связи)")

    p_set = sub.add_parser("set", help="задать углы лепестков 1, 2, 3 (градусы)")
    p_set.add_argument("angles", nargs=3, type=float, metavar=("A1", "A2", "A3"))

    p_home = sub.add_parser("home", help="выставить лепестки в home-позицию")
    p_home.add_argument("--channel", type=int, choices=(1, 2, 3),
                        help="home только одного лепестка; без флага — всех")

    p_scan = sub.add_parser("scan", help="сканирование угла лепестка")
    p_scan.add_argument("channel", type=int, choices=(1, 2, 3))
    p_scan.add_argument("start", type=float, help="начальный угол, градусы")
    p_scan.add_argument("stop", type=float, help="конечный угол, градусы")
    p_scan.add_argument("step", type=float, help="шаг, градусы")
    p_scan.add_argument("--delay", type=float, default=0.0,
                        help="пауза после каждого шага, секунды")

    args = parser.parse_args()

    if args.cmd == "list":
        list_devices()
        return

    if args.simulate:
        if not STUB_AVAILABLE:
            raise SystemExit("Эмулятор недоступен: обновите библиотеку (pip install -U pnpq)")
        # Эмулятор: вся логика скрипта работает без физического прибора.
        # time_scaling_factor=0.2 имитирует время движения лепестков
        # (в 5 раз быстрее реальности); 0.0 — мгновенные движения.
        mpc = PolarizationControllerThorlabsMPC320Stub(time_scaling_factor=0.2)
        run_command(args, mpc)
        return

    serial_number = args.serial or autodetect_serial()

    with AptConnection(serial_number=serial_number) as conn:
        mpc = PolarizationControllerThorlabsMPC320(connection=conn)
        run_command(args, mpc)


def run_command(args, mpc) -> None:
    if args.cmd == "status":
        print_status(mpc)
    elif args.cmd == "identify":
        mpc.identify(ChanIdent.CHANNEL_1)
        print("LED на приборе мигает.")
    elif args.cmd == "set":
        set_angles(mpc, *args.angles)
    elif args.cmd == "home":
        # По мануалу home перед работой не обязателен:
        # прибор запоминает позиции лепестков при включении.
        if args.channel:
            print(f"Homing лепестка {args.channel} ...")
            mpc.home(CHANNELS[args.channel - 1])
        else:
            print("Homing всех лепестков ...")
            for chan in CHANNELS:
                mpc.home(chan)
        print("Готово.")
    elif args.cmd == "scan":
        scan(mpc, args.channel, args.start, args.stop, args.step, args.delay)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nОстановлено пользователем.")
        sys.exit(130)
    except ValueError as e:
        print(f"Ошибка: {e}")
        sys.exit(1)
