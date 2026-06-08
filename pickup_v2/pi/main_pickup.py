"""pickup_v2 主程序 — 任意位置草莓抓取的总调度。

数据流（详见 docs/设计方案.md 第 3.1 节）：
    CSI 摄像头 → YOLO → bbox → coord_transform → kinematics.ik → ProtocolV2

CLI：
    python -m main_pickup                  # 默认：连摄像头 + YOLO + 串口
    python -m main_pickup --dry-run        # 不发串口，只打印命令（PC 调试用）
    python -m main_pickup --once           # 只抓一个目标就退出
    python -m main_pickup --simulate-uv 320 240 0  # 跳过相机/YOLO，模拟像素点 + class_id
    python -m main_pickup --model models/strawberry_yolov8n.pt --port /dev/serial0

注意：
- 配置全部从 config_v2 取（不 import vision/pi/config）。
- 标定文件未就绪时 → 提示先跑阶段 A（intrinsic_calib + homography）。
- 任何 NACK SAFETY / 超时 → 立即 X 急停 + 退出，由用户排查后重启。
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from dataclasses import dataclass

import config_v2 as cfg
from coord_transform import Calibration, bbox_to_armbase, load_calibration
from kinematics import IKUnreachableError, angles_to_servo_degs, ik
from protocol_v2 import CommandResult, ProtocolV2

logger = logging.getLogger("pickup_v2.main")

# class_id → PLACE 标签（与 vision/pi/config.py 的 CLASS_MAP 保持一致）
CLASS_TO_PLACE = {0: "A", 1: "B", 2: "C"}
CLASS_NAMES = {0: "ripe", 1: "semi_ripe", 2: "unripe"}

# 抓取时序参数
PRE_GRASP_LIFT_MM = 30.0  # 目标上方多少 mm 作为预抓取点
POST_GRASP_LIFT_MM = 50.0  # 闭爪后抬起的高度
DETECT_INTERVAL_S = 0.3  # 检测循环间隔


# ============================================================
# 视觉：bbox + class_id（不复用 vision/pi/detector，避免它的触发区过滤）
# ============================================================


@dataclass(frozen=True)
class TargetBBox:
    class_id: int
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def xyxy(self) -> tuple[float, float, float, float]:
        return self.x1, self.y1, self.x2, self.y2


class _Camera:
    """CSI 摄像头薄封装。保留 BGR 通道翻转（OV5647 已知问题）。"""

    def __init__(self, width: int = 640, height: int = 480) -> None:
        from picamera2 import Picamera2

        self.cam = Picamera2()
        self.cam.configure(
            self.cam.create_preview_configuration(
                main={"size": (width, height), "format": "BGR888"}
            )
        )
        self.cam.start()
        time.sleep(1.0)
        logger.info("CSI 摄像头已启动: %dx%d", width, height)

    def read(self):
        frame = self.cam.capture_array()
        return frame[:, :, ::-1].copy()  # 通道翻转 → 真实 BGR

    def close(self) -> None:
        self.cam.stop()


class _Detector:
    """YOLO 检测，返回最高置信度的草莓 bbox。"""

    def __init__(self, model_path: str, conf: float = 0.5, iou: float = 0.45) -> None:
        from ultralytics import YOLO

        self.model = YOLO(model_path)
        self.conf = conf
        self.iou = iou
        logger.info("YOLO 模型已加载: %s (conf=%.2f)", model_path, conf)

    def detect_best(self, frame) -> TargetBBox | None:
        results = self.model.predict(frame, conf=self.conf, iou=self.iou, verbose=False)
        if not results or results[0].boxes is None or len(results[0].boxes) == 0:
            return None
        boxes = results[0].boxes
        best_i = int(boxes.conf.argmax().item())
        cls_id = int(boxes.cls[best_i].item())
        if cls_id not in CLASS_TO_PLACE:
            return None
        x1, y1, x2, y2 = boxes.xyxy[best_i].tolist()
        return TargetBBox(
            class_id=cls_id,
            confidence=float(boxes.conf[best_i].item()),
            x1=x1,
            y1=y1,
            x2=x2,
            y2=y2,
        )


# ============================================================
# 抓取单元
# ============================================================


def _expect_ok(result: CommandResult, action: str) -> None:
    """收到非 ok 响应直接抛 RuntimeError —— 由上层决定急停或重试。"""
    if not result.ok:
        raise RuntimeError(f"{action} 失败: {result.response!r}")


def execute_pickup(
    proto: ProtocolV2 | None,
    xyz: tuple[float, float, float],
    place_label: str,
) -> None:
    """完整抓取-放置时序，对应协议规范.md 第 6 节。

    proto=None 时仅打印命令（dry-run）。
    """
    x, y, z = xyz
    pre_z = z + PRE_GRASP_LIFT_MM
    lift_z = z + POST_GRASP_LIFT_MM

    def _do(name: str, fn) -> None:
        if proto is None:
            logger.info("[DRY-RUN] %s", name)
            return
        result = fn()
        _expect_ok(result, name)

    def _k_move(name: str, tx: float, ty: float, tz: float, gripper_deg: float) -> None:
        """φ_fixed IK 在 Pi 端求解 → 6 关节舵机角 → K 命令。

        MCU 的 M(坐标) 路径已废弃(直接拒绝)，执行路径统一走 K。
        IK 不可达转 RuntimeError，交由 run_loop 急停处理。
        """
        try:
            angles = ik(
                (tx, ty, tz),
                gripper_deg=gripper_deg,
                wrist_rotate_deg=cfg.WRIST_ROTATE_GRASP_DEG,
            )
        except IKUnreachableError as exc:
            raise RuntimeError(f"{name} IK 不可达: {exc}") from exc
        servo = angles_to_servo_degs(angles)
        _do(
            f"K {name} servo={['%.0f' % s for s in servo]}",
            (lambda: proto.send_K(servo)) if proto else None,
        )

    _do("HOME", proto.send_home if proto else None)
    _do("OPEN", proto.send_open if proto else None)
    _k_move(
        f"pre_grasp({x:.0f},{y:.0f},{pre_z:.0f})", x, y, pre_z, cfg.GRIPPER_OPEN_DEG
    )
    _k_move(f"grasp({x:.0f},{y:.0f},{z:.0f})", x, y, z, cfg.GRIPPER_OPEN_DEG)
    _do("CLOSE", proto.send_close if proto else None)
    # lift 在闭爪后，夹爪保持闭合状态以夹住草莓
    _k_move(f"lift({x:.0f},{y:.0f},{lift_z:.0f})", x, y, lift_z, cfg.GRIPPER_CLOSE_DEG)
    _do(
        f"PLACE {place_label}",
        (lambda: proto.send_place(place_label)) if proto else None,
    )
    _do("HOME (final)", proto.send_home if proto else None)


# ============================================================
# 主循环
# ============================================================


def run_loop(
    args: argparse.Namespace,
    calib: Calibration | None,
    detector: _Detector | None,
    camera: _Camera | None,
    proto: ProtocolV2 | None,
) -> None:
    cycle = 0
    while True:
        cycle += 1
        logger.info("─── 循环 #%d ───", cycle)

        # 1. 获取目标 bbox（或模拟）+ class_id
        if args.simulate_uv is not None:
            u, v, sim_cls = args.simulate_uv
            target_xyxy = (u - 10, v - 10, u + 10, v + 10)
            class_id = int(sim_cls)
            logger.info(
                "模拟目标 uv=(%.0f,%.0f) class=%s", u, v, CLASS_NAMES.get(class_id, "?")
            )
        else:
            assert camera is not None and detector is not None
            frame = camera.read()
            target = detector.detect_best(frame)
            if target is None:
                logger.debug("无检测，间隔 %.1fs 重试", DETECT_INTERVAL_S)
                time.sleep(DETECT_INTERVAL_S)
                if args.once:
                    logger.warning("--once 模式但无检测目标，退出")
                    return
                continue
            target_xyxy = target.xyxy
            class_id = target.class_id
            logger.info(
                "检测到 %s bbox=(%.0f,%.0f,%.0f,%.0f) conf=%.2f",
                CLASS_NAMES[class_id],
                *target_xyxy,
                target.confidence,
            )

        # 2. 像素 → 臂基坐标
        if calib is None:
            assert args.no_calib, "未加载标定，但 --no-calib 也没开"
            # 调试：用 simulate-uv 把数值当 mm 直接用
            xa, ya, za = float(target_xyxy[0]), float(target_xyxy[1]), 0.0
            logger.warning("[NO-CALIB] uv 当作 (X,Y) mm 用，Z=0")
        else:
            try:
                xa, ya, za = bbox_to_armbase(target_xyxy, calib)
            except Exception as exc:
                logger.error("坐标变换失败: %s", exc)
                if args.once:
                    return
                continue

        logger.info("臂基坐标 (X,Y,Z)=(%.1f, %.1f, %.1f) mm", xa, ya, za)

        # 3. IK 求解（本地预检；MCU 是第二道防线）
        try:
            angles = ik((xa, ya, za))
        except IKUnreachableError as exc:
            logger.warning("IK 不可达，跳过: %s", exc)
            if args.once:
                return
            continue
        servo_degs = angles_to_servo_degs(angles)
        logger.info(
            "舵机角度 (base,shoulder,elbow,wp,wr,grip)=%s",
            ", ".join(f"{a:.1f}" for a in servo_degs),
        )

        # 4. 执行抓取-放置
        place_label = CLASS_TO_PLACE[class_id]
        try:
            execute_pickup(proto, (xa, ya, za), place_label)
        except RuntimeError as exc:
            logger.error("抓取序列失败: %s — 急停 + 退出", exc)
            if proto is not None:
                try:
                    proto.send_emergency_stop()
                except Exception:
                    logger.exception("急停发送也失败了")
            return

        if args.once:
            logger.info("--once 模式，完成一次抓取后退出")
            return

        time.sleep(0.5)


# ============================================================
# 入口
# ============================================================


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="pickup_v2 主程序")
    p.add_argument("--dry-run", action="store_true", help="不真发串口，只打印命令")
    p.add_argument("--once", action="store_true", help="抓一个目标后退出")
    p.add_argument(
        "--no-calib",
        action="store_true",
        help="跳过标定加载（配合 --simulate-uv 用，把 uv 当 X,Y mm）",
    )
    p.add_argument(
        "--simulate-uv",
        nargs=3,
        metavar=("U", "V", "CLS"),
        type=float,
        default=None,
        help="跳过相机/YOLO，模拟像素点 (u v class_id)",
    )
    p.add_argument(
        "--model", default="models/strawberry_yolov8n.pt", help="YOLO 权重路径"
    )
    p.add_argument("--port", default=None, help="覆盖 config_v2.SERIAL_PORT")
    p.add_argument(
        "--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"]
    )
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    logging.basicConfig(
        level=args.log_level,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    # 标定（除非显式 --no-calib）
    calib: Calibration | None = None
    if not args.no_calib:
        try:
            calib = load_calibration()
            logger.info("标定文件加载成功")
        except FileNotFoundError as exc:
            logger.error("标定缺失: %s", exc)
            logger.error(
                "请先跑阶段 A（intrinsic_calib + homography），"
                "或加 --no-calib 跳过（仅 --simulate-uv 时有意义）"
            )
            return 2

    # 摄像头 + 检测器（除非 --simulate-uv）
    camera: _Camera | None = None
    detector: _Detector | None = None
    if args.simulate_uv is None:
        try:
            camera = _Camera()
            detector = _Detector(args.model)
        except Exception:
            logger.exception("摄像头或 YOLO 初始化失败")
            return 3

    # 串口
    proto: ProtocolV2 | None = None
    if not args.dry_run:
        proto = ProtocolV2(port=args.port)
        try:
            proto.open()
            proto.drain()
        except Exception:
            logger.exception("串口打开失败")
            return 4

    # 主循环
    exit_code = 0
    try:
        run_loop(args, calib, detector, camera, proto)
    except KeyboardInterrupt:
        logger.info("Ctrl-C — 收尾")
    except Exception:
        logger.exception("未预期异常 — 急停")
        if proto is not None:
            try:
                proto.send_emergency_stop()
            except Exception:
                pass
        exit_code = 1
    finally:
        if proto is not None:
            try:
                proto.send_home()
            except Exception:
                logger.warning("收尾 HOME 失败（可忽略）")
            proto.close()
        if camera is not None:
            camera.close()

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
