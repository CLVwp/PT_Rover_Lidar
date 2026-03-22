"""
Envoi de commandes au rover par WebSocket (port 81).
Utilisé par les scripts de navigation autonome et mapping.
"""
import os
import json
import threading
from websockets.sync.client import connect

ROVER_IP = os.environ.get("ROS_ROVER_IP", "10.12.20.225")
WS_URL = f"ws://{ROVER_IP}:81"

# T:1 = vitesse (L, R) typiquement dans [-2, 2], on utilise [-1, 1] en API
CMD_SPEED = 1
# T:11 = PWM direct (L, R) dans [-255, 255]
CMD_PWM = 11

_ws = None
_ws_lock = threading.Lock()


def _ensure_ws():
    global _ws
    if _ws is None:
        _ws = connect(WS_URL, open_timeout=1.5)
    return _ws


def _send(cmd: dict) -> dict:
    """Envoie une commande JSON via WebSocket. Pas de retour obligatoire."""
    global _ws
    payload = json.dumps(cmd, separators=(",", ":"))
    with _ws_lock:
        try:
            ws = _ensure_ws()
            ws.send(payload)
            return {}
        except Exception:
            try:
                if _ws is not None:
                    _ws.close()
            except Exception:
                pass
            _ws = None
            # Une tentative de reconnexion immédiate
            try:
                ws = _ensure_ws()
                ws.send(payload)
                return {}
            except Exception:
                return {}


def set_speed(left: float, right: float) -> dict:
    """
    Commande vitesse différentielle. left/right dans [-1, 1] (1 = pleine vitesse avant).
    Le rover utilise setGoalSpeed(L, R) avec L,R dans [-2, 2].
    """
    L = max(-2.0, min(2.0, float(left)))
    R = max(-2.0, min(2.0, float(right)))
    return _send({"T": CMD_SPEED, "L": L, "R": R})


def set_pwm(left: int, right: int) -> dict:
    """PWM direct. left/right dans [-255, 255]."""
    L = max(-255, min(255, int(left)))
    R = max(-255, min(255, int(right)))
    return _send({"T": CMD_PWM, "L": L, "R": R})


def stop() -> dict:
    """Arrêt moteurs."""
    return set_speed(0.0, 0.0)
