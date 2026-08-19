import contextlib
import sys
from typing import Callable, Dict, List, Optional, Tuple, Union

from pylabrobot.resources.errors import (
  TooLittleLiquidError,
  TooLittleVolumeError,
)
from pylabrobot.resources.liquid import Liquid
from pylabrobot.serializer import SerializableMixin, deserialize, serialize

this = sys.modules[__name__]
this.volume_tracking_enabled = False  # type: ignore

# 单位分池：质量单位（固体）与体积单位（液体）互不混算。
# volume 只统计体积单位、mass 只统计质量单位；按比例移除、容量校验也各自分池。
MASS_UNITS = frozenset({"ug", "mg", "g"})


def set_volume_tracking(enabled: bool):
  this.volume_tracking_enabled = enabled  # type: ignore


def does_volume_tracking() -> bool:
  return this.volume_tracking_enabled  # type: ignore


@contextlib.contextmanager
def no_volume_tracking():
  old_value = this.volume_tracking_enabled
  this.volume_tracking_enabled = False  # type: ignore
  try:
    yield
  finally:
    this.volume_tracking_enabled = old_value  # type: ignore


VolumeTrackerCallback = Callable[[], None]


class VolumeTracker(SerializableMixin):
  """A volume tracker tracks operations that change the volume in a container and raises errors
  if the volume operations are invalid.

  History format:
      - ("liquid_name", +volume, unit): Add liquid
      - (None, -volume, unit): Remove liquid proportionally from all liquids

  liquid_name is always a string (or None for proportional removal).
  unit is "ul" (microliter) or "ug" (microgram), defaults to "ul".
  Backward compatible with 2-element tuples (name, volume) — unit defaults to "ul".
  """

  def __init__(
    self,
    thing: str,
    max_volume: float,
    liquid_history: Optional[List[Tuple[Optional[str], float, str]]] = None,
    unknown_counter: int = 0,
    # Backward compatibility
    liquids: Optional[List[Tuple[Optional[Liquid], float]]] = None,
    initial_volume: Optional[float] = None,
  ) -> None:
    self._is_disabled = False
    self.thing = thing
    self.max_volume = max_volume
    self._unknown_counter = unknown_counter

    # Liquid history: list of (liquid_name, volume, unit) tuples in chronological order
    # Positive volume = add, negative volume = remove (proportionally when name is None)
    # unit: "ul" or "ug", defaults to "ul"; backward compatible with 2-element tuples
    self.liquid_history: List[Tuple[Optional[str], float, str]] = liquid_history or []

    # Backward compatibility: convert old liquids format to history
    if liquids is not None and len(self.liquid_history) == 0:
      for liq, vol in liquids:
        if abs(vol) > 1e-9:
          name = self._get_liquid_name(liq)
          self.liquid_history.append((name, vol, "ul"))
    elif initial_volume is not None and initial_volume > 0 and len(self.liquid_history) == 0:
      self._unknown_counter += 1
      self.liquid_history.append((f"Unknown{self._unknown_counter}", initial_volume, "ul"))

    self._callback: Optional[VolumeTrackerCallback] = None
    self._checkpoint: int = len(self.liquid_history)

  def _get_liquid_name(self, liquid: Union[Liquid, str, None]) -> str:
    """Convert a Liquid, string, or None to a liquid name string."""
    if liquid is None:
      self._unknown_counter += 1
      return f"Unknown{self._unknown_counter}"
    if isinstance(liquid, str):
      return liquid
    return liquid.name

  @staticmethod
  def _normalize_unit(unit: str) -> str:
    """统一单位大小写：uL→ul, mL→ml, uG→ug 等。"""
    return unit.lower() if unit else "ul"

  @staticmethod
  def _is_mass_unit(unit: str) -> bool:
    """单位是否为质量（固体）。质量与体积分池：volume 只算液体、mass 只算固体。"""
    return VolumeTracker._normalize_unit(unit) in MASS_UNITS

  @staticmethod
  def _unpack_entry(entry) -> Tuple[Optional[str], float, str]:
    """解包历史记录条目，兼容2元素和3元素元组，默认 unit 为 'ul'。"""
    if len(entry) >= 3:
      return entry[0], entry[1], VolumeTracker._normalize_unit(entry[2])
    return entry[0], entry[1], "ul"

  @property
  def is_disabled(self) -> bool:
    return self._is_disabled

  @property
  def current_liquids(self) -> Dict[Tuple[str, str], float]:
    """Calculate current liquid state from history.

    Processes history from start to end:
    - Positive volume: add that liquid
    - Negative volume with None: remove proportionally from all liquids

    Returns:
        Dict mapping (liquid_name, unit) to volume.
    """
    liquids: Dict[Tuple[str, str], float] = {}

    for entry in self.liquid_history:
      name, vol, unit = self._unpack_entry(entry)
      if vol > 0:
        if name is not None:
          key = (name, unit)
          liquids[key] = liquids.get(key, 0) + vol
      elif vol < 0 and name is None:
        # 按比例移除仅在同单位池内进行（体积扣体积、质量扣质量），
        # 避免「移除液体」把固体也一起按比例扣减。
        remove_vol = -vol
        same_unit_keys = [k for k in liquids if k[1] == unit]
        total_vol = sum(liquids[k] for k in same_unit_keys)
        if total_vol > 1e-9:
          ratio = min(remove_vol / total_vol, 1.0)
          for k in same_unit_keys:
            liquids[k] -= liquids[k] * ratio
            if liquids[k] < 1e-9:
              del liquids[k]
      elif vol < 0 and name is not None:
        key = (name, unit)
        if key in liquids:
          liquids[key] = max(0, liquids[key] + vol)
          if liquids[key] < 1e-9:
            del liquids[key]

    return liquids

  @property
  def substances(self) -> List[Tuple[str, float, str]]:
    """返回所有当前物料，格式为 ``(名称, 数量, 单位)``。

    与 :attr:`liquids` 不同，此属性同时包含按体积记录的液体和按质量记录的固体。
    """
    return [(name, amount, unit) for (name, unit), amount in self.current_liquids.items()]

  @property
  def liquids(self) -> List[Tuple[str, float, str]]:
    """返回当前液体，格式为 ``(名称, 体积, 单位)``，不包含固体。"""
    return [
      (name, volume, unit)
      for (name, unit), volume in self.current_liquids.items()
      if not self._is_mass_unit(unit)
    ]

  @liquids.setter
  def liquids(self, value: List[Tuple]) -> None:
    """Set contents by clearing current ones and adding new ones.

    Accepts (liquid, volume) or (liquid, volume, unit) tuples.
    “设置”为替换语义：先按 (名称,单位) 精确清空当前全部内容（液体池+固体池），再写入新内容。
    """
    for (name, unit), vol in list(self.current_liquids.items()):
      if vol > 1e-9:
        self.liquid_history.append((name, -vol, unit))
    for item in value:
      liq, vol = item[0], item[1]
      unit = self._normalize_unit(item[2]) if len(item) >= 3 else "ul"
      if abs(vol) > 1e-9:
        name = self._get_liquid_name(liq)
        self.liquid_history.append((name, vol, unit))

  @property
  def volume(self) -> float:
    """当前体积：仅统计液体（体积单位，如 ul）。固体（质量/ug）见 ``mass``，两者分池互不混算。"""
    return sum(v for (_n, unit), v in self.current_liquids.items() if not self._is_mass_unit(unit))

  @property
  def mass(self) -> float:
    """当前质量：仅统计固体（质量单位，如 ug）。液体（体积/ul）见 ``volume``，两者分池互不混算。"""
    return sum(v for (_n, unit), v in self.current_liquids.items() if self._is_mass_unit(unit))

  @volume.setter
  def volume(self, value: float) -> None:
    """Set volume by proportionally adjusting."""
    current_volume = self.volume
    diff = value - current_volume
    if abs(diff) > 1e-9:
      if diff > 0:
        self._unknown_counter += 1
        self.liquid_history.append((f"Unknown{self._unknown_counter}", diff, "ul"))
      else:
        self.liquid_history.append((None, diff, "ul"))

  @property
  def pending_volume(self) -> float:
    """Get the pending volume. (Deprecated: same as volume now)"""
    return self.volume

  @pending_volume.setter
  def pending_volume(self, value: float) -> None:
    """Set pending volume. (Deprecated: same as setting volume now)"""
    self.volume = value

  @property
  def liquid_names(self) -> List[str]:
    """Get all liquid names from history (excluding None for removals)."""
    names: List[str] = []
    for entry in self.liquid_history:
      name, vol, _unit = self._unpack_entry(entry)
      if name is not None and vol > 0:
        if name not in names:
          names.append(name)
    return names

  def disable(self) -> None:
    """Disable the volume tracker."""
    self._is_disabled = True

  def enable(self) -> None:
    """Enable the volume tracker."""
    self._is_disabled = False

  def set_volume(self, volume: float, name: Optional[str] = None, unit: str = "ul") -> None:
    """Set the volume in the container."""
    self.liquids = [(name, volume, unit)]
    self._checkpoint = len(self.liquid_history)

    if self._callback is not None:
      self._callback()

  def set_liquids(self, liquids: List[Tuple[Optional["Liquid"], float, str]]) -> None:
    """Set the liquids in the container."""
    self.liquids = liquids
    self._checkpoint = len(self.liquid_history)

    if self._callback is not None:
      self._callback()

  def remove_liquid(self, volume: float) -> List[Tuple[str, float, str]]:
    """Remove liquid from the container proportionally.

    Args:
        volume: Volume to remove.

    Returns:
        List of (liquid_name, volume, unit) tuples that were removed.
    """
    available_volume = self.volume
    if (volume - available_volume) > 1e-6:
      raise TooLittleLiquidError(
        f"Container {self.thing} has too little liquid: {volume}uL > {available_volume}uL."
      )

    # 仅在液体池内按比例移除（固体不参与体积吸取）；下面的 (None,-volume,"ul") 也只扣液体池
    current = {k: v for k, v in self.current_liquids.items() if not self._is_mass_unit(k[1])}
    total_vol = sum(current.values())
    removed: List[Tuple[str, float, str]] = []
    if total_vol > 1e-9:
      ratio = volume / total_vol
      for (name, unit), vol in current.items():
        removed_vol = vol * ratio
        if removed_vol > 1e-9:
          removed.append((name, removed_vol, unit))

    self.liquid_history.append((None, -volume, "ul"))

    if self._callback is not None:
      self._callback()

    return removed

  def add_liquid(
    self, liquid_or_volume=-1, volume: Optional[float] = None, unit: str = "ul"
  ) -> None:
    """Add liquid to the container.

    Supports multiple calling conventions:
        - add_liquid(100.0)                    - positional volume, unknown liquid
        - add_liquid(volume=100.0)             - keyword volume, unknown liquid
        - add_liquid(Liquid.WATER, 100.0)      - typed liquid + volume
        - add_liquid("water", 100.0)           - liquid by name + volume
        - add_liquid("water", 100.0, unit="ug") - with unit

    Args:
        unit: 单位，仅支持 "ul" 和 "ug"，默认 "ul"。
    """
    if liquid_or_volume == -1 and volume is None:
      raise TypeError("add_liquid() requires at least a volume argument")
    if liquid_or_volume == -1:
      assert volume is not None
      actual_volume = float(volume)
      actual_liquid: Union[Liquid, str, None] = None
    elif volume is None:
      actual_volume = float(liquid_or_volume)
      actual_liquid = None
    else:
      actual_liquid = liquid_or_volume  # type: ignore[assignment]
      actual_volume = volume

    # 固体（质量单位）不占用液体体积容量，跳过体积超容检查
    norm_unit = self._normalize_unit(unit)
    if not self._is_mass_unit(norm_unit) and (actual_volume - self.get_free_volume()) > 1e-6:
      raise TooLittleVolumeError(
        f"Container {self.thing} has too little volume: {actual_volume}uL > {self.get_free_volume()}uL."
      )

    name = self._get_liquid_name(actual_liquid)
    self.liquid_history.append((name, actual_volume, norm_unit))

    if self._callback is not None:
      self._callback()

  def get_used_volume(self) -> float:
    """Get the used volume of the container."""
    return self.volume

  def get_free_volume(self) -> float:
    """Get the free volume of the container."""
    return self.max_volume - self.get_used_volume()

  def get_liquids(self, top_volume: Optional[float] = None) -> List[Tuple[str, float, str]]:
    """Get the current liquids (液体池, 体积单位) in the container.

    固体（质量/ug）不在此返回，见 ``get_solids``。

    Args:
        top_volume: If specified, get only the top N uL (proportionally).
                   If None, returns all liquids.

    Returns:
        List of (liquid_name, volume, unit) tuples.
    """
    current = {k: v for k, v in self.current_liquids.items() if not self._is_mass_unit(k[1])}
    total_vol = sum(current.values())

    if top_volume is None or top_volume >= total_vol:
      return [(name, vol, unit) for (name, unit), vol in current.items()]

    if (top_volume - total_vol) > 1e-6:
      raise TooLittleLiquidError(f"Tracker only has {total_vol}uL")

    ratio = top_volume / total_vol if total_vol > 1e-9 else 0
    return [
      (name, vol * ratio, unit) for (name, unit), vol in current.items() if vol * ratio > 1e-9
    ]

  def get_solids(self) -> List[Tuple[str, float, str]]:
    """Get the current solids (固体池, 质量单位 如 ug) in the container.

    液体（体积/ul）见 ``get_liquids``。

    Returns:
        List of (solid_name, mass, unit) tuples.
    """
    return [
      (name, mass, unit)
      for (name, unit), mass in self.current_liquids.items()
      if self._is_mass_unit(unit)
    ]

  def commit(self) -> None:
    """Commit the pending operations."""
    if self.is_disabled:
      raise RuntimeError(f"Volume tracker {self.thing} is disabled. Call `enable()`.")
    self._checkpoint = len(self.liquid_history)

    if self._callback is not None:
      self._callback()

  def rollback(self) -> None:
    """Rollback operations since the last commit."""
    if self.is_disabled:
      raise RuntimeError("Volume tracker is disabled. Call `enable()`.")
    self.liquid_history = self.liquid_history[: self._checkpoint]

  def clear_liquid_history(self) -> None:
    """Clears the liquid history. Use when there is a wash step."""
    self.liquid_history.clear()
    self._unknown_counter = 0
    self._checkpoint = 0

  def serialize(self) -> dict:
    """Serialize the volume tracker."""
    return {
      "thing": self.thing,
      "max_volume": serialize(self.max_volume),
      "volume": self.volume,
      "mass": self.mass,
      "liquids": self.liquids,
      "substances": self.substances,
      "liquid_history": self.liquid_history,
      "unknown_counter": self._unknown_counter,
    }

  def load_state(self, state: dict) -> None:
    """Load the state of the volume tracker."""
    if "thing" in state:
      self.thing = state["thing"]
    if "max_volume" in state:
      self.max_volume = deserialize(state["max_volume"])
    if "unknown_counter" in state:
      self._unknown_counter = state["unknown_counter"]

    if "liquid_history" in state:
      self.liquid_history = []
      for data in state["liquid_history"]:
        if isinstance(data, (list, tuple)):
          name, vol = data[0], data[1]
          unit = self._normalize_unit(data[2]) if len(data) >= 3 else "ul"
          self.liquid_history.append((name, vol, unit))
        else:
          self.liquid_history.append((data, 0, "ul"))
    elif "substances" in state or "liquids" in state:
      # 新状态用 ``substances`` 表示全部内容物；``liquids`` 兼容尚未区分固体的旧状态。
      for item in state.get("substances", state.get("liquids", [])):
        if isinstance(item, (list, tuple)):
          liq = item[0]
          vol = float(item[1]) if len(item) > 1 else 0
          unit = self._normalize_unit(item[2]) if len(item) >= 3 else "ul"
        else:
          liq, vol = deserialize(item)
          unit = "ul"
        if vol is None:
          continue
        if abs(vol) > 1e-9:
          name = self._get_liquid_name(liq)
          self.liquid_history.append((name, vol, unit))
    elif "volume" in state:
      vol = deserialize(state["volume"])
      if vol > 0:
        self._unknown_counter += 1
        self.liquid_history.append((f"Unknown{self._unknown_counter}", vol, "ul"))

  def register_callback(self, callback: VolumeTrackerCallback) -> None:
    self._callback = callback
