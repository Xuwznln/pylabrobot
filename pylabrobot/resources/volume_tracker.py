import contextlib
import sys
from typing import Callable, Dict, List, Optional, Tuple, Union

from pylabrobot.resources.errors import (
  TooLittleLiquidError,
  TooLittleVolumeError,
)
from pylabrobot.resources.liquid import Liquid
from pylabrobot.serializer import SerializableMixin
from pylabrobot.serializer import deserialize, serialize

this = sys.modules[__name__]
this.volume_tracking_enabled = False  # type: ignore


def set_volume_tracking(enabled: bool):
  this.volume_tracking_enabled = enabled  # type: ignore


def does_volume_tracking() -> bool:
  return this.volume_tracking_enabled  # type: ignore


@contextlib.contextmanager
def no_volume_tracking():
  old_value = this.volume_tracking_enabled
  this.volume_tracking_enabled = False  # type: ignore
  yield
  this.volume_tracking_enabled = old_value  # type: ignore


VolumeTrackerCallback = Callable[[], None]


class VolumeTracker(SerializableMixin):
  """A volume tracker tracks operations that change the volume in a container and raises errors
  if the volume operations are invalid.

  History format:
      - ("liquid_name", +volume): Add liquid
      - (None, -volume): Remove liquid proportionally from all liquids

  liquid_name is always a string (or None for proportional removal).
  """

  def __init__(
    self,
    thing: str,
    max_volume: float,
    liquid_history: Optional[List[Tuple[Optional[str], float]]] = None,
    unknown_counter: int = 0,
    # Backward compatibility
    liquids: Optional[List[Tuple[Optional[Liquid], float]]] = None,
    initial_volume: Optional[float] = None,
  ) -> None:
    self._is_disabled = False
    self.thing = thing
    self.max_volume = max_volume
    self._unknown_counter = unknown_counter

    # Liquid history: list of (liquid_name, volume) tuples in chronological order
    # Positive volume = add, negative volume = remove (proportionally when name is None)
    self.liquid_history: List[Tuple[Optional[str], float]] = liquid_history or []

    # Backward compatibility: convert old liquids format to history
    if liquids is not None and len(self.liquid_history) == 0:
      for liq, vol in liquids:
        if abs(vol) > 1e-9:
          name = self._get_liquid_name(liq)
          self.liquid_history.append((name, vol))
    elif initial_volume is not None and initial_volume > 0 and len(self.liquid_history) == 0:
      self._unknown_counter += 1
      self.liquid_history.append((f"Unknown{self._unknown_counter}", initial_volume))

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

  @property
  def is_disabled(self) -> bool:
    return self._is_disabled

  @property
  def current_liquids(self) -> Dict[str, float]:
    """Calculate current liquid state from history.

    Processes history from start to end:
    - Positive volume: add that liquid
    - Negative volume with None: remove proportionally from all liquids

    Returns:
        Dict mapping liquid name to volume.
    """
    liquids: Dict[str, float] = {}

    for name, vol in self.liquid_history:
      if vol > 0:
        # Add liquid
        if name is not None:
          liquids[name] = liquids.get(name, 0) + vol
      elif vol < 0 and name is None:
        # Proportional removal
        remove_vol = -vol
        total_vol = sum(liquids.values())
        if total_vol > 1e-9:
          ratio = remove_vol / total_vol
          ratio = min(ratio, 1.0)  # Can't remove more than 100%
          for liq_name in list(liquids.keys()):
            liquids[liq_name] -= liquids[liq_name] * ratio
            if liquids[liq_name] < 1e-9:
              del liquids[liq_name]
      elif vol < 0 and name is not None:
        # Specific liquid removal (legacy support)
        if name in liquids:
          liquids[name] = max(0, liquids[name] + vol)
          if liquids[name] < 1e-9:
            del liquids[name]

    return liquids

  @property
  def liquids(self) -> List[Tuple[str, float]]:
    """Get current liquids as a list of (liquid_name, volume) tuples."""
    return list(self.current_liquids.items())

  @liquids.setter
  def liquids(self, value: List[Tuple[Optional[Liquid], float]]) -> None:
    """Set liquids by clearing history and adding new liquids."""
    # Record removal of current volume
    current_vol = self.volume
    if current_vol > 1e-9:
      self.liquid_history.append((None, -current_vol))
    # Add new liquids
    for liq, vol in value:
      if abs(vol) > 1e-9:
        name = self._get_liquid_name(liq)
        self.liquid_history.append((name, vol))

  @property
  def volume(self) -> float:
    """Get the current volume (sum of all liquids)."""
    return sum(self.current_liquids.values())

  @volume.setter
  def volume(self, value: float) -> None:
    """Set volume by proportionally adjusting."""
    current_volume = self.volume
    diff = value - current_volume
    if abs(diff) > 1e-9:
      if diff > 0:
        # Add unknown liquid
        self._unknown_counter += 1
        self.liquid_history.append((f"Unknown{self._unknown_counter}", diff))
      else:
        # Remove proportionally
        self.liquid_history.append((None, diff))

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
    for name, vol in self.liquid_history:
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

  def set_volume(self, volume: float) -> None:
    """Set the volume in the container."""
    self.volume = volume
    self._checkpoint = len(self.liquid_history)

    if self._callback is not None:
      self._callback()

  def set_liquids(self, liquids: List[Tuple[Optional["Liquid"], float]]) -> None:
    """Set the liquids in the container."""
    self.liquids = liquids
    self._checkpoint = len(self.liquid_history)

    if self._callback is not None:
      self._callback()

  def remove_liquid(self, volume: float) -> List[Tuple[str, float]]:
    """Remove liquid from the container proportionally.

    Args:
        volume: Volume to remove.

    Returns:
        List of (liquid_name, volume) tuples that were removed.
    """
    available_volume = self.volume
    if (volume - available_volume) > 1e-6:
      raise TooLittleLiquidError(
        f"Container {self.thing} has too little liquid: {volume}uL > {available_volume}uL."
      )

    # Calculate what will be removed
    current = self.current_liquids
    total_vol = sum(current.values())
    removed: List[Tuple[str, float]] = []
    if total_vol > 1e-9:
      ratio = volume / total_vol
      for name, vol in current.items():
        removed_vol = vol * ratio
        if removed_vol > 1e-9:
          removed.append((name, removed_vol))

    # Record proportional removal in history
    self.liquid_history.append((None, -volume))

    if self._callback is not None:
      self._callback()

    return removed

  def add_liquid(self, liquid_or_volume=-1, volume: Optional[float] = None) -> None:
    """Add liquid to the container.

    Supports multiple calling conventions:
        - add_liquid(100.0)                    - positional volume, unknown liquid
        - add_liquid(volume=100.0)             - keyword volume, unknown liquid
        - add_liquid(Liquid.WATER, 100.0)      - typed liquid + volume
        - add_liquid("water", 100.0)           - liquid by name + volume
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

    if (actual_volume - self.get_free_volume()) > 1e-6:
      raise TooLittleVolumeError(
        f"Container {self.thing} has too little volume: {actual_volume}uL > {self.get_free_volume()}uL."
      )

    # Get liquid name (auto-generate name for None)
    name = self._get_liquid_name(actual_liquid)

    # Record in liquid history
    self.liquid_history.append((name, actual_volume))

    if self._callback is not None:
      self._callback()

  def get_used_volume(self) -> float:
    """Get the used volume of the container."""
    return self.volume

  def get_free_volume(self) -> float:
    """Get the free volume of the container."""
    return self.max_volume - self.get_used_volume()

  def get_liquids(self, top_volume: Optional[float] = None) -> List[Tuple[str, float]]:
    """Get the current liquids in the container.

    Args:
        top_volume: If specified, get only the top N uL (proportionally).
                   If None, returns all liquids.

    Returns:
        List of (liquid_name, volume) tuples.
    """
    current = self.current_liquids
    total_vol = sum(current.values())

    if top_volume is None or top_volume >= total_vol:
      return list(current.items())

    if (top_volume - total_vol) > 1e-6:
      raise TooLittleLiquidError(f"Tracker only has {total_vol}uL")

    # Return proportional amounts
    ratio = top_volume / total_vol if total_vol > 1e-9 else 0
    return [(name, vol * ratio) for name, vol in current.items() if vol * ratio > 1e-9]

  def commit(self) -> None:
    """Commit the pending operations."""
    assert not self.is_disabled, f"Volume tracker {self.thing} is disabled. Call `enable()`."
    self._checkpoint = len(self.liquid_history)

    if self._callback is not None:
      self._callback()

  def rollback(self) -> None:
    """Rollback operations since the last commit."""
    assert not self.is_disabled, "Volume tracker is disabled. Call `enable()`."
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
      "liquids": self.liquids,
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
      # Convert list of lists back to list of tuples
      self.liquid_history = [
        (data[0], data[1]) if isinstance(data, (list, tuple)) else (data, 0)
        for data in state["liquid_history"]
      ]
    # Backward compatibility: convert old format
    elif "liquids" in state:
      for item in state["liquids"]:
        liq, vol = deserialize(item)
        if vol is None:
          continue
        if abs(vol) > 1e-9:
          name = self._get_liquid_name(liq)
          self.liquid_history.append((name, vol))
    elif "volume" in state:
      vol = deserialize(state["volume"])
      if vol > 0:
        self._unknown_counter += 1
        self.liquid_history.append((f"Unknown{self._unknown_counter}", vol))

  def register_callback(self, callback: VolumeTrackerCallback) -> None:
    self._callback = callback
