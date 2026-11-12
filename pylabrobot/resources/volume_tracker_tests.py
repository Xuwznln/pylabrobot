import unittest

from pylabrobot.resources.errors import (
  TooLittleLiquidError,
  TooLittleVolumeError,
)
from pylabrobot.resources.volume_tracker import VolumeTracker


class TestVolumeTracker(unittest.TestCase):
  """Test for the tip volume tracker"""

  def test_history_checkpoint_survives_state_roundtrip(self):
    """保存后恢复仍可回滚待提交操作，固体和液体互不影响。"""
    tracker = VolumeTracker(thing="source", max_volume=100)
    tracker.set_liquids([("water", 60, "ul"), ("salt", 500, "ug")])
    tracker.remove_liquid(20)
    restored = VolumeTracker(thing="restored", max_volume=100)
    restored.load_state(tracker.serialize())
    self.assertEqual(restored.get_used_volume(), 40)
    restored.rollback()
    self.assertEqual(restored.substances, [("water", 60, "ul"), ("salt", 500, "ug")])

  def test_load_official_pending_state(self):
    """官方体积状态的待提交差量必须保留，并能回滚到已提交值。"""
    tracker = VolumeTracker(thing="test", max_volume=100)
    tracker.load_state({"volume": 60, "pending_volume": 40})
    self.assertEqual(tracker.get_used_volume(), 40)
    tracker.rollback()
    self.assertEqual(tracker.get_used_volume(), 60)

  def test_loading_substances_replaces_existing_state(self):
    """重复加载同一快照不叠加物料数量。"""
    tracker = VolumeTracker(thing="test", max_volume=100)
    state = {"substances": [("water", 20, "ul")]}
    tracker.load_state(state)
    tracker.load_state(state)
    self.assertEqual(tracker.substances, [("water", 20, "ul")])

  def test_init(self):
    tracker = VolumeTracker(thing="test", max_volume=100)
    self.assertEqual(tracker.get_free_volume(), 100)
    self.assertEqual(tracker.get_used_volume(), 0)

    tracker.set_volume(20)
    self.assertEqual(tracker.get_free_volume(), 80)
    self.assertEqual(tracker.get_used_volume(), 20)

  def test_validation_preserves_pending_volume_and_callbacks(self):
    """Read-only checks use the pending volume without changing or notifying it."""
    tracker = VolumeTracker(thing="test", max_volume=100, initial_volume=60)
    tracker.remove_liquid(20)
    # 维护版以历史即时投影 volume；纯校验必须保持这一投影和历史不变。
    before_volume = tracker.volume
    before_history = list(tracker.liquid_history)
    calls = []
    tracker.register_callback(lambda: calls.append(tracker.get_used_volume()))
    tracker.validate_remove_liquid(40)
    tracker.validate_add_liquid(60)
    with self.assertRaises(TooLittleLiquidError):
      tracker.validate_remove_liquid(41)
    with self.assertRaises(TooLittleVolumeError):
      tracker.validate_add_liquid(61)
    self.assertEqual(tracker.volume, before_volume)
    self.assertEqual(tracker.liquid_history, before_history)
    self.assertEqual(tracker.get_used_volume(), 40)
    self.assertEqual(calls, [])

  def test_add_liquid(self):
    tracker = VolumeTracker(thing="test", max_volume=100)

    tracker.add_liquid(volume=20)
    self.assertEqual(tracker.get_used_volume(), 20)
    self.assertEqual(tracker.get_free_volume(), 80)

    tracker.commit()
    self.assertEqual(tracker.get_used_volume(), 20)
    self.assertEqual(tracker.get_free_volume(), 80)

    with self.assertRaises(TooLittleVolumeError):
      tracker.add_liquid(volume=100)

  def test_remove_liquid(self):
    tracker = VolumeTracker(thing="test", max_volume=100, initial_volume=60)
    tracker.commit()

    self.assertEqual(tracker.get_used_volume(), 60)
    tracker.remove_liquid(volume=20)
    tracker.commit()
    self.assertEqual(tracker.get_used_volume(), 40)

    with self.assertRaises(TooLittleLiquidError):
      tracker.remove_liquid(volume=100)

  def test_a_callback_registered_twice_is_called_once(self):
    """A tip that enters the same spot again registers the spot's callback again."""
    tracker = VolumeTracker(thing="test", max_volume=100)
    calls: list = []
    callback = lambda: calls.append(1)  # noqa: E731
    tracker.register_callback(callback)
    tracker.register_callback(callback)
    tracker.set_volume(10)
    self.assertEqual(calls, [1])

  def test_a_rollback_tells_the_callbacks(self):
    """A failed operation is rolled back, and a listener is told the volume it last saw is gone."""
    tracker = VolumeTracker(thing="test", max_volume=100, initial_volume=60)
    seen: list = []
    tracker.register_callback(lambda: seen.append(tracker.pending_volume))
    tracker.remove_liquid(volume=20)
    tracker.rollback()
    self.assertEqual(seen, [40, 60])

  def test_liquids_and_substances_are_distinguished(self):
    tracker = VolumeTracker(thing="test", max_volume=100)

    tracker.add_liquid("water", 20, unit="uL")
    tracker.add_liquid("salt", 500, unit="uG")

    self.assertEqual(tracker.liquids, [("water", 20, "ul")])
    self.assertEqual(
      tracker.substances,
      [("water", 20, "ul"), ("salt", 500, "ug")],
    )
    self.assertEqual(tracker.get_solids(), [("salt", 500, "ug")])

    state = tracker.serialize()
    self.assertEqual(state["liquids"], [("water", 20, "ul")])
    self.assertEqual(state["substances"], tracker.substances)

  def test_load_state_prefers_substances_over_legacy_liquids(self):
    tracker = VolumeTracker(thing="test", max_volume=100)

    tracker.load_state(
      {
        "liquids": [("stale", 1, "ul")],
        "substances": [("water", 20, "ul"), ("salt", 500, "ug")],
      }
    )

    self.assertEqual(tracker.liquids, [("water", 20, "ul")])
    self.assertEqual(
      tracker.substances,
      [("water", 20, "ul"), ("salt", 500, "ug")],
    )
