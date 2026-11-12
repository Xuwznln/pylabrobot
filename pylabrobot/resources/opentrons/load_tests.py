"""共享耗材定义转换为资源树的离线回归测试。"""

from copy import deepcopy

import pytest

from pylabrobot.resources import Coordinate, Resource, Tip
from pylabrobot.resources.opentrons.load import ot_definition_to_resource

labware = pytest.importorskip("opentrons_shared_data.labware")


@pytest.mark.parametrize(
  "definition",
  [
    "eppendorf_96_tiprack_1000ul_eptips",
    "eppendorf_96_tiprack_10ul_eptips",
    "geb_96_tiprack_1000ul",
    "geb_96_tiprack_10ul",
    "opentrons_96_filtertiprack_1000ul",
    "opentrons_96_filtertiprack_10ul",
    "opentrons_96_filtertiprack_200ul",
    "opentrons_96_filtertiprack_20ul",
    "opentrons_96_tiprack_1000ul",
    "opentrons_96_tiprack_10ul",
    "opentrons_96_tiprack_20ul",
    "opentrons_96_tiprack_300ul",
  ],
)
def test_shared_tip_definition_roundtrip(definition):
  """直径、长度、容量来自耗材定义；结构和占用状态可往返。"""
  data = labware.load_definition(definition, 1)
  rack = ot_definition_to_resource(data, "rack")
  restored = Resource.deserialize(rack.serialize())
  restored.load_all_state(rack.serialize_all_state())
  spot = restored.get_item("A1")
  tip = spot.tip
  assert isinstance(tip, Tip)
  assert tip.parent is spot
  assert tip.get_size_x() == data["wells"]["A1"]["diameter"]
  assert tip.get_size_z() == data["parameters"]["tipLength"]
  assert tip.maximal_volume == data["wells"]["A1"]["totalLiquidVolume"]
  assert "total_tip_length" not in tip.serialize()
  assert "children" not in spot.serialize()


def test_factory_binds_each_well_and_racks_have_distinct_names():
  """每个工厂绑定自身孔位定义，同型号的两个枪头盒不共享节点名。"""
  data = deepcopy(labware.load_definition("opentrons_96_tiprack_300ul", 1))
  data["wells"]["A1"]["diameter"] = 6.1
  data["wells"]["A1"]["totalLiquidVolume"] = 275
  deck = Resource("deck", size_x=300, size_y=300, size_z=100)
  for name in ("rack_one", "rack_two"):
    rack = ot_definition_to_resource(data, name)
    tip = rack.get_item("A1").make_tip()
    assert tip.get_size_x() == 6.1
    assert tip.maximal_volume == 275
    assert rack.get_item("B1").make_tip().get_size_x() == 5.23
    deck.assign_child_resource(rack, location=Coordinate.zero())
