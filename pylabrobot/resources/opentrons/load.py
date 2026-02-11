import json
import math
import os
import urllib.request
from typing import TYPE_CHECKING, Dict, List, Union, cast

try:
  import opentrons_shared_data.labware

  USE_OT = True
except ImportError as e:
  USE_OT = False
  _OT_IMPORT_ERROR = e

from pylabrobot.resources import Coordinate, Tip, TipRack, TipSpot
from pylabrobot.resources.carrier import PlateHolder
from pylabrobot.resources.plate import Plate
from pylabrobot.resources.resource_holder import ResourceHolder
from pylabrobot.resources.tube import Tube
from pylabrobot.resources.tube_rack import TubeRack
from pylabrobot.resources.well import CrossSectionType, Well

if TYPE_CHECKING:
  try:
    from opentrons_shared_data.labware.types import LabwareDefinition  # type: ignore
  except ImportError:
    from opentrons_shared_data.labware.dev_types import LabwareDefinition  # type: ignore


class UnknownResourceType(Exception):
  pass


def _download_file(url: str, local_path: str) -> bytes:
  with urllib.request.urlopen(url) as response, open(local_path, "wb") as out_file:
    data = response.read()
    out_file.write(data)
    return data  # type: ignore


def _download_ot_resource_file(ot_name: str, force_download: bool):
  """Download an Opentrons tip rack definition file from GitHub.

  Args:
    ot_name: The name of the tip rack, like "opentrons_96_tiprack_300ul".

  Returns:
    The labware definition as a dictionary.
  """
  url = f"https://raw.githubusercontent.com/Opentrons/opentrons/5b51a98ce736b2bb5aff780bf3fdf91941a038fa/shared-data/labware/definitions/2/{ot_name}/1.json"
  if os.path.exists("/tmp"):
    path = f"/tmp/{ot_name}.json"  # only works with linux/mac systems
  else:
    path = f"C:/Windows/Temp/{ot_name}.json"
  if force_download or not os.path.exists(path):
    data = _download_file(url=url, local_path=path)
  else:
    with open(path, "rb") as f:
      data = f.read()
  return json.loads(data)


def ot_definition_to_resource(
  data: "LabwareDefinition", name: str
) -> Union[Plate, TipRack, TubeRack]:
  """Convert an Opentrons definition file to a PyLabRobot resource file."""

  if not USE_OT:
    raise ImportError(
      "opentrons_shared_data is not installed. "
      f"Import error: {_OT_IMPORT_ERROR}. "
      "run `pip install opentrons_shared_data`"
    )

  display_category = data["metadata"]["displayCategory"]

  size_x = data["dimensions"]["xDimension"]
  size_y = data["dimensions"]["yDimension"]
  size_z = data["dimensions"]["zDimension"]

  tube_rack_display_cats = {"adapter", "aluminumBlock", "tubeRack"}

  if display_category in [
    "wellPlate",
    "tipRack",
    "tubeRack",
    "adapter",
    "aluminumBlock",
    "reservoir",
  ]:
    items = data["ordering"]
    wells: List[Union[TipSpot, Well, Tube]] = []  # TODO: can we use TypeGuard?

    def volume_from_name(name: str) -> float:
      # like "Opentrons 96 Filter Tip Rack 200 µL"
      items = name.split(" ")
      volume, unit = items[-2], items[-1]
      if unit == "mL":
        volume *= 1000
      return float(volume)

    for column in items:
      for item in column:
        well_data = data["wells"][item]

        if well_data["shape"] == "circular":
          diameter = well_data["diameter"]
          # pythagoras. rounding: good enough?
          well_size_x = well_size_y = round(diameter / math.sqrt(2), 3)
        elif "xDimension" in well_data and "yDimension" in well_data:
          well_size_x = well_data["xDimension"]
          well_size_y = well_data["yDimension"]
        else:
          raise ValueError("Unknown well shape.")

        well_size_z = well_data["depth"]

        location = Coordinate(
          x=well_data["x"] - well_size_x / 2,
          y=well_data["y"] - well_size_y / 2,
          z=well_data["z"],
        )

        if display_category == "wellPlate":
          if well_data["shape"] == "rectangular":
            cross_section_type = CrossSectionType.RECTANGLE
          else:
            cross_section_type = CrossSectionType.CIRCLE

          well = Well(
            name=item,
            size_x=well_size_x,
            size_y=well_size_y,
            size_z=well_size_z,
            material_z_thickness=None,  # not known for OT labware
            max_volume=well_data["totalLiquidVolume"],
            cross_section_type=cross_section_type,
          )
          well.location = location
          wells.append(well)
        elif display_category == "tipRack":
          # closure
          def make_tip(name: str) -> Tip:
            return Tip(
              total_tip_length=data["parameters"]["tipLength"],
              has_filter="Filter" in data["metadata"]["displayName"],
              maximal_volume=volume_from_name(data["metadata"]["displayName"]),
              fitting_depth=data["parameters"]["tipOverlap"],
              name=name,
            )

          tip_spot = TipSpot(
            name=item,
            size_x=well_size_x,
            size_y=well_size_y,
            make_tip=make_tip,
          )
          tip_spot.location = location
          wells.append(tip_spot)
        elif display_category in tube_rack_display_cats:
          tube = Tube(
            name=item,
            size_x=well_size_x,
            size_y=well_size_y,
            size_z=well_size_z,
            max_volume=well_data["totalLiquidVolume"],
          )
          tube.location = location
          wells.append(tube)
        elif display_category == "reservoir":
          if well_data["shape"] == "rectangular":
            cross_section_type = CrossSectionType.RECTANGLE
          else:
            cross_section_type = CrossSectionType.CIRCLE

          well = Well(
            name=item,
            size_x=well_size_x,
            size_y=well_size_y,
            size_z=well_size_z,
            max_volume=well_data["totalLiquidVolume"],
            cross_section_type=cross_section_type,
          )
          well.location = location
          wells.append(well)

    ordering = data["ordering"]
    flattened_ordering = [item for sublist in ordering for item in sublist]
    ordered_items = dict(zip(flattened_ordering, wells))

    if display_category == "wellPlate":
      return Plate(
        name=name,
        size_x=size_x,
        size_y=size_y,
        size_z=size_z,
        ordered_items=cast(Dict[str, Well], ordered_items),
        model=data["metadata"]["displayName"],
      )
    if display_category == "tipRack":
      return TipRack(
        name=name,
        size_x=size_x,
        size_y=size_y,
        size_z=size_z,
        ordered_items=cast(Dict[str, TipSpot], ordered_items),
        model=data["metadata"]["displayName"],
      )
    if display_category in tube_rack_display_cats:
      # Implemented for aluminum block adapters for temperature controlling module
      # https://shop.opentrons.com/aluminum-block-set/
      return TubeRack(
        name=name,
        size_x=size_x,
        size_y=size_y,
        size_z=size_z,
        ordered_items=cast(Dict[str, Tube], ordered_items),
        model=data["metadata"]["displayName"],
      )
    if display_category == "reservoir":
      return Plate(
        name=name,
        size_x=size_x,
        size_y=size_y,
        size_z=size_z,
        ordered_items=cast(Dict[str, Well], ordered_items),
        model=data["metadata"]["displayName"],
      )

  raise UnknownResourceType(f"Unknown resource type '{display_category}'.")


def load_opentrons_resource(fn: str, name: str) -> Union[Plate, TipRack, TubeRack]:
  """Load an Opentrons resource from a file.

  Args:
    fn: path to the file.

  Returns:
    A :class:`~pylabrobot.resources.Resource`.

  Raises:
    ValueError: if the file is not a valid opentrons definition file.

    UnknownResourceType: if the file is a valid opentrons definition file, but the resource type is
      not supported.

  Examples:

    Load a tip rack:

    >>> from pylabrobot.resources.opentrons import load_opentrons_resource
    >>> load_opentrons_resource("opentrons/definitions/2/96_standard.json", "96Standard")

  """

  with open(fn, "r", encoding="utf-8") as f:
    data = json.load(f)
  return ot_definition_to_resource(data, name)


def load_shared_opentrons_resource(
  definition: str, name: str, version: int = 1
) -> Union[Plate, TipRack, TubeRack]:
  """Load an Opentrons resource from the shared Opentrons resource library.

  See https://github.com/Opentrons/opentrons/tree/edge/shared-data.

  Args:
    definition: name of the labware definition.
    version: version of the labware definition.
    name: desired name of the PyLabRobot
      :class:`~pylabrobot.resources.Resource`

  Returns:
    A :class:`~pylabrobot.resources.Resource`.

  Raises:
    ValueError: if the file is not a valid opentrons definition file.

    UnknownResourceType: if the file is a valid opentrons definition file, but the resource type is
      not supported.

  Examples:

    Load a tip rack:

    >>> from pylabrobot.resources.opentrons import load_shared_opentrons_resource
    >>> load_shared_opentrons_resource("opentrons_96_tiprack_labware", "96Standard")

  """

  data = opentrons_shared_data.labware.load_definition(definition, version)
  return ot_definition_to_resource(data, name)


def load_ot_tip_rack(
  ot_name: str, plr_resource_name: str, with_tips: bool = True, force_download: bool = False
) -> TipRack:
  """Convert an Opentrons tip rack definition file to a PyLabRobot TipRack resource."""

  data = _download_ot_resource_file(ot_name=ot_name, force_download=force_download)

  display_category = data["metadata"]["displayCategory"]
  if not display_category == "tipRack":
    raise ValueError("Not a tip rack definition file.")

  items = data["ordering"]
  wells: List[TipSpot] = []

  for column in items:
    for item in column:
      well_data = data["wells"][item]

      assert well_data["shape"] == "circular", "We assume all tip racks are circular."
      diameter = well_data["diameter"]
      well_size_x = well_size_y = round(diameter / math.sqrt(2), 3)

      # closure
      def make_tip(name: str) -> Tip:
        return Tip(
          name=name,
          total_tip_length=data["parameters"]["tipLength"],
          has_filter="Filter" in data["metadata"]["displayName"],
          maximal_volume=well_data["totalLiquidVolume"],
          fitting_depth=data["parameters"]["tipOverlap"],
        )

      tip_spot = TipSpot(
        name=item,
        size_x=well_size_x,
        size_y=well_size_y,
        make_tip=make_tip,
      )
      tip_spot.location = Coordinate(
        x=well_data["x"] - well_size_x / 2,
        y=well_data["y"] - well_size_y / 2,
        z=well_data["z"],
      )
      wells.append(tip_spot)

  ordering = data["ordering"]
  flattened_ordering = [item for sublist in ordering for item in sublist]
  ordered_items = dict(zip(flattened_ordering, wells))

  tr = TipRack(
    name=plr_resource_name,
    size_x=data["dimensions"]["xDimension"],
    size_y=data["dimensions"]["yDimension"],
    size_z=data["dimensions"]["zDimension"],
    ordered_items=cast(Dict[str, TipSpot], ordered_items),
    model=data["metadata"]["displayName"],
  )
  if with_tips:
    tr.fill()
  else:
    tr.empty()
  return tr


def load_ot_tube_rack(
  ot_name: str, plr_resource_name: str, force_download: bool = False
) -> TubeRack:
  """Convert an Opentrons tube rack definition file to a PyLabRobot TubeRack resource."""

  data = _download_ot_resource_file(ot_name=ot_name, force_download=force_download)

  display_category = data["metadata"]["displayCategory"]
  if display_category not in {"tubeRack", "aluminumBlock"}:
    raise ValueError("Not a tube rack definition file.")

  items = data["ordering"]
  wells: List[ResourceHolder] = []

  for column in items:
    for item in column:
      well_data = data["wells"][item]

      assert well_data["shape"] == "circular", "We assume all tip racks are circular."
      diameter = well_data["diameter"]
      well_size_x = well_size_y = round(diameter / math.sqrt(2), 3)
      well_size_z = well_data["depth"]

      resource_holder = ResourceHolder(
        name=item,
        size_x=well_size_x,
        size_y=well_size_y,
        size_z=well_size_z,
      )
      resource_holder.location = Coordinate(
        x=well_data["x"] - well_size_x / 2,
        y=well_data["y"] - well_size_y / 2,
        z=well_data["z"],
      )
      wells.append(resource_holder)

  ordering = data["ordering"]
  flattened_ordering = [item for sublist in ordering for item in sublist]
  ordered_items = dict(zip(flattened_ordering, wells))

  return TubeRack(
    name=plr_resource_name,
    size_x=data["dimensions"]["xDimension"],
    size_y=data["dimensions"]["yDimension"],
    size_z=data["dimensions"]["zDimension"],
    ordered_items=cast(Dict[str, ResourceHolder], ordered_items),
    model=data["metadata"]["displayName"],
  )


def load_ot_plate_holder(
  ot_name: str, plr_resource_name: str, z_offset: float, force_download: bool = False
) -> PlateHolder:
  """Convert an Opentrons adapter definition file to a PyLabRobot PlateHolder resource."""

  data = _download_ot_resource_file(ot_name=ot_name, force_download=force_download)

  display_category = data["metadata"]["displayCategory"]
  if display_category not in {"adapter", "aluminumBlock"}:
    raise ValueError("Not a plate adapter definition file.")

  location = data["cornerOffsetFromSlot"]

  return PlateHolder(
    name=plr_resource_name,
    size_x=data["dimensions"]["xDimension"],
    size_y=data["dimensions"]["yDimension"],
    size_z=data["dimensions"]["zDimension"],
    child_location=Coordinate(location["x"], location["y"], z_offset),
    pedestal_size_z=0,
    model=data["metadata"]["displayName"],
  )
