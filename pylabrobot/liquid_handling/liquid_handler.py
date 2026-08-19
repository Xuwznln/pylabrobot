import warnings

warnings.warn(
  "Importing from pylabrobot.liquid_handling.liquid_handler is deprecated. "
  "Use pylabrobot.legacy.liquid_handling.liquid_handler instead.",
  DeprecationWarning,
  stacklevel=2,
)

from pylabrobot.legacy.liquid_handling.liquid_handler import *  # noqa: F401,F403,E402
