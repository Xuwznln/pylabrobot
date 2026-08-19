import warnings

warnings.warn(
  "Importing from pylabrobot.liquid_handling.backends.backend is deprecated. "
  "Use pylabrobot.legacy.liquid_handling.backends.backend instead.",
  DeprecationWarning,
  stacklevel=2,
)

from pylabrobot.legacy.liquid_handling.backends.backend import *  # noqa: F401,F403,E402
