import os
if int(os.getenv("TYPED", "0")):
  from typeguard import install_import_hook
  install_import_hook(__name__)
from tinygrad.sunnypilot.modeld_v2.tensor import Tensor                                    # noqa: F401
from tinygrad.sunnypilot.modeld_v2.engine.jit import TinyJit                               # noqa: F401
from tinygrad.sunnypilot.modeld_v2.ops import UOp
Variable = UOp.variable
from tinygrad.sunnypilot.modeld_v2.dtype import dtypes                                     # noqa: F401
from tinygrad.sunnypilot.modeld_v2.helpers import GlobalCounters, fetch, Context, getenv   # noqa: F401
from tinygrad.sunnypilot.modeld_v2.device import Device                                    # noqa: F401
