import numpy as np
from tinygrad.sunnypilot.modeld_v2.helpers import flat_mv
from tinygrad.sunnypilot.modeld_v2.device import Compiled, Allocator

class NpyAllocator(Allocator):
  def _copyout(self, dest:memoryview, src:np.ndarray): dest[:] = flat_mv(np.require(src, requirements='C').data)

class NpyDevice(Compiled):
  def __init__(self, device:str): super().__init__(device, NpyAllocator(), None, None, None)
