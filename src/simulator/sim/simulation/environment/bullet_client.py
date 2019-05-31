# Taken from pybullet: gym/pybullet_envs/bullet/bullet_client.py

import functools
import inspect
import pybullet
from .robot_models import Husky

class BulletClient(object):
  """A wrapper for pybullet to manage different clients."""

  def __init__(self, connection_mode=None, options=""):
    """Creates a Bullet client and connects to a simulation.

    Args:
      connection_mode:
        `None` connects to an existing simulation or, if fails, creates a
          new headless simulation,
        `pybullet.GUI` creates a new simulation with a GUI,
        `pybullet.DIRECT` creates a headless simulation,
        `pybullet.SHARED_MEMORY` connects to an existing simulation.
    """
    self._shapes = {}
    self._client = pybullet.connect(pybullet.DIRECT)
    return

    print("Creating new pybullet env with connection_mode = %s"%connection_mode)
    if connection_mode is None:
      self._client = -1#pybullet.connect(pybullet.SHARED_MEMORY)
      if self._client >= 0:
        print("PyBullet using shared memory")
        return
      else:
        print("Shared memory failed")
        print("Conn mode:", connection_mode, options)
        connection_mode = pybullet.DIRECT
    self._client = pybullet.connect(connection_mode, options=options)

  def __del__(self):
    """Clean up connection if not already done."""
    try:
      pybullet.disconnect(physicsClientId=self._client)
    except pybullet.error:
      pass

  def __getattr__(self, name):
    """Inject the client id into Bullet functions."""
    attribute = getattr(pybullet, name)
    if inspect.isbuiltin(attribute):
        attribute = functools.partial(attribute, physicsClientId=self._client)
    return attribute
