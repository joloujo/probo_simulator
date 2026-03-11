from math import pi
import numpy as np
import pytest

from probo_sim.graph_slam import GraphSLAM, OdomFactor

class TestGraphSLAM:

    def test_OdomFactor(self):

        assert OdomFactor(np.array([1, 0, 0])).cost(np.array([0, 0, 0, 1, 0, 0])) == pytest.approx(0)
        assert OdomFactor(np.array([1, 0, 0])).cost(np.array([0, 0, pi/2, 0, 1, pi/2])) == pytest.approx(0)

        assert OdomFactor(np.array([1, 1, pi/2])).cost(np.array([0, 0, 0, 1, 1, pi/2])) == pytest.approx(0)
        assert OdomFactor(np.array([1, 1, pi/2])).cost(np.array([-1, -2, pi, -2, -3, -pi/2])) == pytest.approx(0)

        assert OdomFactor(np.array([0, 0, 0])).cost(np.array([0, 0, 0, 2, 0, 0])) == pytest.approx(4)
        assert OdomFactor(np.array([0, 0, 0])).cost(np.array([0, 0, 0, 0, 2, 0])) == pytest.approx(4)
        assert OdomFactor(np.array([0, 0, 0])).cost(np.array([0, 0, 0, 0, 0, 2])) == pytest.approx(4)

