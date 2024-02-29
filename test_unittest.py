import unittest
import importlib
import importnb
import numpy as np
from importnb import imports
#from importnb import Notebook, get_ipython, imports


from unittest.mock import patch

import cvxopt
from cvxopt import matrix, printing

from robot3D_basic_solution import *


class TestRobotArm(unittest.TestCase):
	
	def test_forward_kinematics1(self):
		
		# Lentghs of the parts
		L1, L2, L3, L4 = [5, 8, 3, 0]
		Phi = np.array([30, -50, -30, 0])
		T_01, T_02, T_03, T_04, e = forward_kinematics(Phi, L1, L2, L3, L4)
		
		actual = e
		expected = np.array([18.47772028, -0.71432837,  0. ])

		assert np.allclose(expected, actual)

	def test_forward_kinematics2(self):
		
		# Lentghs of the parts
		L1, L2, L3, L4 = [5, 8, 3, 0]
		Phi = np.array([0, 0, 0, 0])
		T_01, T_02, T_03, T_04, e = forward_kinematics(Phi, L1, L2, L3, L4)
		
		actual = e
		expected = np.array([21, 2,  0. ])

		assert np.allclose(expected, actual)

	def test_forward_kinematics3(self):
		
		# Lentghs of the parts
		L1, L2, L3, L4 = [5, 8, 3, 0]
		Phi = np.array([-30, 50, 30, 0])
		T_01, T_02, T_03, T_04, e = forward_kinematics(Phi, L1, L2, L3, L4)
		
		actual = e
		expected = np.array([18.47772028,  4.71432837,  0. ])

		assert np.allclose(expected, actual)



if __name__ == '__main__':
	unittest.main()



