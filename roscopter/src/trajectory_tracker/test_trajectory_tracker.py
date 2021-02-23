import unittest
from trajectory_tracker import TrajectoryTracker
import numpy as np

class TestTrajectoryTracker(unittest.TestCase):

    def setUp(self):
        self.trajectory_follower = TrajectoryTracker()

    def test_quaternionToSO3(self):
        R = self.trajectory_follower.quaternionToSO3(0.7071, 0.7071 , 0 ,0)
        R_key =  np.array([[1.0000    ,     0     ,    0],
                            [0  ,  -0.0000  , -1.0000],
                            [0  ,  1.0000   , -0.0000]])
        r_tol= 0
        a_tol=1e-4
        self.assertTrue(np.allclose(R,R_key,rtol=r_tol,atol=a_tol))

    def test_finiteDifferencing(self):
        dt = self.trajectory_follower.finiteDifferencing(23.641797,24,0.01)
        dt_key =  36
        r_tol= 0
        a_tol=1
        self.assertTrue(np.isclose(dt,dt_key,rtol=r_tol,atol=a_tol))


    def test_veeOperator(self):
        skewSymmetricMatrix = np.array([[0 , -3 , 2],
                                        [3, 0 , -1],
                                        [-2 , 1 , 0]])
        v = self.trajectory_follower.veeOperator(skewSymmetricMatrix)
        v_key = np.array([[1],[2],[3]])
        r_tol= 0
        a_tol=1e-4
        self.assertTrue(np.allclose(v,v_key,rtol=r_tol,atol=a_tol))

    # def test_isupper(self):
        # self.assertEqual('FOO'.isupper())
        # self.assertFalse('Foo'.isupper())

if __name__ == '__main__':
    unittest.main()

