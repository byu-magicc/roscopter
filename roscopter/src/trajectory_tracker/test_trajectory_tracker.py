import unittest
from trajectory_tracker import TrajectoryTracker
import numpy as np

class TestTrajectoryTracker(unittest.TestCase):

    def setUp(self):
        self.trajectory_follower = TrajectoryTracker()

    def test_quaternionToSO3(self):
        q = np.array([0.7071, 0.7071 , 0 ,0])
        R = self.trajectory_follower.quaternionToSO3(q)
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

    def test_computeLinearError(self):
        state = np.array([[1],[3.4],[5.46]])
        desired_state = np.array([[12],[5],[-2]])
        error = self.trajectory_follower.computeLinearError(state,desired_state)
        error_key = np.array([[-11],[-1.6],[7.46]])
        self.assertTrue(np.array_equal(error, error_key))

    def test_computeDesiredForceVector(self):
         position_error = np.array([[0],[0],[0]])
         velocity_error = np.array([[0],[0],[0]])
         gravity = 9.8
         mass = 3.69
         self.trajectory_follower.desired_acceleration = np.array([[0],[0],[0]])
         desired_force_vector = self.trajectory_follower.computeDesiredForceVector(position_error , velocity_error)
         desired_force_vector_key = np.array([[0],[0],[-mass*gravity]])
         r_tol= 0
         a_tol=1e-4
         self.assertTrue(np.allclose(desired_force_vector,desired_force_vector_key,rtol=r_tol,atol=a_tol))



if __name__ == '__main__':
    unittest.main()

