from __future__ import annotations

import numpy as np

try:
    import tf_transformations as _tf
except ModuleNotFoundError:
    from transforms3d.euler import euler2quat, quat2euler
    from transforms3d.quaternions import mat2quat, qmult, quat2mat

    def _xyzw_to_wxyz(quaternion):
        x, y, z, w = quaternion
        return np.array([w, x, y, z], dtype=np.float64)

    def _wxyz_to_xyzw(quaternion):
        w, x, y, z = quaternion
        return np.array([x, y, z, w], dtype=np.float64)

    class _TfCompat:
        @staticmethod
        def quaternion_from_euler(roll, pitch, yaw):
            return _wxyz_to_xyzw(euler2quat(roll, pitch, yaw))

        @staticmethod
        def euler_from_quaternion(quaternion):
            return quat2euler(_xyzw_to_wxyz(quaternion))

        @staticmethod
        def quaternion_multiply(q1, q2):
            return _wxyz_to_xyzw(qmult(_xyzw_to_wxyz(q1), _xyzw_to_wxyz(q2)))

        @staticmethod
        def quaternion_matrix(quaternion):
            matrix = np.eye(4, dtype=np.float64)
            matrix[:3, :3] = quat2mat(_xyzw_to_wxyz(quaternion))
            return matrix

        @staticmethod
        def quaternion_from_matrix(matrix):
            return _wxyz_to_xyzw(mat2quat(np.asarray(matrix, dtype=np.float64)[:3, :3]))

        @staticmethod
        def inverse_matrix(matrix):
            return np.linalg.inv(np.asarray(matrix, dtype=np.float64))

        @staticmethod
        def concatenate_matrices(*matrices):
            result = np.eye(4, dtype=np.float64)
            for matrix in matrices:
                result = result @ np.asarray(matrix, dtype=np.float64)
            return result

    _tf = _TfCompat()

tf_transformations = _tf
