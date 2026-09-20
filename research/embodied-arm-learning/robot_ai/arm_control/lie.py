from __future__ import annotations

import math

import numpy as np


_EPS = 1e-12


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(vector, dtype=np.float64).reshape(3)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=np.float64)


def unskew(matrix: np.ndarray) -> np.ndarray:
    matrix = np.asarray(matrix, dtype=np.float64)
    return np.array([matrix[2, 1], matrix[0, 2], matrix[1, 0]], dtype=np.float64)


def vec_to_se3(twist: np.ndarray) -> np.ndarray:
    twist = np.asarray(twist, dtype=np.float64).reshape(6)
    result = np.zeros((4, 4), dtype=np.float64)
    result[:3, :3] = skew(twist[:3])
    result[:3, 3] = twist[3:]
    return result


def se3_to_vec(matrix: np.ndarray) -> np.ndarray:
    matrix = np.asarray(matrix, dtype=np.float64)
    return np.concatenate((unskew(matrix[:3, :3]), matrix[:3, 3]))


def matrix_exp3(so3_matrix: np.ndarray) -> np.ndarray:
    omega_theta = unskew(so3_matrix)
    theta = float(np.linalg.norm(omega_theta))
    if theta < _EPS:
        return np.eye(3, dtype=np.float64)
    omega_hat = np.asarray(so3_matrix, dtype=np.float64) / theta
    return np.eye(3) + math.sin(theta) * omega_hat + (1.0 - math.cos(theta)) * (omega_hat @ omega_hat)


def matrix_log3(rotation: np.ndarray) -> np.ndarray:
    rotation = np.asarray(rotation, dtype=np.float64)
    acos_input = float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
    if acos_input >= 1.0 - _EPS:
        return np.zeros((3, 3), dtype=np.float64)
    if acos_input <= -1.0 + _EPS:
        if 1.0 + rotation[2, 2] > _EPS:
            omega = np.array([rotation[0, 2], rotation[1, 2], 1.0 + rotation[2, 2]])
            omega /= math.sqrt(2.0 * (1.0 + rotation[2, 2]))
        elif 1.0 + rotation[1, 1] > _EPS:
            omega = np.array([rotation[0, 1], 1.0 + rotation[1, 1], rotation[2, 1]])
            omega /= math.sqrt(2.0 * (1.0 + rotation[1, 1]))
        else:
            omega = np.array([1.0 + rotation[0, 0], rotation[1, 0], rotation[2, 0]])
            omega /= math.sqrt(2.0 * (1.0 + rotation[0, 0]))
        return skew(math.pi * omega)
    theta = math.acos(acos_input)
    return theta * (rotation - rotation.T) / (2.0 * math.sin(theta))


def transform_inverse(transform: np.ndarray) -> np.ndarray:
    transform = np.asarray(transform, dtype=np.float64)
    rotation = transform[:3, :3]
    position = transform[:3, 3]
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation.T
    result[:3, 3] = -(rotation.T @ position)
    return result


def matrix_exp6(se3_matrix: np.ndarray) -> np.ndarray:
    se3_matrix = np.asarray(se3_matrix, dtype=np.float64)
    omega_theta = unskew(se3_matrix[:3, :3])
    theta = float(np.linalg.norm(omega_theta))
    result = np.eye(4, dtype=np.float64)
    if theta < _EPS:
        result[:3, 3] = se3_matrix[:3, 3]
        return result
    omega_hat = se3_matrix[:3, :3] / theta
    result[:3, :3] = matrix_exp3(se3_matrix[:3, :3])
    g_matrix = (
        np.eye(3) * theta
        + (1.0 - math.cos(theta)) * omega_hat
        + (theta - math.sin(theta)) * (omega_hat @ omega_hat)
    )
    result[:3, 3] = g_matrix @ (se3_matrix[:3, 3] / theta)
    return result


def matrix_log6(transform: np.ndarray) -> np.ndarray:
    transform = np.asarray(transform, dtype=np.float64)
    rotation = transform[:3, :3]
    position = transform[:3, 3]
    omega_matrix = matrix_log3(rotation)
    result = np.zeros((4, 4), dtype=np.float64)
    if np.linalg.norm(omega_matrix) < _EPS:
        result[:3, 3] = position
        return result
    theta = math.acos(float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)))
    result[:3, :3] = omega_matrix
    result[:3, 3] = (
        np.eye(3)
        - omega_matrix / 2.0
        + (1.0 / theta - 1.0 / (2.0 * math.tan(theta / 2.0)))
        * (omega_matrix @ omega_matrix)
        / theta
    ) @ position
    return result


def adjoint(transform: np.ndarray) -> np.ndarray:
    transform = np.asarray(transform, dtype=np.float64)
    rotation = transform[:3, :3]
    position = transform[:3, 3]
    result = np.zeros((6, 6), dtype=np.float64)
    result[:3, :3] = rotation
    result[3:, 3:] = rotation
    result[3:, :3] = skew(position) @ rotation
    return result
