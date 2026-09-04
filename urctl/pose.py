"""Rigid-body pose math in UR's convention — pure stdlib.

A UR pose is ``[x, y, z, rx, ry, rz]``: metres plus an **axis-angle rotation
vector** (direction = axis, length = angle in radians), which is what
``get_actual_tcp_pose()``, ``get_tcp_offset()``, RTDE's ``actual_TCP_pose``
and ``movel`` all use. :class:`Transform` is the same thing as a rotation
matrix + translation, with the two operations URScript's ``pose_trans`` and
``pose_inv`` provide, so host-side code can compose frames (base ← flange ←
camera) without a round-trip to the controller and without numpy.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

Vec3 = tuple[float, float, float]
Mat3 = tuple[Vec3, Vec3, Vec3]  # row-major

IDENTITY: Mat3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
_EPS = 1e-12


def rotvec_to_matrix(rv: Sequence[float]) -> Mat3:
    """Rodrigues: rotation vector → row-major 3x3."""
    rx, ry, rz = (float(v) for v in rv)
    theta = math.sqrt(rx * rx + ry * ry + rz * rz)
    if theta < _EPS:
        return IDENTITY
    kx, ky, kz = rx / theta, ry / theta, rz / theta
    c, s = math.cos(theta), math.sin(theta)
    v = 1.0 - c
    return (
        (c + kx * kx * v, kx * ky * v - kz * s, kx * kz * v + ky * s),
        (ky * kx * v + kz * s, c + ky * ky * v, ky * kz * v - kx * s),
        (kz * kx * v - ky * s, kz * ky * v + kx * s, c + kz * kz * v),
    )


def matrix_to_rotvec(m: Mat3) -> Vec3:
    """Inverse Rodrigues, stable at 0 and π."""
    trace = m[0][0] + m[1][1] + m[2][2]
    cos_theta = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    theta = math.acos(cos_theta)
    if theta < 1e-9:
        return (0.0, 0.0, 0.0)
    if math.pi - theta < 1e-3:
        # Near π the sin-based formula loses precision (sin θ → 0) and so does
        # acos (cos is flat there): take θ from the antisymmetric part instead
        # (|R − Rᵀ|/2 = sin θ) and the axis from the symmetric part
        # (R + I)/2 = k kᵀ, picking the largest diagonal.
        anti = (m[2][1] - m[1][2], m[0][2] - m[2][0], m[1][0] - m[0][1])
        sin_theta = math.sqrt(sum(a * a for a in anti)) / 2.0
        theta = math.pi - math.asin(min(1.0, sin_theta))
        # k kᵀ = ((R + Rᵀ)/2 − cos θ · I) / (1 − cos θ), exact for any θ.
        c = math.cos(theta)
        kk = [
            [((m[i][j] + m[j][i]) / 2.0 - (c if i == j else 0.0)) / (1.0 - c) for j in range(3)]
            for i in range(3)
        ]
        i = max(range(3), key=lambda a: kk[a][a])
        k = [0.0, 0.0, 0.0]
        k[i] = math.sqrt(max(kk[i][i], 0.0))
        for j in range(3):
            if j != i:
                k[j] = kk[i][j] / k[i]
        # Sign is ambiguous at exactly π; pick the one consistent with the
        # antisymmetric part when it isn't zero.
        if sum(a * b for a, b in zip(anti, k, strict=True)) < 0:
            k = [-v for v in k]
        return (k[0] * theta, k[1] * theta, k[2] * theta)
    f = theta / (2.0 * math.sin(theta))
    return (
        (m[2][1] - m[1][2]) * f,
        (m[0][2] - m[2][0]) * f,
        (m[1][0] - m[0][1]) * f,
    )


def _matmul(a: Mat3, b: Mat3) -> Mat3:
    return tuple(  # type: ignore[return-value]
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3)
    )


def _transpose(m: Mat3) -> Mat3:
    return tuple(tuple(m[j][i] for j in range(3)) for i in range(3))  # type: ignore[return-value]


def _matvec(m: Mat3, v: Sequence[float]) -> Vec3:
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


@dataclass(frozen=True)
class Transform:
    """``p_parent = R · p_child + t`` — a frame expressed in its parent."""

    rotation: Mat3 = IDENTITY
    translation: Vec3 = (0.0, 0.0, 0.0)

    @classmethod
    def from_pose(cls, pose: Sequence[float]) -> Transform:
        """From a UR ``[x, y, z, rx, ry, rz]``."""
        if len(pose) != 6:
            raise ValueError(f"pose must have 6 elements, got {len(pose)}")
        x, y, z, rx, ry, rz = (float(v) for v in pose)
        return cls(rotvec_to_matrix((rx, ry, rz)), (x, y, z))

    @classmethod
    def from_axes(
        cls,
        x_axis: Sequence[float],
        y_axis: Sequence[float],
        z_axis: Sequence[float],
        origin: Sequence[float],
    ) -> Transform:
        """From the child's axes and origin expressed in the parent (columns of R)."""
        rotation: Mat3 = tuple(  # type: ignore[assignment]
            (float(x_axis[i]), float(y_axis[i]), float(z_axis[i])) for i in range(3)
        )
        return cls(rotation, (float(origin[0]), float(origin[1]), float(origin[2])))

    def to_pose(self) -> list[float]:
        """As a UR ``[x, y, z, rx, ry, rz]``."""
        return [*self.translation, *matrix_to_rotvec(self.rotation)]

    def apply(self, point: Sequence[float]) -> Vec3:
        """Map a point from the child frame into the parent."""
        r = _matvec(self.rotation, point)
        t = self.translation
        return (r[0] + t[0], r[1] + t[1], r[2] + t[2])

    def rotate(self, direction: Sequence[float]) -> Vec3:
        """Map a direction (no translation) from the child frame into the parent."""
        return _matvec(self.rotation, direction)

    def compose(self, other: Transform) -> Transform:
        """``self · other``: ``other`` expressed in ``self``'s child becomes expressed in
        ``self``'s parent (URScript's ``pose_trans(self, other)``)."""
        return Transform(_matmul(self.rotation, other.rotation), self.apply(other.translation))

    def inverse(self) -> Transform:
        """URScript's ``pose_inv``."""
        rt = _transpose(self.rotation)
        t = _matvec(rt, self.translation)
        return Transform(rt, (-t[0], -t[1], -t[2]))

    def as_dict(self) -> dict:
        return {
            "pose": self.to_pose(),
            "rotation": [list(r) for r in self.rotation],
            "translation": list(self.translation),
        }


def pose_trans(a: Sequence[float], b: Sequence[float]) -> list[float]:
    """URScript ``pose_trans(a, b)`` on plain 6-vectors."""
    return Transform.from_pose(a).compose(Transform.from_pose(b)).to_pose()


def pose_inv(a: Sequence[float]) -> list[float]:
    """URScript ``pose_inv(a)`` on a plain 6-vector."""
    return Transform.from_pose(a).inverse().to_pose()
