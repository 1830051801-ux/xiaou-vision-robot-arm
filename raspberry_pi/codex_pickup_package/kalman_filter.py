from __future__ import annotations


class OneDKalman:
    def __init__(self, q: float = 1e-3, r: float = 1e-2) -> None:
        self.q = q
        self.r = r
        self.x: float | None = None
        self.p = 1.0

    def update(self, z: float) -> float:
        if self.x is None:
            self.x = float(z)
            return self.x
        self.p += self.q
        k = self.p / (self.p + self.r)
        self.x = self.x + k * (float(z) - self.x)
        self.p = (1 - k) * self.p
        return self.x


if __name__ == "__main__":
    f = OneDKalman()
    print([round(f.update(v), 3) for v in [1, 1.2, 0.9, 1.1, 10, 1.0]])
