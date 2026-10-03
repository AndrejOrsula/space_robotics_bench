from pydantic import BaseModel


class Transform(BaseModel):
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
