from enum import auto

from srb.core.algorithms import LowercaseNameEnum


class Domain(LowercaseNameEnum):
    ASTEROID = auto()
    EARTH = auto()
    MARS = auto()
    MOON = auto()
    ORBIT = auto()

    @property
    def gravity_magnitude(self) -> float:
        """Representative gravitational acceleration magnitude in m/s²."""
        match self:
            case Domain.ASTEROID:
                return 0.14219
            case Domain.EARTH:
                return 9.80665
            case Domain.MARS:
                return 3.72076
            case Domain.MOON:
                return 1.62496
            case Domain.ORBIT:
                return 0.0

    @property
    def gravity_variation(self) -> float:
        """Difference between maximum and minimum gravity in m/s²."""
        match self:
            case Domain.ASTEROID:
                return 0.28438
            case Domain.EARTH:
                return 0.0698
            case Domain.MARS:
                return 0.0279
            case Domain.MOON:
                return 0.0253
            case Domain.ORBIT:
                return 0.0

    @property
    def gravity_range(self) -> tuple[float, float]:
        """Gravity range in m/s², centered on the magnitude."""
        magnitude = self.gravity_magnitude
        delta = self.gravity_variation / 2.0
        return (magnitude - delta, magnitude + delta)

    @property
    def light_intensity(self) -> float:
        """Representative solar irradiance in W/m²."""
        match self:
            case Domain.ASTEROID:
                return 190.0
            case Domain.EARTH:
                return 775
            case Domain.MARS:
                return 590
            case Domain.MOON | Domain.ORBIT:
                return 1361.0

    @property
    def light_intensity_variation(self) -> float:
        """Difference between maximum and minimum solar irradiance in W/m²."""
        match self:
            case Domain.ASTEROID:
                return 50.0
            case Domain.EARTH:
                return 450.0
            case Domain.MARS:
                return 226.0
            case Domain.MOON | Domain.ORBIT:
                return 0.5

    @property
    def light_intensity_range(self) -> tuple[float, float]:
        """Solar irradiance range in W/m², centered on the intensity."""
        intensity = self.light_intensity
        delta = self.light_intensity_variation / 2.0
        return (intensity - delta, intensity + delta)

    @property
    def light_angular_diameter(self) -> float:
        """Solar angular diameter in degrees at each domain's Sun distance.

        The Sun subtends about 0.533° at 1 AU; diameter scales inversely with
        distance and is independent of atmospheric scatter.
        """
        match self:
            case Domain.EARTH | Domain.MOON | Domain.ORBIT:
                return 0.53
            case Domain.MARS:
                return 0.35
            case Domain.ASTEROID:
                return 0.20

    @property
    def light_angular_diameter_variation(self) -> float:
        """Variation in solar angular diameter in degrees."""
        match self:
            case Domain.EARTH | Domain.MOON | Domain.ORBIT:
                return 0.021
            case Domain.MARS:
                return 0.08
            case Domain.ASTEROID:
                return 0.029

    @property
    def light_angular_diameter_range(self) -> tuple[float, float]:
        """Solar angular-diameter range in degrees, centered on the diameter."""
        diameter = self.light_angular_diameter
        delta = self.light_angular_diameter_variation / 2.0
        return (diameter - delta, diameter + delta)

    @property
    def light_color_temperature(self) -> float:
        """Representative solar light-source color temperature in kelvin."""
        match self:
            case Domain.EARTH:
                return 5750.0
            case Domain.ASTEROID | Domain.MARS | Domain.MOON | Domain.ORBIT:
                return 5778.0

    @property
    def light_color_temperature_variation(self) -> float:
        """Variation in solar light-source color temperature in kelvin."""
        match self:
            case Domain.EARTH:
                return 1500.0
            case Domain.MARS:
                return 500.0
            case Domain.ASTEROID | Domain.MOON | Domain.ORBIT:
                return 0.0

    @property
    def light_color_temperature_range(self) -> tuple[float, float]:
        """Color-temperature range in kelvin, centered on the temperature."""

        temperature = self.light_color_temperature
        delta = self.light_color_temperature_variation / 2.0
        return (temperature - delta, temperature + delta)
