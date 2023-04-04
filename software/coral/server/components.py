from abc import ABC
from dataclasses import dataclass
from typing import List, Tuple, Union

from camera import DebugView


class BaseSlider(ABC):
    """Base slider."""

    html_slider_id: str

    def __init__(self, id_: str) -> None:
        """Initialize the slider's HTML element id."""
        self.html_slider_id = f"slider-{id_}"

    def to_props(self) -> dict:
        """Get the slider's props."""
        raise NotImplementedError


@dataclass
class SingleSlider(BaseSlider):
    """Single slider."""

    id_: str
    min_: int
    max_: int
    step: int
    value: int

    def __post_init__(self) -> None:
        """Initialize the slider's id."""
        super().__init__(id_=self.id_)

    def to_props(self) -> dict:
        """Get the slider's props."""
        return {
            "type": self.__class__.__name__,
            "label": self.id_.replace("-", " ").title(),
            "min": self.min_,
            "max": self.max_,
            "step": self.step,
            "value": self.value,
        }


@dataclass
class XYSlider(BaseSlider):
    """XY slider."""

    id_: str
    min_: Tuple[int, int]
    max_: Tuple[int, int]
    step: Tuple[int, int]
    value: Tuple[int, int]

    def __post_init__(self) -> None:
        """Initialize the slider's id."""
        super().__init__(id_=self.id_)

    def to_props(self) -> dict:
        """Get the slider's props."""
        return {
            "type": self.__class__.__name__,
            "label": self.id_.replace("-", " ").title(),
            "min": self.min_,
            "max": self.max_,
            "step": self.step,
            "value": self.value,
        }


@dataclass
class HSVSlider(BaseSlider):
    """HSV slider."""

    id_: str
    value: Tuple[int, int, int]

    def __post_init__(self) -> None:
        """Initialize the slider's id."""
        super().__init__(id_=self.id_)

    def to_props(self) -> dict:
        """Get the slider's props."""
        return {
            "type": self.__class__.__name__,
            "label": self.id_.replace("-", " ").title(),
            "value": self.value,
        }


@dataclass
class Subfeed:
    """Subfeed."""

    value: DebugView

    def to_props(self) -> dict:
        """Get the subfeed's props."""

        return {
            "type": self.__class__.__name__,
            "options": [
                view.get_name() for view in DebugView if view != DebugView.DEFAULT
            ],
            "value": self.value.get_name(),
        }


class AppState:
    """Singleton for managing the app."""

    _sliders: List[BaseSlider]
    _subfeeds: List[Subfeed]

    def __init__(
        self, sliders: List[BaseSlider] = [], subfeed_views: List[DebugView] = []
    ) -> None:
        """Initialize the app state."""
        self._sliders = sliders
        self._subfeeds = [Subfeed(view) for view in subfeed_views]

    def get_slider_value(
        self, slider_id: str
    ) -> Union[int, Tuple[int, int], Tuple[int, int, int]]:
        """Get a slider's value."""
        for slider in self._sliders:
            if slider.id_ == slider_id:
                return slider.value

    def get_subfeed_views(self) -> List[DebugView]:
        """Get the subfeed view states."""
        return [subfeed.value for subfeed in self._subfeeds]

    def set_slider_value(self, html_slider_id: str, value: int) -> None:
        """Set a slider's value."""
        for slider in self._sliders:
            # SingleSlider
            if slider.html_slider_id == html_slider_id:
                slider.value = value
            # XYSlider, HSVSlider
            elif slider.html_slider_id == html_slider_id[:-2]:
                # XYSlider
                if html_slider_id.endswith("-x"):
                    slider.value = (value, slider.value[1])
                elif html_slider_id.endswith("-y"):
                    slider.value = (slider.value[0], value)
                # HSVSlider
                elif html_slider_id.endswith("-h"):
                    slider.value = (value, slider.value[1], slider.value[2])
                elif html_slider_id.endswith("-s"):
                    slider.value = (slider.value[0], value, slider.value[2])
                elif html_slider_id.endswith("-v"):
                    slider.value = (slider.value[0], slider.value[1], value)

    def set_subfeed_value(self, html_subfeed_id: int, value: str) -> None:
        """Set a subfeed's value."""
        self._subfeeds[int(html_subfeed_id.split("-")[-1])].value = DebugView.from_name(
            value
        )

    def add_sliders(self, sliders: List[BaseSlider]) -> None:
        """Add sliders."""
        self._sliders.extend(sliders)

    def get_slider_props(self) -> dict:
        """Get a dict of sliders."""
        return {slider.html_slider_id: slider.to_props() for slider in self._sliders}

    def get_subfeed_props(self) -> dict:
        """Get a dict of subfeeds."""
        return {
            f"subfeed-{i}": subfeed.to_props()
            for i, subfeed in enumerate(self._subfeeds)
        }
