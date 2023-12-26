from typing import Any

from avstack.config import ALGORITHMS, MODELS


class _Wrapper:
    def __init__(self, model, ID, ID_input) -> None:
        self.model = model
        self.ID = ID
        self.ID_input = ID_input

    def __call__(self, *args: Any, **kwds: Any) -> Any:
        return self.model(*args, **kwds)

    def __getattr__(self, name: str) -> Any:
        return getattr(self.model, name)


@MODELS.register_module()
class SensorWrapper(_Wrapper):
    def __init__(self, model, ID, **kwargs) -> None:
        model = MODELS.build(model, default_args=kwargs)
        super().__init__(model, ID, ID_input=[])


@ALGORITHMS.register_module()
class PerceptionWrapper(_Wrapper):
    def __init__(self, algorithm, ID, ID_input, **kwargs) -> None:
        algorithm = ALGORITHMS.build(algorithm, default_args=kwargs)
        super().__init__(algorithm, ID, ID_input)


@ALGORITHMS.register_module()
class TrackingWrapper(_Wrapper):
    def __init__(self, algorithm, ID, ID_input, **kwargs) -> None:
        algorithm = ALGORITHMS.build(algorithm, default_args=kwargs)
        super().__init__(algorithm, ID, ID_input)


@ALGORITHMS.register_module()
class ClusteringWrapper(_Wrapper):
    def __init__(self, algorithm, ID, ID_input, **kwargs) -> None:
        algorithm = ALGORITHMS.build(algorithm, default_args=kwargs)
        super().__init__(algorithm, ID, ID_input)
