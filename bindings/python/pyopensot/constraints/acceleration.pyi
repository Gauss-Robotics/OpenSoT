from typing import Literal, overload, TypeVar

import numpy
import pyopensot

M = TypeVar("M", bound=int)


class JointLimits(pyopensot.Constraint):
    def __init__(self, arg0, arg1: pyopensot.AffineHelper,
                 arg2: numpy.ndarray[tuple[M, Literal[1]], numpy.dtype[numpy.float64]],
                 arg3: numpy.ndarray[tuple[M, Literal[1]], numpy.dtype[numpy.float64]],
                 arg4: numpy.ndarray[tuple[M, Literal[1]], numpy.dtype[numpy.float64]],
                 arg5: float) -> None: ...

    def setJointAccMax(self, arg0: numpy.ndarray[
        tuple[M, Literal[1]], numpy.dtype[numpy.float64]]) -> None: ...

    def setPStepAheadPredictor(self, arg0: float) -> bool: ...

    def update(self) -> None: ...


class TorqueLimits(pyopensot.Constraint):
    def __init__(self, arg0, arg1: pyopensot.AffineHelper, arg2: list[pyopensot.AffineHelper],
                 arg3: list[str],
                 arg4: numpy.ndarray[tuple[M, Literal[1]], numpy.dtype[numpy.float64]]) -> None: ...

    def disableContact(self, arg0: str) -> bool: ...

    def enableContact(self, arg0: str) -> bool: ...

    def getEnabledContacts(self) -> list[bool]: ...

    def setTorqueLimits(self, arg0: numpy.ndarray[
        tuple[M, Literal[1]], numpy.dtype[numpy.float64]]) -> None: ...

    def update(self) -> None: ...


class VelocityLimits(pyopensot.Constraint):
    @overload
    def __init__(self, arg0, arg1: pyopensot.AffineHelper, arg2: float, arg3: float) -> None: ...

    @overload
    def __init__(self, arg0, arg1: pyopensot.AffineHelper,
                 arg2: numpy.ndarray[tuple[M, Literal[1]], numpy.dtype[numpy.float64]],
                 arg3: float) -> None: ...

    def setPStepAheadPredictor(self, arg0: float) -> bool: ...

    @overload
    def setVelocityLimits(self, arg0: float) -> None: ...

    @overload
    def setVelocityLimits(self, arg0: numpy.ndarray[
        tuple[M, Literal[1]], numpy.dtype[numpy.float64]]) -> None: ...

    def update(self) -> None: ...
