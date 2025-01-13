from typing import Literal, overload, TypeVar

import numpy
import pyopensot

M = TypeVar("M", bound=int)


class CoP(pyopensot.Constraint):
    def __init__(self, arg0: str, arg1: pyopensot.AffineHelper, arg2,
                 arg3: numpy.ndarray[tuple[Literal[2], Literal[1]], numpy.dtype[numpy.float64]],
                 arg4: numpy.ndarray[
                     tuple[Literal[2], Literal[1]], numpy.dtype[numpy.float64]]) -> None: ...

    def update(self) -> None: ...


class CoPs(pyopensot.Constraint):
    def __init__(self, arg0: list[pyopensot.AffineHelper], arg1: list[str], arg2, arg3: list[
        numpy.ndarray[tuple[Literal[2], Literal[1]], numpy.dtype[numpy.float64]]], arg4: list[
        numpy.ndarray[tuple[Literal[2], Literal[1]], numpy.dtype[numpy.float64]]]) -> None: ...

    def update(self) -> None: ...


class FrictionCone(pyopensot.Constraint):
    def __init__(self, arg0: str, arg1: pyopensot.AffineHelper, arg2, arg3: tuple[numpy.ndarray[
        tuple[Literal[3], Literal[3]], numpy.dtype[numpy.float64]], float]) -> None: ...

    def setContactRotationMatrix(self, arg0: numpy.ndarray[
        tuple[Literal[3], Literal[3]], numpy.dtype[numpy.float64]]) -> None: ...

    def setFrictionCone(self, arg0: tuple[numpy.ndarray[
        tuple[Literal[3], Literal[3]], numpy.dtype[numpy.float64]], float]) -> None: ...

    def setMu(self, arg0: float) -> None: ...

    def update(self) -> None: ...


class FrictionCones(pyopensot.Constraint):
    def __init__(self, arg0: list[str], arg1: list[pyopensot.AffineHelper], arg2, arg3: list[tuple[
        numpy.ndarray[
            tuple[Literal[3], Literal[3]], numpy.dtype[numpy.float64]], float]]) -> None: ...

    def getFrictionCone(self, arg0: str) -> FrictionCone: ...

    def update(self) -> None: ...


class NormalTorque(pyopensot.Constraint):
    def __init__(self, arg0: str, arg1: pyopensot.AffineHelper, arg2,
                 arg3: numpy.ndarray[tuple[Literal[2], Literal[1]], numpy.dtype[numpy.float64]],
                 arg4: numpy.ndarray[tuple[Literal[2], Literal[1]], numpy.dtype[numpy.float64]],
                 arg5: float) -> None: ...

    def setMu(self, arg0: float) -> None: ...

    def update(self) -> None: ...


class NormalTorques(pyopensot.Constraint):
    def __init__(self, arg0: list[str], arg1: list[pyopensot.AffineHelper], arg2, arg3: list[
        numpy.ndarray[tuple[Literal[2], Literal[1]], numpy.dtype[numpy.float64]]], arg4: list[
        numpy.ndarray[tuple[Literal[2], Literal[1]], numpy.dtype[numpy.float64]]],
                 arg5: list[float]) -> None: ...

    def getNormalTorque(self, arg0: str) -> NormalTorque: ...

    def update(self) -> None: ...


class WrenchLimits(pyopensot.Constraint):
    def __init__(self, arg0: str,
                 arg1: numpy.ndarray[tuple[M, Literal[1]], numpy.dtype[numpy.float64]],
                 arg2: numpy.ndarray[tuple[M, Literal[1]], numpy.dtype[numpy.float64]],
                 arg3: pyopensot.AffineHelper) -> None: ...

    def getWrenchLimits(self) -> tuple[
        numpy.ndarray[tuple[M, Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[
            tuple[M, Literal[1]], numpy.dtype[numpy.float64]]]: ...

    def isReleased(self) -> bool: ...

    def releaseContact(self, arg0: bool) -> None: ...

    def setWrenchLimits(self, arg0: numpy.ndarray[tuple[M, Literal[1]], numpy.dtype[numpy.float64]],
                        arg1: numpy.ndarray[
                            tuple[M, Literal[1]], numpy.dtype[numpy.float64]]) -> None: ...


class WrenchesLimits(pyopensot.Constraint):
    @overload
    def __init__(self, arg0: list[str],
                 arg1: numpy.ndarray[tuple[M, Literal[1]], numpy.dtype[numpy.float64]],
                 arg2: numpy.ndarray[tuple[M, Literal[1]], numpy.dtype[numpy.float64]],
                 arg3: list[pyopensot.AffineHelper]) -> None: ...

    @overload
    def __init__(self, arg0: list[str],
                 arg1: list[numpy.ndarray[tuple[M, Literal[1]], numpy.dtype[numpy.float64]]],
                 arg2: list[numpy.ndarray[tuple[M, Literal[1]], numpy.dtype[numpy.float64]]],
                 arg3: list[pyopensot.AffineHelper]) -> None: ...

    @overload
    def __init__(self, arg0: dict[str, WrenchLimits],
                 arg1: list[pyopensot.AffineHelper]) -> None: ...

    def getWrenchLimits(self, arg0: str) -> WrenchLimits: ...

    def update(self) -> None: ...
