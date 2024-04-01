from typing import Iterable, overload, Generic, List, Callable, TypeVar, Union, Iterator
from sortedcontainers import SortedKeyList


T = TypeVar('T')

class SortedBuffer(SortedKeyList, Generic[T]):
    def __init__(self, key: Callable[[T], float], iterable: Union[Iterable[T], None] = None, capacity: int = 50):
        super().__init__(iterable, key)
        self.capacity = capacity
    
    def add(self, value: T):
        super().add(value)
        if len(self) > self.capacity:
            self.pop(0)
    
    # https://stackoverflow.com/questions/71182977/what-is-the-type-annotation-for-return-value-of-getitem
    @overload
    def __getitem__(self, index: slice) -> List[T]: ...
    
    @overload
    def __getitem__(self, index: int) -> T: ...
    
    def __getitem__(self, index: Union[slice, int]) -> Union[T, List[T]]:
        return super().__getitem__(index)
    
    def __iter__(self) -> Iterator[T]:
        return super().__iter__()