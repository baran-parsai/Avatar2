from abc import ABC, abstractmethod
class LLM(ABC):
    @abstractmethod
    def response(self, text):
        pass
