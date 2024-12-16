class BaseAgent:
    def __init__(self, id):
        self.id = id

    def decide_action(self, env):
        raise NotImplementedError("Subclasses must implement this method")

class RandomAgent(BaseAgent):
    def decide_action(self, env):
        return "random_move"
