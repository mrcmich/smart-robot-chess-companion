class SimpleInteger():
    def __init__(self, value):
        self.value = int(value)
    
    def get_int(self):
        return self.value
    
    def increment(self):
        self.value += 1

n = SimpleInteger(0)
n.get_int()
n