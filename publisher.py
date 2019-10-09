class Publisher:
    def __init__(self):
        self.consumers = {}

    def publish(self, msg):
        for consumer in self.consumers[msg['sender']]:
            consumer.update(msg)
