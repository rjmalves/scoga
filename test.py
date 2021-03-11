import pika
import datetime
from PikaBus.abstractions.AbstractPikaBus import AbstractPikaBus
from PikaBus.PikaBusSetup import PikaBusSetup


def MessageHandlerMethod(**kwargs):
    """
    A message handler method may simply be a method with som **kwargs.
    The **kwargs will be given all incoming pipeline data, the bus and the incoming payload.
    """
    data: dict = kwargs['data']
    bus: AbstractPikaBus = kwargs['bus']
    payload: dict = kwargs['payload']
    print(payload)
    if payload['reply']:
        payload['reply'] = False
        bus.Reply(payload=payload)

# Use pika connection params to set connection details
credentials = pika.PlainCredentials('guest', 'guest')
connParams = pika.ConnectionParameters(host='localhost')

# Create a PikaBusSetup instance with a listener queue, and add the message handler method.
pikaBusSetup = PikaBusSetup(connParams,
                            defaultListenerQueue='myQueue',
                            defaultSubscriptions='myTopic')
pikaBusSetup.AddMessageHandler(MessageHandlerMethod)

# Start consuming messages from the queue.
pikaBusSetup.StartConsumers()

# Create a temporary bus to subscribe on topics and send, defer or publish messages.
bus = pikaBusSetup.CreateBus()
bus.Subscribe('myTopic')
payload = {'hello': 'world!', 'reply': True}

# To send a message means sending a message explicitly to one receiver.
bus.Send(payload=payload, queue='myQueue')

# To defer a message means sending a message explicitly to one receiver with some delay before it is processed.
bus.Defer(payload=payload, delay=datetime.timedelta(seconds=1), queue='myQueue')

# To publish a message means publishing a message on a topic received by any subscribers of the topic.
bus.Publish(payload=payload, topic='myTopic')

input('Hit enter to stop all consuming channels \n\n')

print("OUVI o enter")
pikaBusSetup.StopConsumers()
print("parou CONSUMERS")
pikaBusSetup.Stop()
print("parou BUS")
