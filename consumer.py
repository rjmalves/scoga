#!/usr/bin/env python3

import pika  # type: ignore
import sys
import time


def callback(ch, method, properties, body):
    print(" [x] Received %r" % body)
    time.sleep(body.count(b'.'))
    print(" [x] Done")


def main():
    params = pika.ConnectionParameters('localhost')
    connection = pika.BlockingConnection(params)
    channel = connection.channel()
    channel.exchange_declare(exchange="logs",
                             exchange_type="direct")
    result = channel.queue_declare(queue="", exclusive=True)

    severities = sys.argv[1:]
    if not severities:
        sys.stderr.write("Usage: %s [info] [warning] [error]\n" % sys.argv[0])
        sys.exit(1)

    for severity in severities:
        channel.queue_bind(exchange="logs",
                           queue=result.method.queue,
                           routing_key=severity)

    channel.basic_consume(queue=result.method.queue,
                          on_message_callback=callback,
                          auto_ack=True)

    try:
        print(" [*] Waiting for messages. To exit press CTRL+C")
        channel.start_consuming()
    except KeyboardInterrupt:
        print("Finalizando!")
        exit()


if __name__ == "__main__":
    main()
