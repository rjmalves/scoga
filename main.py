#!/usr/bin/env python3

import pika  # type: ignore
import sys

# ESBOÇO DA IMPLEMENTAÇÃO EM RABBITMQ
# CADA CONTROLLER É UM "MICROSSERVIÇO"
# - Cada controller tem uma queue.
# - Cada controller tem seus tópicos. Tem uma thread que fica escutando os
# detectores, tem outra que calcula os dados de tráfego, outra que conta o
# tempo pra passar o plano, outra que recebe os setpoints, etc.
# - Cada funcionalidade do controller pode ser uma exchange e os objetos de
# interesse podem ser binding keys.
# - Por exemplo, se um controller lê dados do detector de ID 125 da simulação,
# então o GATE precisa publicar algo na exchange "detectors" com routing key
# "125". Por outro lado, se o controlador detecta uma mudança de estado do
# semáforo 37 devido ao plano, então ele publica uma mensagem na exchange.
# "semaphores" com routing key "37". No corpo da mensagem, deve estar o novo
# estado do semáforo. Da mesma forma, no caso do detector deve estar o novo
# estado do detector.
# - Para o fornecimento de relógio, deve existir uma exchange "clock", onde a
# thread do GATE responsável por dar passos na simulação irá avisar sempre que
# se passar 1 segundo. Todos os controladores devem estar inscritos.
# - Deve existir outra exchange "setpoints", onde serão enviados os setpoints
# de execução para cada controlador. Estes serão obtidos a partir do serviço
# de otimização, que irá publicar sempre que tiver uma informação nova. Da
# mesma forma dos anteriores, a routing key será o ID do controlador.

# Resumindo:
#   - Exchanges: clock, detectores, semaphores, setpoints.
#   - Binding / routing keys: IDs (do detector, controlador, semáforo, etc.)


def main():
    severity = "".join(sys.argv[1]) if len(sys.argv) > 1 else "info"
    print(severity)
    message = " ".join(sys.argv[2:]) or "Hello, World!"
    print(message)
    params = pika.ConnectionParameters('localhost')
    connection = pika.BlockingConnection(params)
    channel = connection.channel()
    channel.basic_publish(exchange="logs",
                          routing_key=severity,
                          body=message)
    print(" [x] Sent %r:%r" % (severity, message))


if __name__ == "__main__":
    main()
