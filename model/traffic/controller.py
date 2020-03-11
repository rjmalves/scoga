# Modelo de controlador para a simulação de tráfego com controle em tempo real
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 11 de Março de 2020

# Imports gerais de módulos padrão
import pika  # type: ignore
import sys
# Imports de módulos específicos da aplicação


class Controller:
    """
    Responsável por conferir a passagem de tempo e avançar com o plano
    semafórico, obedecendo aos setpoints de controle.
    """

    def __init__(self, controller_id: str):
        self.id = controller_id
        self.current_time = 0.0

        # Define os parâmetros da conexão (local do broker RabbitMQ)
        self.parameters = pika.ConnectionParameters(host="localhost")
        # Cria uma conexão com o broker bloqueante
        self.connection = pika.BlockingConnection(self.parameters)
        # Cria um canal dentro da conexão
        self.channel = self.connection.channel()
        # Declara as exchanges
        self.channel.exchange_declare(exchange="clock_tick",
                                      exchange_type="fanout")
        # Cria as queues e realiza um bind no canal
        declare_result = self.channel.queue_declare(queue="", exclusive=True)
        print(type(declare_result))
        self.clock_queue_name = declare_result.method.queue
        self.channel.queue_bind(exchange="clock_tick",
                                queue=self.clock_queue_name)

    # Função para escutar alterações de relógio
    def clock_listening(self):
        print("Controlador {} começando a escutar o relógio!".format(self.id))
        # Faz a inscrição na fila. Como é fanout, não precisa da binding key.
        self.channel.basic_consume(queue=self.clock_queue_name,
                                   on_message_callback=self.clock_callback)
        # Começa a escutar. Como a conexão é bloqueante, trava aqui.
        self.channel.start_consuming()

    # Função de callback para quando receber um tick
    def clock_callback(self, ch, method, property, body: bytes):
        self.current_time = float(body.decode())
        print("Controller {} - Clock Tick! Instante atual = {}"
              .format(self.id, self.current_time))


# Caso de teste do controlador.
if __name__ == "__main__":
    try:
        # Confere se recebeu pelo menos dois argumentos na linha de comando
        if len(sys.argv) < 2:
            print("Por favor, informe um ID.")
            exit(0)
        # Assume que o parâmetro passado por linha de comando é o id
        controller_id = str(sys.argv[1])
        controller = Controller(controller_id)
        # Começa a escutar o relógio
        controller.clock_listening()
    except KeyboardInterrupt:
        print("Finalizando o teste do controlador {}!".format(controller_id))
        exit(0)
