# Modelo de controlador para a simulação de tráfego com controle em tempo real
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 11 de Março de 2020

# Imports gerais de módulos padrão
import pika  # type: ignore
import sys
import time
import threading
import traceback
# Imports de módulos específicos da aplicação


class Controller:
    """
    Responsável por conferir a passagem de tempo e avançar com o plano
    semafórico, obedecendo aos setpoints de controle.
    """

    def __init__(self, controller_id: str):
        self.id = controller_id
        self.current_time = 0.0
        self.is_started = False

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
        self.clock_queue_name = declare_result.method.queue
        self.channel.queue_bind(exchange="clock_tick",
                                queue=self.clock_queue_name)

    def start(self):
        """
        Inicializa as threads do controlador. A partir deste momento ele:
        1) Se inscreve no exchange 'clock_tick' e passa a ouvir o relógio.
        2) Começa a publicar no exchange 'semaphores' sempre que houver uma
        mudança de estado. TODO
        3) Se inscreve no exchange 'detectors' para poder ouvir quando
        houver detecção. TODO
        4) Se inscreve no exchange 'setpoints' para alterar os seus
        parâmetros de plano conforme ordenado pelo tempo real. TODO
        """
        self.is_started = True
        self.clock_thread = threading.Thread(target=self.clock_listening,
                                             daemon=True)
        self.clock_thread.start()

    def clock_listening(self):
        """
        Função responsável pela thread que está inscrita para receber o tick
        do relógio da simulação.
        """
        # TODO - Substituir por um logging decente.
        print("Controlador {} começando a escutar o relógio!".format(self.id))
        # Toda thread que não seja a principal precisa ter o traceback printado
        try:
            # Faz a inscrição na fila.
            # Como é fanout, não precisa da binding key.
            self.channel.basic_consume(queue=self.clock_queue_name,
                                       on_message_callback=self.clock_callback)
            # Começa a escutar. Como a conexão é bloqueante, trava aqui.
            self.channel.start_consuming()
        except Exception:
            traceback.print_exc()
            self.clock_thread.join()

    def clock_callback(self,
                       ch: pika.adapters.blocking_connection.BlockingChannel,
                       method: pika.spec.Basic.Deliver,
                       property: pika.spec.BasicProperties,
                       body: bytes):
        """
        Função executada logo após uma atualização de relógio por meio do
        gerador de relógio. Atualiza o instante de tempo atual para o
        controlador.
        """
        self.current_time = float(body.decode())
        # TODO - Substituir por um logging decente.
        print("Controller {} - Clock Tick! Instante atual = {}"
              .format(self.id, self.current_time))

    def __del__(self):
        """
        Como esta classe instancia threads, deve ter os .join() explícitos no
        destrutor.
        """
        # Não faz sentido dar join numa thread que não foi iniciada
        if self.is_started:
            self.clock_thread.join()


# Caso de teste do controlador.
# TODO - Substituir por uma rotina de testes decente usando pytest.
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
        controller.start()
        # Aguarda todas as threads serem finalizadas
        while threading.active_count() > 0:
            time.sleep(1.0)
    except KeyboardInterrupt:
        # TODO - Substituir por um logging decente.
        print("Finalizando o teste do controlador {}!".format(controller_id))
        exit(0)
