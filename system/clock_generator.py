# Gerador de relógio para a simulação de tráfego com controle em tempo real
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 11 de Março de 2020

# Imports gerais de módulos padrão
import time
import pika  # type: ignore
from pika import spec  # type: ignore
import sys
import threading
import traceback
# Imports de módulos específicos da aplicação


class ClockGenerator:
    """
    Gerador de relógio para os dispositivos da simulação. É responsável por:
      - Criar a queue de relógio (clock_tick)
      - Contar quantos passos da simulação já foram executados e, a partir
      deste dado e da duração do passo de tempo da simulação, emitir uma
      mensagem para avisar os dispositivos inscritos que se passou um segundo.
    """

    def _on_connection_open(self, connection: pika.SelectConnection):
        """
        """
        self.channel = connection.channel(
            on_open_callback=self._on_channel_open)

    def _on_channel_open(self, channel):
        """
        """
        channel.confirm_delivery(ack_nack_callback=
            self._delivery_confirm)
        # Declara a exchange de relógio
        channel.exchange_declare(exchange="clock_tick",
                                 exchange_type="fanout")

    def _delivery_confirm(self, frame):
        """
        """
        if isinstance(frame.method, spec.Basic.Ack):
            pass
        else:
            raise Exception("Mensagem CLOCK_TICK não recebida pelo RabbitMQ")
            

    def __init__(self, simulation_time_step: float):
        # Define os parâmetros da conexão (local do broker RabbitMQ)
        self.parameters = pika.ConnectionParameters(host="localhost")
        self.connection = pika.SelectConnection(
            parameters=self.parameters,
            on_open_callback=self._on_connection_open)
        
        self.clk_thread = threading.Thread(target=self.clock_control,
                                           daemon=True)
        try:
            self.clk_thread.start()
            time.sleep(1)
        except:
            self.clk_thread.join()
            traceback.print_exc()
        finally:
            # Contador de passos de simulação e do instante de tempo
            # atual na simulação
            self.simulation_time_step = simulation_time_step
            self.num_simulation_steps: int = 0
            self.current_sim_time: float = 0.0

    def clock_control(self):
        """
        """
        try:
            self.connection.ioloop.start()
        except:
            self.connection.close()

    def clock_tick(self):
        # Aumenta o contador de passos
        self.num_simulation_steps += 1
        # Atualiza o tempo atual na simulação
        self.current_sim_time += self.simulation_time_step
        current_time = self.current_sim_time
        # Se o valor do tempo atual, em segundos, é maior que o anterior
        if abs(int(round(current_time)) - current_time) < 1e-3:
            self.channel.basic_publish(exchange="clock_tick",
                                       routing_key="",
                                       body=str(self.current_sim_time))

    def __del__(self):
        """
        Como esta classe instancia threads, deve ter os .join() explícitos no
        destrutor.
        """
        self.clk_thread.join()


if __name__ == "__main__":
    # Executa testes básicos com passo de 1 segundo. Para conferir é
    # necessário executar algum dispositivo, como o controlador.

    # Valores default de teste
    sim_time_step = 1.0
    sim_step_duration = 1.0
    # Valores opcionais caso o usuário informe via linha de comando
    if len(sys.argv) == 2:
        sim_time_step = float(sys.argv[1])
        sim_step_duration = 1.0
    elif len(sys.argv) >= 3:
        sim_time_step = float(sys.argv[1])
        sim_step_duration = float(sys.argv[2])

    clock_gen = ClockGenerator(sim_time_step)
    try:
        print("Iniciando o teste do ClockGenerator!\n" +
              "Tempo na simulação a cada passo: {}\n".format(sim_time_step) +
              "Tempo para computar um passo: {}".format(sim_step_duration))
        reset_time = time.time()
        while True:
            current_time = time.time()
            if current_time - reset_time > sim_step_duration:
                clock_gen.clock_tick()
                reset_time = current_time
            else:
                time.sleep(0.1)
    except KeyboardInterrupt:
        print("Finalizando o teste do ClockGenerator!")
        exit(0)
