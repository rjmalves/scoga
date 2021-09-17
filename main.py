#!/usr/bin/env python3

# Imports gerais de módulos padrão
from multiprocessing import Process, Queue

from model.traffic.controller import Controller
from system.optimization.scoot import EnumOptimizationMethods
import time
from typing import Dict
# Imports específicos da aplicação
from system.simulation import Simulation
from system.simulation import console


def apresentacao():
    """
    """
    desc = ("[bold]AVALIAÇÃO DE TÉCNICAS DE " +
            "[bold blue on white]CONTROLE SEMAFÓRICO" +
            "[bold white on black] :vertical_traffic_light: PARA " +
            "[bold white on blue]CONTROLE DE TRÁFEGO"
            + "[bold on black] :car:")
    console.print(desc)
    autor = ("AUTOR: [bold]ROGERIO JOSÉ MENEZES ALVES - " +
             "[bold white on blue]rogerioalves.ee@gmail.com")
    console.print(autor)
    propos = ("MESTRADO EM ENGENHARIA ELÉTRICA - " +
              "[bold]UFES. 2018 - 2020")
    console.print(propos)
    data = ("[underline]Vila Velha - ES - Brasil")
    console.print(data)
    console.rule("")


def inicia_controlador(id: str,
                       cfg: str,
                       q: Queue):
    """
    """
    ctrl = Controller(id)
    ctrl.start(cfg)
    console.log(f"Iniciando controlador {id}")
    q.get(block=True)
    console.log(f"Terminando controlador {id}")
    ctrl.end()


def main():
    console.rule("[bold]SIMULAÇÃO DE CONTROLE DE TRÁFEGO EM TEMPO REAL")
    apresentacao()
    # Importa os arquivos
    sim = Simulation("config/simulations/corridor.json",
                     EnumOptimizationMethods.ITLC)

    # Inicia os controladores
    processes: Dict[str, Process] = {}
    queues: Dict[str, Queue] = {}
    for ctrl_id, ctrl_cfg in sim.controller_configs.items():
        queues[ctrl_id] = Queue()
        processes[ctrl_id] = Process(target=inicia_controlador,
                                     args=(ctrl_id,
                                           ctrl_cfg,
                                           queues[ctrl_id]
                                           ))
        processes[ctrl_id].start()

    # Aguarda um tempo até os controladores serem iniciados
    time.sleep(2)

    # Inicia a simulação
    try:
        sim.start()
        while sim.is_running():
            time.sleep(1e-6)
    finally:
        # Termina os controladores
        for _, q in queues.items():
            q.put("")
        for _, p in processes.items():
            p.join()
        sim.end()
        sim.export_histories()
        console.rule("[bold]FIM DA EXECUÇÃO")
        return 0


if __name__ == "__main__":
    main()
