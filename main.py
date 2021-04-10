#!/usr/bin/env python3

# Imports gerais de módulos padrão
from system.optimization.scoot import EnumOptimizationMethods
import time
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


def main():
    console.rule("[bold]SIMULAÇÃO DE CONTROLE DE TRÁFEGO EM TEMPO REAL")
    apresentacao()
    # Importa os arquivos
    sim = Simulation("config/simulations/crossing.json",
                     EnumOptimizationMethods.FixedTime)

    try:
        # Inicia a simulação
        sim.start()
        while sim.is_running():
            time.sleep(1.5e-3)
    finally:
        # sim.stop_communication()
        sim.export_histories()
        console.rule("[bold]FIM DA EXECUÇÃO")
        return 0


if __name__ == "__main__":
    main()
