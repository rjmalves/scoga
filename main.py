#!/usr/bin/env python3

# Imports gerais de módulos padrão
import time
# Imports específicos da aplicação
from system.simulation import Simulation

#   - Exchanges: clock, detectores, semaphores, setpoints.
#   - Binding / routing keys: IDs (do detector, controlador, semáforo, etc.)


def main():
    try:
        sim = Simulation("config/simulations/cross.json")
        # Inicia a simulação
        sim.start()
        while sim.is_running():
            time.sleep(0.1)
        # Quando terminar, exporta os históricos
        sim.export_histories()
    except KeyboardInterrupt:
        # Se for interrompida, exporta mesmo assim tudo o que ocorreu
        sim.export_histories()
        print("Simulação Finalizada!")
        return 0


if __name__ == "__main__":
    main()
