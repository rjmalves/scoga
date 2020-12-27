#!/usr/bin/env python3

# Imports gerais de módulos padrão
import time
import coloredlogs  # type: ignore
# Imports específicos da aplicação
from system.simulation import Simulation

# Configurações do logger utilizados
coloredlogs.install(level="INFO",
                    fmt='%(asctime)s,%(msecs)03d %(hostname)s' +
                        ' %(name)s[%(lineno)d] %(levelname)s: %(message)s')


def main():
    # Importa os arquivos
    sim = Simulation("config/simulations/manhattan3.json")
    try:
        # Inicia a simulação
        sim.start()
        while sim.is_running():
            time.sleep(0.1)
        # Quando terminar, exporta os históricos
        sim.export_histories()
    except KeyboardInterrupt:
        # Se for interrompida, exporta mesmo assim tudo o que ocorreu
        sim.export_histories()
        return 0


if __name__ == "__main__":
    main()
