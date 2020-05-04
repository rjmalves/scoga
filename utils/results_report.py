# Gerador de relatórios de uma simulação de tráfego
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 26 de Março de 2020

# Imports gerais de bibliotecas padrão
import sys
import pickle
from os import listdir
from os.path import isfile, join
import matplotlib.pyplot as plt
from typing import Dict, Tuple, List


class Reporter:
    """
    Classe responsável por gerar os reports de uma simulação.
    """
    def __init__(self, result_dir: str):
        self.result_dir = result_dir

    def process_tl_data(self) -> Dict[str, Tuple[List[float], List[int]]]:
        """
        Processa os dados de semáforos existentes no diretório da simulação.
        """
        tl_data_dict: Dict[str, Tuple[List[float], List[int]]] = {}
        # Para cada arquivo no diretório
        for f in listdir(result_dir):
            full_path = join(result_dir, f)
            if isfile(full_path):
                # Se o arquivo é de semáforo, processa os dados
                if "trafficlight" in f and "pickle" in f:
                    with open(full_path, "rb") as fileref:
                        data = pickle.load(fileref)
                        tl_name = f.split(".")[0]
                        times = [d[0] for d in data]
                        states = [d[1] for d in data]
                        # Substitui os valores para plot
                        states = [0.998 if s == 0.0 else s for s in states]
                        states = [0.999 if s == 1.0 else s for s in states]
                        states = [1.000 if s == 2.0 else s for s in states]
                        # Adiciona o elemento
                        tl_data_dict[tl_name] = (times, states)
        # Retorna os dados de semáforos
        return tl_data_dict

    def process_detector_data(self) -> Dict[str,
                                            Tuple[List[float], List[int]]]:
        """
        Processa os dados de detectores existentes no diretório da simulação.
        """
        detector_data_dict: Dict[str, Tuple[List[float], List[int]]] = {}
        # Para cada arquivo no diretório
        for f in listdir(result_dir):
            full_path = join(result_dir, f)
            if isfile(full_path):
                # Se o arquivo é de detector, processa os dados
                if "detector" in f and "pickle" in f:
                    with open(full_path, "rb") as fileref:
                        data = pickle.load(fileref)
                        det_name = f.split(".")[0]
                        times = [d[0] for d in data]
                        states = [d[1] for d in data]
                        detector_data_dict[det_name] = (times, states)
        # Retorna os dados de detector
        return detector_data_dict

    def make_result_plots(self):
        """
        Transforma os dados obtidos do diretório em arquivos com gráficos.
        """
        # Processa os dados de detectores
        det_data = self.process_detector_data()
        # Cria os subplots, compartilhando o eixo do tempo
        n_dets = len(det_data.keys())
        fig, axs = plt.subplots(n_dets, 1, sharex=True)
        axs[n_dets - 1].set_xlabel("Tempo (s)")
        for i, (det_name, data) in enumerate(det_data.items()):
            axs[i].plot(data[0], data[1])
            axs[i].set_title(det_name)
        figname = join(self.result_dir, "") + "detectors.pdf"
        plt.savefig(figname, orientation="portrait", papertype="a4")
        # Processa os dados de semáforos
        tl_data = self.process_tl_data()
        # Cria os plots
        n_tls = len(tl_data.keys())
        fig, axs = plt.subplots(n_tls, 1, sharex=True)
        axs[n_tls - 1].set_xlabel("Tempo (s)")
        for i, (tl_name, data) in enumerate(tl_data.items()):
            axs[i].plot(data[0], data[1])
            # Força a escala do eixo y
            axs[i].set_ylim(bottom=0.0, top=1.0)
            # Faz um preenchimento com as cores
            axs[i].fill_between(data[0],
                                0,
                                data[1],
                                where=[d == 0.998 for d in data[1]],
                                facecolor='red',
                                interpolate=True)
            axs[i].fill_between(data[0],
                                0,
                                data[1],
                                where=[d == 0.999 for d in data[1]],
                                facecolor='yellow',
                                interpolate=True)
            axs[i].fill_between(data[0],
                                0,
                                data[1],
                                where=[d == 1.000 for d in data[1]],
                                facecolor='green',
                                interpolate=True)
            # Da um título
            axs[i].set_title(tl_name)
        figname = join(self.result_dir, "") + "trafficlights.pdf"
        plt.savefig(figname, orientation="portrait", papertype="a4")


if __name__ == "__main__":
    # Retorna imediatamente em caso de não haver diretório
    if len(sys.argv) < 2:
        print("Por favor forneça um diretório válido.")
        exit(1)
    # Processa normalmente
    result_dir = sys.argv[1]
    reporter = Reporter(result_dir)
    reporter.make_result_plots()
