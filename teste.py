# Imports gerais de bibliotecas padrão
import pickle
from os import listdir
from os.path import isfile, join
from typing import Dict, Tuple, List
from pandas import DataFrame
import plotly.express as px


def process_lane_data() -> Dict[str, DataFrame]:
    """
    Processa os dados de arestas existentes no diretório da simulação.
    """
    result_dir = 'results/cross/1589245768'
    lane_data = DataFrame()
    # Para cada arquivo no diretório
    for f in listdir(result_dir):
        full_path = join(result_dir, f)
        if isfile(full_path):
            # Se o arquivo é de aresta, processa os dados
            if f == "lanes.pickle":
                with open(full_path, "rb") as fileref:
                    lane_data = pickle.load(fileref)
    # Retorna os dados de arestas
    return lane_data


if __name__ == "__main__":
    lane_data = process_lane_data()
    fig = px.line(lane_data,
                  x='sampling_time',
                  y='average_occupancy',
                  color='lane_id')
    fig.write_image("results/cross/1589245768/lanes.pdf")
