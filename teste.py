# Imports gerais de bibliotecas padrão
import pickle
from os import listdir
from os.path import isfile, join
from typing import Dict, Tuple, List
from pandas import DataFrame
import plotly.express as px


def process_tl_data() -> Dict[str, DataFrame]:
    """
    Processa os dados de arestas existentes no diretório da simulação.
    """
    result_dir = 'results/cross/1589251483'
    tl_data = DataFrame()
    # Para cada arquivo no diretório
    for f in listdir(result_dir):
        full_path = join(result_dir, f)
        if isfile(full_path):
            # Se o arquivo é de aresta, processa os dados
            if f == "trafficlights.pickle":
                with open(full_path, "rb") as fileref:
                    tl_data = pickle.load(fileref)
    # Retorna os dados de arestas
    return tl_data


if __name__ == "__main__":
    tl_data = process_tl_data()
    tl_data['y'] = 1.0
    is_01 = tl_data['tl_id'] == '0-1'
    print(tl_data[is_01])
    fig = px.area(tl_data,
                  x='sampling_time',
                  y='y',
                  color='state')
    fig.write_image("results/cross/1589251483/trafficlights.pdf")
